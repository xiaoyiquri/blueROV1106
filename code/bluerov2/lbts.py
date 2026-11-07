"""Lazy Belief Tree Search, covariance propagation and Monte-Carlo validation."""
from __future__ import annotations
from dataclasses import dataclass
from types import SimpleNamespace
import heapq
import time as _time
import warnings
import numpy as np

from .model import Parameters, parameters, wrap_to_pi, model_metadata
from .dynamics import bluerov2_dynamics, rk4_step
from .kprm import (Roadmap, Edge, attach_query_endpoints, is_position_collision_free,
                   is_trajectory_collision_free, get_kinodynamic_edge)

try:
    import casadi as ca
except Exception: ca = None


def _ns(**kw): return SimpleNamespace(**kw)


def default_lbts_options(model=None):
    model = model or parameters().model
    nx,nu=12,model.num_thrusters
    sigma0=np.r_[ [.08,.08,.08], np.deg2rad([2,2,3]), [.03,.03,.03], np.deg2rad([1,1,1]) ]
    process_std=np.r_[ [.002,.002,.002], np.deg2rad([.05,.05,.08]), [.035,.035,.035], np.deg2rad([.8,.8,1.0]) ]
    sensor_std=np.r_[ [.06,.06,.08], np.deg2rad([1.2,1.2,2.0]), [.025,.025,.035], np.deg2rad([.7,.7,1.0]) ]
    return _ns(
        attach_query=True,
        query=_ns(connection_radius=2.5,k_neighbors=14,try_direct_connection=True,verbose=True),
        initial_covariance=np.diag(sigma0**2), initial_estimation_covariance=.60*np.diag(sigma0**2),
        process_noise_qc=np.diag(process_std**2),
        sensor=_ns(measurement_indices=np.arange(nx),default_std=sensor_std,regions=[]),
        lqr=_ns(Q=np.diag([8,8,8,4,4,4,2,2,2,1,1,1]),R=.05*np.eye(nu),Qf=8*np.diag([8,8,8,4,4,4,2,2,2,1,1,1])),
        chance_constraint=.10,
        chance=_ns(num_samples=200,rng_seed=11,include_nominal_point=True,visualize=False,plot_mode="animate",pause_seconds=.05,covariance_sigma=3.),
        cost=_ns(covariance_weight=0.,covariance_matrix=np.diag([1,1,1,.2,.2,.2,.1,.1,.1,.05,.05,.05])),
        dominance=_ns(cost_tolerance=1e-10,psd_tolerance=1e-9,indices=np.arange(nx)),
        search=_ns(stop_at_first_solution=False,max_expansions=20000,max_popped_edges=100000,max_wall_time=np.inf,max_beliefs_per_vertex=200,bound_tolerance=1e-10,verbose=True),
        output=_ns(include_beliefs=False,include_trajectory_graph=False),
        linearization=_ns(prefer_casadi=True,allow_finite_difference=True,finite_difference_step=1e-6),
        numerics=_ns(min_eigenvalue=0.,psd_tolerance=1e-9),
    )


def _merge_ns(base, override):
    if override is None:return base
    items = override.items() if isinstance(override, dict) else vars(override).items()
    for k,v in items:
        if isinstance(v, dict): v = SimpleNamespace(**v)
        if hasattr(base,k) and isinstance(getattr(base,k),SimpleNamespace) and isinstance(v,SimpleNamespace): _merge_ns(getattr(base,k),v)
        else:setattr(base,k,v)
    return base


def nearest_psd(P, min_eigenvalue=0.):
    P=(np.asarray(P,float)+np.asarray(P,float).T)/2
    vals,vecs=np.linalg.eigh(P); vals=np.maximum(vals,min_eigenvalue)
    return (vecs*vals)@vecs.T


def standard_gaussian_samples(dim, num_samples, seed):
    return np.random.default_rng(seed).standard_normal((dim,num_samples))


@dataclass
class RK4Linearizer:
    mode: str
    model: Any
    function: Any = None
    finite_difference_step: float = 1e-6


def make_rk4_linearizer(model, settings):
    if getattr(settings,"prefer_casadi",True) and ca is not None:
        try:
            x=ca.SX.sym("x_lbts",12); u=ca.SX.sym("u_lbts",model.num_thrusters); dt=ca.SX.sym("dt_lbts")
            # symbolic RK4 is kept here rather than using a separate discretizer.
            def fun_dyn(xx): return bluerov2_dynamics(xx,u,model)
            k1=bluerov2_dynamics(x,u,model);k2=bluerov2_dynamics(x+dt*k1/2,u,model);k3=bluerov2_dynamics(x+dt*k2/2,u,model);k4=bluerov2_dynamics(x+dt*k3,u,model)
            xn=x+dt*(k1+2*k2+2*k3+k4)/6
            F=ca.Function("rk4_linearizer",[x,u,dt],[xn,ca.jacobian(xn,x),ca.jacobian(xn,u)])
            return RK4Linearizer("casadi",model,F,getattr(settings,"finite_difference_step",1e-6))
        except Exception as exc:
            if not getattr(settings,"allow_finite_difference",True): raise
            warnings.warn(f"CasADi linearization unavailable; using finite difference: {exc}")
    if not getattr(settings,"allow_finite_difference",True):raise RuntimeError("CasADi unavailable and finite difference disabled")
    return RK4Linearizer("finite-difference",model,None,getattr(settings,"finite_difference_step",1e-6))


def evaluate_rk4_linearization(linearizer, x, u, dt):
    if linearizer.mode=="casadi":
        vals=linearizer.function(np.asarray(x),np.asarray(u),float(dt))
        return tuple(np.asarray(v,float) for v in vals)
    x=np.asarray(x,float);u=np.asarray(u,float); xn=rk4_step(x,u,dt,linearizer.model); A=np.zeros((12,12));B=np.zeros((12,linearizer.model.num_thrusters));h0=linearizer.finite_difference_step
    for i in range(12):
        h=h0*max(1.,abs(x[i]));d=np.zeros(12);d[i]=h;A[:,i]=(rk4_step(x+d,u,dt,linearizer.model)-rk4_step(x-d,u,dt,linearizer.model))/(2*h)
    for i in range(linearizer.model.num_thrusters):
        h=h0*max(1.,abs(u[i]));d=np.zeros(linearizer.model.num_thrusters);d[i]=h;B[:,i]=(rk4_step(x,u+d,dt,linearizer.model)-rk4_step(x,u-d,dt,linearizer.model))/(2*h)
    return xn,A,B


def measurement_model_at_state(x, sensor):
    indices=np.asarray(sensor.measurement_indices,dtype=int).reshape(-1); m=len(indices); C=np.zeros((m,12)); C[np.arange(m),indices]=1
    std=np.asarray(sensor.default_std,float).reshape(-1)
    R=np.diag((std[indices] if len(std)==12 else std)**2)
    p=np.asarray(x)[:3]
    for region in sensor.regions:
        if np.all(p>=np.asarray(region["min"])) and np.all(p<=np.asarray(region["max"])):
            if "R" in region and region["R"] is not None:R=np.asarray(region["R"],float)
            elif "std" in region and region["std"] is not None:
                s=np.asarray(region["std"],float).reshape(-1);R=np.diag((s[indices] if len(s)==12 else s)**2)
    return C,nearest_psd(R,1e-14)


def collision_mask_samples(samples, world):
    samples=np.asarray(samples);r=world.robot_radius;b=world.position_bounds
    mask=np.any((samples < (b[:,0]+r)[:,None]) | (samples > (b[:,1]-r)[:,None]),axis=0)
    for o in world.obstacles:
        if o["type"]=="sphere":mask |= np.sum((samples-o["center"][:,None])**2,axis=0)<=(o["radius"]+r)**2
        elif o["type"]=="box":mask |= np.all(np.abs(samples-o["center"][:,None]) <= (o["size"][:,None]/2+r),axis=0)
    return mask


def chance_collision_probability(xbar, covariance_sequence, world, Z, include_nominal_point=True, settings=None):
    settings=settings or _ns(stop_threshold=np.inf); K=xbar.shape[1]; risks=np.full(K,np.nan)
    for k in range(K):
        Ppos=nearest_psd(covariance_sequence[k][:3,:3],0.)
        vals,vecs=np.linalg.eigh(Ppos); L=vecs@np.diag(np.sqrt(np.maximum(vals,0)))
        samples=xbar[:3,k,None]+L@Z; mask=collision_mask_samples(samples,world); risk=float(mask.mean())
        if include_nominal_point and not is_position_collision_free(xbar[:3,k],world):risk=1.
        risks[k]=risk
        if not getattr(settings,"visualize",False) and risk>getattr(settings,"stop_threshold",np.inf):break
    valid=risks[np.isfinite(risks)]; return (float(np.max(valid)) if len(valid) else np.inf),risks


def _psd_leq(A,B,tol):
    D=(np.asarray(B)-np.asarray(A));D=(D+D.T)/2
    return np.min(np.linalg.eigvalsh(D)) >= -tol


def belief_dominated(beliefs, new_cost, P, Pt, settings):
    idx=np.asarray(settings.indices,dtype=int); tol=settings.cost_tolerance
    for old in beliefs:
        if old.get("active",True) and old["cost_to_come"]<=new_cost+tol and _psd_leq(old["P"][np.ix_(idx,idx)],P[np.ix_(idx,idx)],settings.psd_tolerance) and _psd_leq(old["Ptilde"][np.ix_(idx,idx)],Pt[np.ix_(idx,idx)],settings.psd_tolerance):return True
    return False


def propagate_belief(edge: Edge, P0, Ptilde0, model, world, options, linearizer=None, gaussian_samples=None):
    if not isinstance(edge, Edge):
        if isinstance(edge, dict):
            edge = Edge(np.asarray(edge["state_trajectory"]), np.asarray(edge["control_trajectory"]), np.asarray(edge["time"]), float(edge.get("cost", 0.)), edge.get("info", {}))
        elif isinstance(edge, (tuple, list)) and len(edge) >= 4:
            info = edge[3] if isinstance(edge[3], dict) else {}
            edge = Edge(np.asarray(edge[0]), np.asarray(edge[2]), np.asarray(info["time"]), float(info.get("cost", 0.)), info)
        else:
            raise TypeError("edge must be an Edge or MATLAB-style (xbar,N,ubar,info) record")
    out={"success":False,"message":"","P":None,"Ptilde":None,"PSequence":[],"PtildeSequence":[],"gains":[],"kalmanGains":[],"A":[],"B":[],"measurementMatrices":[],"measurementCovariances":[],"time":edge.time,"risk":np.inf,"riskByStep":None,"covarianceCost":np.inf}
    xbar,ubar,t=edge.state_trajectory,edge.control_trajectory,edge.time; N=ubar.shape[1]
    if xbar.shape!=(12,N+1) or t.shape!=(N+1,) or np.any(np.diff(t)<=0):out["message"]="invalid edge dimensions/time";return out
    linearizer=linearizer or make_rk4_linearizer(model,options.linearization)
    A=[];B=[]
    for k in range(N):
        _,a,b=evaluate_rk4_linearization(linearizer,xbar[:,k],ubar[:,k],t[k+1]-t[k]);
        if not np.all(np.isfinite(a)) or not np.all(np.isfinite(b)):out["message"]=f"nonfinite Jacobian at {k}";return out
        A.append(a);B.append(b)
    S=options.lqr.Qf.copy();K=[]
    for k in range(N-1,-1,-1):
        H=options.lqr.R+B[k].T@S@B[k];G=B[k].T@S@A[k]
        Kk=np.linalg.pinv(H)@G if np.linalg.cond(H)>1e12 else np.linalg.solve(H,G);K.insert(0,Kk);F=A[k]-B[k]@Kk;S=(options.lqr.Q+Kk.T@options.lqr.R@Kk+F.T@S@F);S=(S+S.T)/2
    P=nearest_psd(P0,options.numerics.min_eigenvalue);Pt=nearest_psd(Ptilde0,options.numerics.min_eigenvalue)
    Ps=[P.copy()];Pts=[Pt.copy()];Ls=[];Cs=[];Rs=[];covint=0.
    for k in range(N):
        dt=t[k+1]-t[k];Phat=nearest_psd(P-Pt,0);Q=options.process_noise_qc*dt;F=A[k]-B[k]@K[k]
        Pnext=A[k]@Pt@A[k].T+F@Phat@F.T+Q;Pminus=A[k]@Pt@A[k].T+Q
        C,R=measurement_model_at_state(xbar[:,k+1],options.sensor);L=(Pminus@C.T)@np.linalg.inv(C@Pminus@C.T+R);I=np.eye(12)-L@C
        Ptn=I@Pminus@I.T+L@R@L.T;Pnext=nearest_psd(Pnext,options.numerics.min_eigenvalue);Ptn=nearest_psd(Ptn,options.numerics.min_eigenvalue)
        diff=(Pnext-Ptn+ (Pnext-Ptn).T)/2
        if np.min(np.linalg.eigvalsh(diff)) < -100*options.numerics.psd_tolerance:out["message"]=f"P-Ptilde not PSD at {k}";return out
        P=nearest_psd(Ptn+nearest_psd(diff,0),options.numerics.min_eigenvalue);Pt=Ptn;Ps.append(P.copy());Pts.append(Pt.copy());Ls.append(L);Cs.append(C);Rs.append(R);covint+=dt*np.trace(options.cost.covariance_matrix@P)
    settings=options.chance; Z=gaussian_samples if gaussian_samples is not None else standard_gaussian_samples(3,settings.num_samples,settings.rng_seed); chance=SimpleNamespace(**vars(settings)); chance.stop_threshold=options.chance_constraint
    risk,by=chance_collision_probability(xbar,Ps,world,Z,options.chance.include_nominal_point,chance)
    out.update(success=np.all(np.isfinite(P)),P=P,Ptilde=Pt,PSequence=Ps,PtildeSequence=Pts,gains=K,kalmanGains=Ls,A=A,B=B,measurementMatrices=Cs,measurementCovariances=Rs,risk=risk,riskByStep=by,covarianceCost=options.cost.covariance_weight*covint)
    return out


def backward_dijkstra(graph: Roadmap, goal: int):
    n=len(graph.vertices);h=np.full(n,np.inf);h[goal]=0.;q=[(0.,goal)]
    while q:
        d,j=heapq.heappop(q)
        if d!=h[j]:continue
        for i in graph.parents[j]:
            edge_idx=graph.edges[i].index(j);nd=d+graph.edge_costs[i][edge_idx]
            if nd<h[i]:h[i]=nd;heapq.heappush(q,(nd,i))
    return h


def _empty_stats():
    return {k:0 for k in ["popped_edges","expanded_edges","stale_edges","mean_collision_checks","mean_collision_rejected","chance_rejected","dominance_rejected","numerical_rejected","vertex_capacity_rejected","accepted_beliefs","incumbent_updates"]}|{"termination":"","remaining_queue_size":0,"elapsed_seconds":0.,"total_elapsed_seconds":0.,"linearization_mode":""}


def _assemble_result(graph, beliefs, goal_id, success, incumbent, heuristic, options, stats, start, goal, attachment=None):
    result={"success":bool(success),"statistics":stats,"roadmap":graph if options.output.include_trajectory_graph else None,"model":graph.param.model,"world":graph.param.world,"options":options,"metadata":model_metadata(graph.param.model),"start_idx":start,"goal_idx":goal,"path":[],"heuristic":heuristic,"attachment_report":attachment}
    if not success:return result
    v,bi=incumbent; nodes=[]; edges=[]
    while v!=start or bi!=0:
        node=beliefs[v][bi]; nodes.append((v,bi));
        if node["parent"] is None:break
        pv,pbi=node["parent"];edges.append(node["incoming"]);v,bi=pv,pbi
    edges=edges[::-1];nodes=nodes[::-1];result["path"]=[start]+[n[0] for n in nodes]
    result["vertexPath"]=np.asarray(result["path"],dtype=int)
    result["beliefIndexPath"]=np.asarray([0]+[n[1] for n in nodes],dtype=int)
    result["edgePath"]=np.asarray([[a,b] for a,b in zip(result["path"][:-1],result["path"][1:])],dtype=int)
    X=[];U=[];tt=[];Ks=[];Ls=[];As=[];Bs=[];Cs=[];Rs=[];Ps=[];Pts=[];counts=[];risks=[];tbase=0.
    for edge, node_id in zip(edges,nodes):
        node=beliefs[node_id[0]][node_id[1]]
        prop=node["incoming_propagation"];x=edge.state_trajectory;u=edge.control_trajectory;t=edge.time+tbase
        if not X:X.append(x);U.append(u);tt.append(t);Ps.extend(prop["PSequence"]);Pts.extend(prop["PtildeSequence"])
        else:X.append(x[:,1:]);U.append(u);tt.append(t[1:]);Ps.extend(prop["PSequence"][1:]);Pts.extend(prop["PtildeSequence"][1:])
        Ks.extend(prop["gains"]);Ls.extend(prop["kalmanGains"]);As.extend(prop["A"]);Bs.extend(prop["B"]);Cs.extend(prop["measurementMatrices"]);Rs.extend(prop["measurementCovariances"]);counts.append(u.shape[1]);risks.append(prop["risk"]);tbase=tt[-1][-1]
    result.update(nominalTrajectory=np.concatenate(X,axis=1),controlTrajectory=np.concatenate(U,axis=1),time=np.concatenate(tt),feedbackGains=Ks,kalmanGains=Ls,linearizationA=As,linearizationB=Bs,measurementMatrices=Cs,measurementCovariances=Rs,covarianceSequence=Ps,estimationCovarianceSequence=Pts,edgeStepCounts=counts,edgeRisks=np.asarray(risks),cost=beliefs[goal][incumbent[1]]["cost_to_come"])
    if options.output.include_beliefs:result["beliefs"]=beliefs
    return result


def lbts(graph: Roadmap, start_state_or_index, goal_state_or_index, user_options=None):
    options=default_lbts_options(graph.param.model);_merge_ns(options,user_options)
    if np.isscalar(start_state_or_index) and np.isscalar(goal_state_or_index):
        start,goal=int(start_state_or_index),int(goal_state_or_index);g=graph;attach=None
    else:
        if not options.attach_query: raise ValueError("state queries require attach_query=True")
        g,attach=attach_query_endpoints(graph,start_state_or_index,goal_state_or_index,graph.param.world);start,goal=attach["start_idx"],attach["goal_idx"]
    # Validate all six-dimensional controller interfaces before expensive search.
    if g.param.model.num_thrusters != 6:raise ValueError("LBTS execution requires six thrusters")
    g.validate(six_only=True)
    if options.lqr.R.shape!=(6,6):raise ValueError("LQR R must be (6,6)")
    h=backward_dijkstra(g,goal); beliefs=[[] for _ in g.vertices]
    P0=options.initial_covariance;Pt0=options.initial_estimation_covariance
    beliefs[start]=[{"P":P0,"Ptilde":Pt0,"cost_to_come":0.,"priority":h[start],"parent":None,"incoming":None,"incoming_propagation":None,"active":True}]
    Z=standard_gaussian_samples(3,options.chance.num_samples,options.chance.rng_seed);lin=make_rk4_linearizer(g.param.model,options.linearization);valid=[[0]*len(e) for e in g.edges];q=[]
    for ei,(j,c) in enumerate(zip(g.edges[start],g.edge_costs[start])):
        if np.isfinite(h[j]):heapq.heappush(q,(c+h[j],start,0,j,ei,c))
    stats=_empty_stats();t0=_time.perf_counter();inc=(None,None);inc_cost=np.inf
    while q and stats["expanded_edges"]<options.search.max_expansions and stats["popped_edges"]<options.search.max_popped_edges and _time.perf_counter()-t0<options.search.max_wall_time:
        priority,i,bi,j,ei,nom=heapq.heappop(q);stats["popped_edges"]+=1
        if priority>=inc_cost-options.search.bound_tolerance:stats["termination"]="incumbent_bound_proved";break
        src=beliefs[i][bi]
        if not src["active"]:stats["stale_edges"]+=1;continue
        if valid[i][ei]==0:
            stats["mean_collision_checks"]+=1;valid[i][ei]=1 if is_trajectory_collision_free(g.edge_data[i][ei].state_trajectory,g.param.world) else -1
        if valid[i][ei]<0:stats["mean_collision_rejected"]+=1;continue
        stats["expanded_edges"]+=1;prop=propagate_belief(g.edge_data[i][ei],src["P"],src["Ptilde"],g.param.model,g.param.world,options,lin,Z)
        if not prop["success"]:stats["numerical_rejected"]+=1;continue
        if prop["risk"]>options.chance_constraint:stats["chance_rejected"]+=1;continue
        newcost=src["cost_to_come"]+nom+prop["covarianceCost"]
        if belief_dominated(beliefs[j],newcost,prop["P"],prop["Ptilde"],options.dominance):stats["dominance_rejected"]+=1;continue
        if len(beliefs[j])>=options.search.max_beliefs_per_vertex:stats["vertex_capacity_rejected"]+=1;continue
        node={"P":prop["P"],"Ptilde":prop["Ptilde"],"cost_to_come":newcost,"priority":newcost+h[j],"parent":(i,bi),"incoming":g.edge_data[i][ei],"incoming_propagation":prop,"active":True};newbi=len(beliefs[j]);beliefs[j].append(node);stats["accepted_beliefs"]+=1
        if j==goal:
            if newcost<inc_cost:inc_cost=newcost;inc=(j,newbi);stats["incumbent_updates"]+=1
            if options.search.stop_at_first_solution:stats["termination"]="first_solution";break
        else:
            for nei,(nj,nc) in enumerate(zip(g.edges[j],g.edge_costs[j])):
                if np.isfinite(h[nj]):heapq.heappush(q,(newcost+nc+h[nj],j,newbi,nj,nei,nc))
    if not stats["termination"]:
        stats["termination"]="queue_empty" if not q else "limit"
    stats["elapsed_seconds"]=_time.perf_counter()-t0;stats["total_elapsed_seconds"]=stats["elapsed_seconds"];stats["remaining_queue_size"]=len(q);stats["linearization_mode"]=lin.mode
    result=_assemble_result(g,beliefs,goal,inc[0] is not None,inc,h,options,stats,start,goal,attach)
    return result


def monte_carlo_path(result: dict, user_options=None):
    o=_ns(num_runs=40,rng_seed=0,plot=False,plot_estimated_mean=True,plot_covariance=True,covariance_sigma=3.,enforce_thrust_limits=True,verbose=True);_merge_ns(o,user_options)
    if not result.get("success",False):raise ValueError("result.success must be true")
    X=result["nominalTrajectory"];U=result["controlTrajectory"];t=result["time"];model=result["model"];steps=U.shape[1];runs=int(o.num_runs);dt=np.diff(t);K=result["feedbackGains"];L=result["kalmanGains"];C=result["measurementMatrices"];R=result["measurementCovariances"];Q_c=result["options"].process_noise_qc
    rng=np.random.default_rng(o.rng_seed);true=np.zeros((12,steps+1,runs));est=np.zeros_like(true);ctrl=np.zeros((model.num_thrusters,steps,runs));
    def root(P):
        vals,vecs=np.linalg.eigh(nearest_psd(P,0));return vecs@np.diag(np.sqrt(np.maximum(vals,0)))
    # Ptilde is estimation-error covariance; P-Ptilde is estimate covariance.
    # Match MC_path.m: estimated0=xbar+hatRoot*z, true0=estimated0+errorRoot*z.
    P0=result["covarianceSequence"][0];Pt0=result["estimationCovarianceSequence"][0];est[:,0,:]=X[:,0,None]+root(nearest_psd(P0-Pt0,0))@rng.standard_normal((12,runs));true[:,0,:]=est[:,0,:]+root(Pt0)@rng.standard_normal((12,runs));sat=0
    for k in range(steps):
        err=est[:,k,:]-X[:,k,None];err[3:6]=wrap_to_pi(err[3:6]);un=U[:,k,None]-K[k]@err;cmd=np.clip(un,model.thrust_min[:,None],model.thrust_max[:,None]) if o.enforce_thrust_limits else un;sat+=np.count_nonzero(abs(cmd-un)>1e-12);ctrl[:,k,:]=cmd
        Q=root(Q_c*dt[k]);Rr=root(R[k]);
        for s in range(runs):
            true[:,k+1,s]=rk4_step(true[:,k,s],cmd[:,s],dt[k],model)+Q@rng.standard_normal(12);prior=rk4_step(est[:,k,s],cmd[:,s],dt[k],model);z=C[k]@true[:,k+1,s]+Rr@rng.standard_normal(C[k].shape[0]);innov=z-C[k]@prior;angle=np.any(np.abs(C[k][:,3:6])>0,axis=1);innov[angle]=wrap_to_pi(innov[angle]);est[:,k+1,s]=prior+L[k]@innov
    if not np.all(np.isfinite(true)) or not np.all(np.isfinite(est)) or not np.all(np.isfinite(ctrl)):
        raise FloatingPointError("nonfinite Monte Carlo integration; use a finer plan time grid")
    err=true-X[:,:,None];err[3:6]=wrap_to_pi(err[3:6]);rmse=np.sqrt(np.mean(np.sum(err[:3]**2,axis=0),axis=1));coll=np.array([not is_trajectory_collision_free(true[:,:,s],result["world"]) for s in range(runs)])
    return {"true_trajectories":true,"estimated_trajectories":est,"control_trajectories":ctrl,"nominal_trajectory":X,"nominal_control":U,"time":t,"collision_mask":coll,"collision_rate":float(coll.mean()),"mean_state_error":err.mean(axis=2),"position_rmse":rmse,"saturated_element_count":int(sat),"control_saturation_rate":sat/ctrl.size,"settings":o}
