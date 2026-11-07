"""Kinodynamic PRM and the MATLAB trajectory-graph data interface."""
from __future__ import annotations
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any
import heapq
import pickle
import warnings
import copy
import numpy as np

from .model import Parameters, ModelParameters, parameters, wrap_to_pi, model_metadata
from .dynamics import bluerov2_dynamics, rotation_zyx, rk4_step

try:
    import casadi as ca
except Exception:  # pragma: no cover
    ca = None


@dataclass
class Edge:
    state_trajectory: np.ndarray
    control_trajectory: np.ndarray
    time: np.ndarray
    cost: float
    info: dict[str, Any] = field(default_factory=dict)

    @property
    def N(self) -> int:
        return self.control_trajectory.shape[1]

    def validate(self, n_thrusters: int = 6) -> None:
        x, u, t = self.state_trajectory, self.control_trajectory, self.time
        if x.shape != (12, u.shape[1] + 1) or u.shape[0] != n_thrusters or t.shape != (u.shape[1] + 1,):
            raise ValueError("edge shapes must be x=(12,N+1), u=(nu,N), t=(N+1)")
        if not np.all(np.isfinite(x)) or not np.all(np.isfinite(u)) or not np.all(np.isfinite(t)) or np.any(np.diff(t) <= 0):
            raise ValueError("edge contains nonfinite data or non-increasing time")

    def to_dict(self) -> dict[str, Any]:
        return {"state_trajectory": self.state_trajectory, "control_trajectory": self.control_trajectory,
                "time": self.time, "cost": float(self.cost), "info": self.info}


@dataclass
class Roadmap:
    vertices: np.ndarray
    edges: list[list[int]]
    parents: list[list[int]]
    edge_costs: list[list[float]]
    edge_data: list[list[Edge]]
    param: Parameters
    sampled_positions: np.ndarray | None = None
    query_info: dict[str, Any] | None = None
    attempt_count: int = 0

    @property
    def edge_count(self):
        return sum(len(e) for e in self.edges)

    def validate(self, six_only: bool = True) -> None:
        if self.vertices.ndim != 2 or self.vertices.shape[1] != 12:
            raise ValueError("Vertices must have shape (n,12)")
        n = len(self.vertices)
        if not (len(self.edges) == len(self.parents) == len(self.edge_costs) == len(self.edge_data) == n):
            raise ValueError("roadmap adjacency containers have inconsistent lengths")
        nu = self.param.model.num_thrusters
        if six_only and nu != 6:
            raise ValueError("the executable six-thruster graph cannot contain an eight-thruster model")
        for i in range(n):
            if not (len(self.edges[i]) == len(self.edge_costs[i]) == len(self.edge_data[i])):
                raise ValueError(f"edge data mismatch at vertex {i}")
            for j, c, edge in zip(self.edges[i], self.edge_costs[i], self.edge_data[i]):
                if not 0 <= j < n or not np.isfinite(c) or c < 0:
                    raise ValueError("invalid edge index/cost")
                edge.validate(nu)


def weighted_state_distance(x: np.ndarray, Y: np.ndarray, weights: np.ndarray) -> np.ndarray:
    x = np.asarray(x, float).reshape(12)
    Y = np.asarray(Y, float)
    if Y.ndim != 2 or Y.shape[1] != 12:
        raise ValueError("Y must have shape (n,12)")
    d = Y - x
    d[:, 3:6] = wrap_to_pi(d[:, 3:6])
    return np.sqrt(np.sum((d*np.asarray(weights).reshape(1,12))**2, axis=1))


def is_position_collision_free(position: np.ndarray, world) -> bool:
    p = np.asarray(position, float).reshape(3)
    r = float(world.robot_radius); b = np.asarray(world.position_bounds)
    if np.any(p < b[:, 0] + r) or np.any(p > b[:, 1] - r):
        return False
    for o in world.obstacles:
        if o["type"] == "sphere":
            if np.linalg.norm(p-o["center"]) <= o["radius"] + r: return False
        elif o["type"] == "box":
            if np.all(np.abs(p-o["center"]) <= o["size"]/2+r): return False
    return True


def is_trajectory_collision_free(xbar: np.ndarray, world) -> bool:
    xbar = np.asarray(xbar, float)
    if xbar.shape[1] == 1: return is_position_collision_free(xbar[:3,0], world)
    for a, b in zip(xbar[:3,:-1].T, xbar[:3,1:].T):
        count = max(1, int(np.ceil(np.linalg.norm(b-a)/world.collision_step)))
        for q in range(count+1):
            if not is_position_collision_free(a + q/count*(b-a), world): return False
    return True


def sample_blue_rov2_states(param: Parameters) -> tuple[np.ndarray, np.ndarray]:
    rng = np.random.default_rng(param.prm.rng_seed)
    b = param.world.position_bounds; r = param.world.robot_radius
    positions: list[np.ndarray] = []
    attempts = 0
    # Offline graph deliberately ignores configured obstacles, as in MATLAB.
    while len(positions) < param.prm.num_positions and attempts < param.prm.max_sample_attempts:
        attempts += 1
        q = b[:,0] + r + rng.random(3)*(b[:,1]-b[:,0]-2*r)
        if np.any(q > b[:,1]-r): continue
        if positions and min(np.sum((np.asarray(positions)-q)**2, axis=1)) < param.prm.min_position_separation**2: continue
        positions.append(q)
    if len(positions) < 2:
        raise RuntimeError(f"accepted only {len(positions)} positions")
    pos = np.asarray(positions)
    rows=[]
    for i, p in enumerate(pos):
        delta = pos-p; dist=np.linalg.norm(delta,axis=1)
        near=np.flatnonzero((dist < param.prm.connection_radius)&(dist>.01))
        for j in near:
            direction=delta[j]/dist[j]
            yaw=np.arctan2(direction[1],direction[0])
            pitch=-np.arctan2(direction[2],np.hypot(direction[0],direction[1]))
            pitch=np.clip(pitch,*param.prm.pitch_range)
            roll=rng.uniform(*param.prm.roll_range)
            R=rotation_zyx(roll,pitch,yaw)
            vel=R.T@(param.prm.sample_speed*direction)
            omega=param.prm.sample_angular_rate*(2*rng.random(3)-1)
            rows.append(np.r_[p,roll,pitch,yaw,vel,omega])
    if not rows: raise RuntimeError("no position pairs are within connection_radius")
    return np.asarray(rows), pos


def mean_control(x0: np.ndarray, xf: np.ndarray, param: Parameters):
    """CasADi direct multiple shooting local connector.

    Returns ``(xbar, cost, N, ubar, info)``.  Infeasible IPOPT calls return an
    empty trajectory and ``success=False`` exactly as the MATLAB connector.
    """
    if ca is None: raise RuntimeError("CasADi is required for mean_control")
    x0=np.asarray(x0,float).reshape(12); xf=np.asarray(xf,float).reshape(12)
    model=param.model; ocp=param.ocp; N=int(ocp.N); nx=12; nu=model.num_thrusters
    target=xf.copy(); target[3:6]=x0[3:6]+wrap_to_pi(xf[3:6]-x0[3:6])
    opti=ca.Opti(); X=opti.variable(nx,N+1); U=opti.variable(nu,N); T=opti.variable()
    p0=opti.parameter(nx); pf=opti.parameter(nx); opti.set_value(p0,x0); opti.set_value(pf,target)
    dt=T/N; opti.subject_to(X[:,0]==p0); opti.subject_to(X[:,-1]==pf)
    opti.subject_to(opti.bounded(ocp.Tmin,T,ocp.Tmax))
    for k in range(N):
        xk=X[:,k]; uk=U[:,k]
        k1=bluerov2_dynamics(xk,uk,model); k2=bluerov2_dynamics(xk+dt*k1/2,uk,model)
        k3=bluerov2_dynamics(xk+dt*k2/2,uk,model); k4=bluerov2_dynamics(xk+dt*k3,uk,model)
        opti.subject_to(X[:,k+1]==xk+dt*(k1+2*k2+2*k3+k4)/6)
    lower=np.tile(model.thrust_min[:,None],(1,N)); upper=np.tile(model.thrust_max[:,None],(1,N))
    opti.subject_to(opti.bounded(ca.DM(lower).reshape((-1,1)), ca.vec(U), ca.DM(upper).reshape((-1,1))))
    lv=np.tile(ocp.max_linear_velocity,(N+1,1)).T; av=np.tile(ocp.max_angular_velocity,(N+1,1)).T
    opti.subject_to(opti.bounded(ca.DM(-lv).reshape((-1,1)),ca.vec(X[6:9,:]),ca.DM(lv).reshape((-1,1))))
    opti.subject_to(opti.bounded(ca.DM(-av).reshape((-1,1)),ca.vec(X[9:12,:]),ca.DM(av).reshape((-1,1))))
    opti.subject_to(opti.bounded(-ocp.max_pitch,X[4,:].T,ocp.max_pitch))
    effort=0
    for k in range(N): effort += U[:,k].T@ca.DM(ocp.R)@U[:,k]
    slew=0
    for k in range(N-1): slew += ca.sumsqr(U[:,k+1]-U[:,k])
    objective=ocp.w_time*T+dt*effort+ocp.w_delta_u*slew; opti.minimize(objective)
    opts={"expand":True,"print_time":ocp.verbose,"ipopt":{"print_level":5 if ocp.verbose else 0,"max_iter":ocp.ipopt_max_iter,"tol":ocp.acceptable_tol,"acceptable_tol":10*ocp.acceptable_tol,"sb":"yes"}}
    opti.solver("ipopt",opts)
    alpha=np.linspace(0,1,N+1); Xguess=x0[:,None]+(target-x0)[:,None]*alpha
    distance=np.linalg.norm(target[:3]-x0[:3]); Tg=np.clip(distance/max(ocp.v_nominal,1e-3),ocp.Tmin,ocp.Tmax)
    hover_tau=hover_restoring(x0,model); hover_u=np.clip(model.allocation_pseudoinverse@hover_tau,model.thrust_min,model.thrust_max)
    opti.set_initial(X,Xguess); opti.set_initial(U,np.tile(hover_u[:,None],(1,N))); opti.set_initial(T,Tg)
    info={"success":False,"return_status":"","T":np.nan,"time":None,"cost":np.inf,"message":""}
    try:
        sol=opti.solve(); xbar=np.asarray(sol.value(X)); ubar=np.asarray(sol.value(U)); cost=float(sol.value(objective)); tf=float(sol.value(T))
        info.update(success=True,T=tf,time=np.linspace(0,tf,N+1),cost=cost,return_status=str(sol.stats().get("return_status","Solve_Succeeded")))
        return xbar,cost,N,ubar,info
    except Exception as exc:
        info["message"]=str(exc)
        try: info["return_status"]=str(opti.stats().get("return_status","exception"))
        except Exception: info["return_status"]="exception"
        return np.empty((0,0)),np.inf,N,np.empty((nu,0)),info


def hover_restoring(x: np.ndarray, model: ModelParameters) -> np.ndarray:
    xz=np.asarray(x,float).copy(); xz[6:]=0
    acc=bluerov2_dynamics(xz,np.zeros(model.num_thrusters),model)[6:]
    return -model.mass_matrix@acc


def _direction_compatible(a,b,param: Parameters) -> bool:
    delta=b[:3]-a[:3]; d=np.linalg.norm(delta)
    if d < param.prm.min_edge_position_distance:return False
    direction=delta/d
    va=rotation_zyx(*a[3:6])@a[6:9]; vb=rotation_zyx(*b[3:6])@b[6:9]
    sa,sb=np.linalg.norm(va),np.linalg.norm(vb)
    caa=1 if sa<=param.prm.direction_speed_tolerance else float(va@direction/sa)
    cab=1 if sb<=param.prm.direction_speed_tolerance else float(vb@direction/sb)
    return caa>=param.prm.source_direction_cosine and cab>=param.prm.target_direction_cosine


def build_kinodynamic_prm(param: Parameters | None = None, *, solve_edges: bool = True) -> Roadmap:
    param=param or parameters(); V,pos=sample_blue_rov2_states(param); n=len(V)
    edges=[[] for _ in range(n)]; parents=[[] for _ in range(n)]; costs=[[] for _ in range(n)]; data=[[] for _ in range(n)]
    attempts=0
    for i in range(n):
        pd=np.linalg.norm(V[:,:3]-V[i,:3],axis=1)
        for j in np.flatnonzero((pd<param.prm.connection_radius)&(pd>=param.prm.min_edge_position_distance)):
            if not _direction_compatible(V[i],V[j],param):continue
            attempts += 1
            if not solve_edges: continue
            xbar,cost,N,u,info=mean_control(V[i],V[j],param)
            if not info["success"] or not np.isfinite(cost):continue
            edge=Edge(xbar,u,np.asarray(info["time"]),cost,info); edges[i].append(int(j)); costs[i].append(cost); data[i].append(edge); parents[j].append(i)
    g=Roadmap(V,edges,parents,costs,data,param,pos,attempt_count=attempts); g.validate(six_only=False)
    if param.prm.save_file:
        save_trajectory_graph(param.prm.save_file,g)
    return g


def get_kinodynamic_edge(roadmap: Roadmap, source: int, target: int) -> Edge | None:
    for j,e in zip(roadmap.edges[source],roadmap.edge_data[source]):
        if j==target:return e
    return None


def save_trajectory_graph(path: str | Path, roadmap: Roadmap) -> None:
    roadmap.validate(six_only=True)
    payload={"format":"bluerov2-python-trajectory-graph-v1","metadata":model_metadata(roadmap.param.model),"roadmap":roadmap}
    with open(path,"wb") as f: pickle.dump(payload,f,protocol=pickle.HIGHEST_PROTOCOL)


def load_trajectory_graph(path: str | Path, *, allow_heavy_reference: bool=False) -> Roadmap:
    path=Path(path)
    if path.suffix.lower()==".mat":
        raise ValueError("MAT trajectory graphs are legacy MATLAB data; export/convert them explicitly before Python execution. Eight-thruster data is rejected.")
    with open(path,"rb") as f: payload=pickle.load(f)
    g=payload.get("roadmap",payload) if isinstance(payload,dict) else payload
    if not isinstance(g,Roadmap): raise ValueError("invalid trajectory graph file")
    if g.param.model.num_thrusters != 6 and not allow_heavy_reference: raise ValueError("eight-thruster graph cannot be loaded by six-thruster execution")
    g.validate(six_only=not allow_heavy_reference); return g


def attach_query_endpoints(roadmap: Roadmap, start_state: np.ndarray, goal_state: np.ndarray, world=None):
    # MATLAB passes the struct by value; preserve the reusable offline roadmap.
    g=copy.deepcopy(roadmap)
    start=np.asarray(start_state,float).reshape(12); goal=np.asarray(goal_state,float).reshape(12)
    world=world or g.param.world
    if not is_position_collision_free(start[:3],world) or not is_position_collision_free(goal[:3],world): raise ValueError("start or goal is in collision/outside workspace")
    base=len(g.vertices); g.vertices=np.vstack([g.vertices,start,goal]);
    g.edges.extend([[],[]]); g.parents.extend([[],[]]); g.edge_costs.extend([[],[]]); g.edge_data.extend([[],[]])
    sidx,gidx=base,base+1
    def connect(src_idx, candidates, outgoing):
        if not candidates:return
        d=weighted_state_distance(g.vertices[src_idx],g.vertices[candidates],g.param.metric_weights)
        order=[candidates[k] for k in np.argsort(d)[:g.param.query.k_neighbors]]
        for j in order:
            a,b=(src_idx,j) if outgoing else (j,src_idx)
            xbar,cost,N,u,info=mean_control(g.vertices[a],g.vertices[b],g.param)
            if info["success"] and is_trajectory_collision_free(xbar,world):
                edge=Edge(xbar,u,np.asarray(info["time"]),cost,info); g.edges[a].append(b);g.edge_costs[a].append(cost);g.edge_data[a].append(edge);g.parents[b].append(a)
    cand=list(range(base));
    near=[j for j in cand if np.linalg.norm(g.vertices[j,:3]-start[:3])<=g.param.query.connection_radius]
    connect(sidx,near,True)
    near=[j for j in cand if np.linalg.norm(g.vertices[j,:3]-goal[:3])<=g.param.query.connection_radius]
    connect(gidx,near,False)
    if g.param.query.try_direct_connection and np.linalg.norm(start[:3]-goal[:3])<=g.param.query.connection_radius:
        xbar,cost,N,u,info=mean_control(start,goal,g.param)
        if info["success"] and is_trajectory_collision_free(xbar,world):
            edge=Edge(xbar,u,np.asarray(info["time"]),cost,info);g.edges[sidx].append(gidx);g.edge_costs[sidx].append(cost);g.edge_data[sidx].append(edge);g.parents[gidx].append(sidx)
    g.query_info={"base_vertex_count":base,"start_idx":sidx,"goal_idx":gidx,"start_state":start,"goal_state":goal,"world":world}
    return g,g.query_info


def collision_filtered_adjacency(roadmap: Roadmap, world):
    e=[[] for _ in roadmap.edges]; c=[[] for _ in roadmap.edge_costs]; blocked=0
    for i in range(len(roadmap.edges)):
        for j,cost,edge in zip(roadmap.edges[i],roadmap.edge_costs[i],roadmap.edge_data[i]):
            if is_trajectory_collision_free(edge.state_trajectory,world):e[i].append(j);c[i].append(cost)
            else:blocked+=1
    return e,c,blocked


def dijkstra_directed(edges: list[list[int]], costs: list[list[float]], start: int, goal: int):
    n=len(edges); dist=np.full(n,np.inf); prev=np.full(n,-1,int); dist[start]=0.; q=[(0.,start)]
    while q:
        d,i=heapq.heappop(q)
        if d!=dist[i]:continue
        if i==goal:break
        for j,c in zip(edges[i],costs[i]):
            nd=d+c
            if nd<dist[j]:dist[j]=nd;prev[j]=i;heapq.heappush(q,(nd,j))
    if not np.isfinite(dist[goal]):return np.inf,[]
    path=[];i=goal
    while i>=0:path.append(int(i));i=prev[i]
    return float(dist[goal]),path[::-1]


def query_kinodynamic_prm(roadmap: Roadmap, start_state, goal_state, world=None):
    g,info=attach_query_endpoints(roadmap,start_state,goal_state,world)
    e,c,blocked=collision_filtered_adjacency(g,world or g.param.world); cost,path=dijkstra_directed(e,c,info["start_idx"],info["goal_idx"])
    if not path:return {"success":False,"cost":np.inf,"path":[],"roadmap":g,"blocked_edge_count":blocked,"message":"no collision-free directed path"}
    X=[];U=[];T=[]; edges_path=[]
    for a,b in zip(path[:-1],path[1:]):
        edge=get_kinodynamic_edge(g,a,b);edges_path.append(edge)
        if not X:X.append(edge.state_trajectory);T.append(edge.time);U.append(edge.control_trajectory)
        else:X.append(edge.state_trajectory[:,1:]);T.append(edge.time[1:]+T[-1][-1]);U.append(edge.control_trajectory)
    return {"success":True,"cost":cost,"path":path,"roadmap":g,"blocked_edge_count":blocked,"nominal_trajectory":np.concatenate(X,axis=1),"control_trajectory":np.concatenate(U,axis=1),"time":np.concatenate(T),"edges":edges_path}

