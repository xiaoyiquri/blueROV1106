"""Reproducible comparison with original .m numerical kernels, using MATLAB or GNU Octave.
No planning/control source is edited. Six-input FD dimension substitutions live only in a temp test harness.
"""
import argparse
from pathlib import Path
import hashlib
import json
import re
import shutil
import subprocess
import sys
import tempfile
import numpy as np
from scipy.io import savemat,loadmat
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bluerov2.model import parameters,heavy_reference_parameters,World,wrap_to_pi
from bluerov2.dynamics import bluerov2_dynamics,rk4_step
from bluerov2.kprm import Edge,Roadmap
from bluerov2.lbts import (default_lbts_options,make_rk4_linearizer,evaluate_rk4_linearization,
                          propagate_belief,chance_collision_probability,backward_dijkstra)
ROOT=Path(__file__).resolve().parents[1]


def matlab_model(m):
    names={'added_mass':'addedMass','linear_damping':'linearDamping','quadratic_damping':'quadraticDamping',
           'smooth_abs_eps':'smoothAbsEps','mass_matrix':'massMatrix','mass_matrix_inverse':'massMatrixInverse'}
    result={names.get(k,k):getattr(m,k) for k in ['m','Ig','W','B','rg','rb','added_mass','linear_damping','quadratic_damping','smooth_abs_eps','mass_matrix','mass_matrix_inverse','allocation']}
    for k,v in result.items():
        if np.ndim(v)==1:result[k]=v[:,None]
    return result


def matlab_options(o):
    return {'lqr':{k:np.asarray(v,dtype=float) for k,v in vars(o.lqr).items()},'processNoiseQc':o.process_noise_qc,'numerics':{'minEigenvalue':0.,'psdTolerance':o.numerics.psd_tolerance},
        'sensor':{'measurementIndices':np.arange(1,13),'defaultStd':o.sensor.default_std,'regions':np.empty((0,0))},
        'cost':{'covarianceWeight':o.cost.covariance_weight,'covarianceMatrix':o.cost.covariance_matrix},
        'chance':{'visualize':False,'includeNominalPoint':True},'chanceConstraint':o.chance_constraint}


def main(argv=None):
    a=argparse.ArgumentParser();a.add_argument('--engine',choices=['octave','matlab'],default='octave');a.add_argument('--executable');a.add_argument('--output',type=Path,default=ROOT/'reports/matlab_alignment.json');args=a.parse_args(argv)
    exe=args.executable or shutil.which(args.engine)
    if not exe:raise SystemExit('MATLAB/Octave executable not found; no numerical equivalence claim can be made')
    rng=np.random.default_rng(20261103);models=[parameters().model,heavy_reference_parameters().model]
    states=rng.uniform(-.3,.3,(12,24));states[2,:]+=1.5;states[4,:]=np.linspace(-.7,.7,24)
    forces=[rng.uniform(-8,8,(m.num_thrusters,24)) for m in models];dt=np.linspace(.005,.04,24)
    times=np.array([0.,.03,.11,.16]);X=np.zeros((12,4));X[2]=1.5;X[0]=[0.,.02,.07,.1];X[6]=.1
    U=[rng.uniform(-2,2,(m.num_thrusters,3)) for m in models]
    options=[default_lbts_options(m) for m in models]
    for o in options:o.cost.covariance_weight=.2;o.linearization.prefer_casadi=False
    P0=options[0].initial_covariance;Pt0=options[0].initial_estimation_covariance
    Z=rng.normal(size=(3,200));world=World(position_bounds=np.array([[-100,100]]*3),obstacles=[])
    chance_world=World(position_bounds=np.array([[-4,4],[-2.5,2.5],[.5,4]]),obstacles=[{'type':'sphere','center':np.array([.8,.1,1.5]),'radius':.12}])
    chance_x=np.zeros((12,1));chance_x[:3,0]=[.35,0,1.5]
    chance_P=np.eye(12)*.01;chance_P[:3,:3]=[[.04,.015,0],[.015,.09,.01],[0,.01,.16]]
    fstate=rng.uniform(-.1,.1,(12,10));fref=rng.uniform(-.1,.1,(12,10));fstate[3:6,:]+=np.array([2*np.pi,-2*np.pi,2*np.pi])[:,None]
    fK=rng.uniform(-10,10,(6,12,10));fu=rng.uniform(-40,40,(6,10))
    edges=[[1,2],[2,3],[3,4],[4],[]];costs=[[3.,1.],[1.,1.],[2.,7.],[1.],[]]
    obj=lambda xs:np.array(xs,dtype=object)
    # Explicit cell arrays, including unequal-size graph adjacency.
    def cells(xs):
        r=np.empty((1,len(xs)),dtype=object)
        for i,x in enumerate(xs):r[0,i]=x
        return r
    mw={'positionBounds':world.position_bounds,'robotRadius':world.robot_radius,'obstacles':np.empty((0,0))}
    cw={'positionBounds':chance_world.position_bounds,'robotRadius':chance_world.robot_radius,
        'obstacles':{'type':'sphere','center':np.array([.8,.1,1.5])[:,None],'radius':.12}}
    data={'models':cells([matlab_model(m) for m in models]),'states':states,'forces':cells(forces),'dt':dt,
          'X':X,'U':cells(U),'times':times,'P0':P0,'Pt0':Pt0,'world':mw,'Z':Z,'options':cells([matlab_options(o) for o in options]),
          'chance_x':chance_x,'chance_P':chance_P,'chance_world':cw,'feedback_state':fstate,'feedback_ref':fref,
          'feedback_K':fK,'feedback_u':fu,'thrust_min':models[0].thrust_min[:,None],'thrust_max':models[0].thrust_max[:,None],
          'graph':{'Vertices':np.zeros((5,12)),'Edges':cells([np.array(e)+1 for e in edges]),'EdgesCost':cells([np.array(c) for c in costs])}}
    report={'engine':args.engine,'native_matlab_run':args.engine=='matlab','random_input_seed':20261103,
        'scope':'24 nonlinear dynamics/RK4 cases each for six and original eight inputs; finite-difference Jacobians; three unequal intervals of original Riccati/Joseph belief propagation; six-input wrapped/saturated feedback algebra; original backward-Dijkstra; shared-sample chance constraint case',
        'harness_dimension_only_replacements':{'zeros(12, 8)':'zeros(12, numel(u))','for i = 1:8':'for i = 1:numel(u)','zeros(8,1)':'zeros(numel(u),1)'},
        'limitations':['Original MATLAB executable and its CasADi/IPOPT OCP and full search were not run unless engine=matlab; original hardware executor is fixed at eight controls.','Six-input comparison uses identical Python six-input geometry injected into original numerical .m kernels; it does not validate physical motor geometry.','Identical explicit Gaussian samples are shared; no assumption about MATLAB/NumPy RNG equivalence.'], 'checks':{},'differences':[]}
    with tempfile.TemporaryDirectory(prefix='bluerov-alignment-') as d:
        d=Path(d);input_file=d/'input.mat';output_file=d/'output.mat';savemat(input_file,data)
        source=ROOT/'bluerov2_lazybts/evaluateRK4Linearization.m';text=source.read_text()
        for before,after in report['harness_dimension_only_replacements'].items():
            assert text.count(before)==1;text=text.replace(before,after)
        (d/source.name).write_text(text)
        executor=(ROOT/'bluerov2_lazybts/executeLBTSPlanHardware.m').read_text()
        helper=re.search(r'function error=stateDifference\(state,reference\).*?\nend',executor,re.S).group()
        (d/'source_stateDifference.m').write_text(helper.replace('stateDifference(','source_stateDifference(')+'\n')
        def mq(p):return "'"+str(p).replace("'","''")+"'"
        expression=f'addpath({mq(ROOT/"tools")}); alignment_reference({mq(input_file)},{mq(output_file)},{mq(ROOT)},{mq(d)});'
        cmd=[exe,'--no-gui','--quiet','--eval',expression] if args.engine=='octave' else [exe,'-batch',expression]
        run=subprocess.run(cmd,capture_output=True,text=True,timeout=180)
        if run.returncode:raise RuntimeError(run.stdout+'\n'+run.stderr)
        ref=loadmat(output_file,simplify_cells=True)
        report['engine_version']=str(ref['engine_version'])
        source_files=['bluerov2_kprm/bluerov2Dynamics.m','bluerov2_lazybts/rk4StepLBTS.m','bluerov2_lazybts/evaluateRK4Linearization.m','bluerov2_lazybts/propagateBelief.m','bluerov2_lazybts/chanceCollisionProbability.m','bluerov2_lazybts/executeLBTSPlanHardware.m']
        report['source_sha256']={f:hashlib.sha256((ROOT/f).read_bytes()).hexdigest() for f in source_files}
        def compare(name,a,b,atol=1e-8,rtol=1e-6):
            a=np.asarray(a,float)
            if isinstance(b,np.ndarray) and b.dtype==object:b=np.stack([np.asarray(v,float) for v in b.flat])
            b=np.asarray(b,float).reshape(a.shape)
            error=float(np.max(np.abs(a-b)));ok=bool(np.allclose(a,b,atol=atol,rtol=rtol,equal_nan=True))
            report['checks'][name]={'passed':ok,'max_abs_error':error,'atol':atol,'rtol':rtol}
            if not ok:report['differences'].append(name)
        for j,m in enumerate(models):
            label=f'{m.num_thrusters}_input'
            dx=np.column_stack([bluerov2_dynamics(states[:,k],forces[j][:,k],m) for k in range(24)])
            nxt=np.column_stack([rk4_step(states[:,k],forces[j][:,k],dt[k],m) for k in range(24)])
            compare(label+'.dynamics',dx,ref['dynamics'][j],1e-12)
            compare(label+'.rk4',nxt,ref['rk4'][j],1e-12)
            lin=make_rk4_linearizer(m,options[j].linearization);_,A,B=evaluate_rk4_linearization(lin,states[:,0],forces[j][:,0],dt[0])
            compare(label+'.jacobian_A',A,ref['A'][j]);compare(label+'.jacobian_B',B,ref['B'][j])
            out=propagate_belief(Edge(X,U[j],times,1.,{}),P0,Pt0,m,world,options[j],lin,Z)
            for key in ['P','Ptilde','PSequence','PtildeSequence','gains','kalmanGains','covarianceCost']:
                compare(label+'.'+key,out[key],ref['prop'][j][key])
        errors=fstate-fref;errors[3:6]=wrap_to_pi(errors[3:6]);feedback=np.empty_like(fu)
        for k in range(10):feedback[:,k]=np.clip(fu[:,k]-fK[:,:,k]@errors[:,k],models[0].thrust_min,models[0].thrust_max)
        compare('six_feedback_wrap_and_saturation',feedback,ref['feedback'],1e-12)
        compare('wrapped_state_errors',errors,ref['errors'],1e-12)
        roadmap=Roadmap(np.zeros((5,12)),edges,[[],[0],[0,1],[1,2],[2,3]],costs,[[]]*5,parameters())
        compare('backward_dijkstra',backward_dijkstra(roadmap,4),ref['h'],0.,0.)
        risk,_=chance_collision_probability(chance_x,[chance_P],chance_world,Z,True)
        compare('chance_constraint_same_Z',risk,ref['risk'],0.,0.)
        report['chance_example']={'python_risk':risk,'matlab_source_risk':float(ref['risk']),
            'cause':'Python always uses eigenvector square root; original MATLAB uses Cholesky for positive-definite covariance. Equal distributions, different finite deterministic samples/masks with the same Z.'}
        # Diagnostic calculation only: do not modify chance_collision_probability.
        from bluerov2.lbts import nearest_psd, collision_mask_samples
        ppos=nearest_psd(chance_P[:3,:3],0.)
        vals,vecs=np.linalg.eigh(ppos)
        roots={'python_eigen':vecs@np.diag(np.sqrt(np.maximum(vals,0))),
               'matlab_positive_definite_cholesky':np.linalg.cholesky(ppos)}
        masks={name:collision_mask_samples(chance_x[:3,0,None]+root@Z,chance_world)
               for name,root in roots.items()}
        report['chance_example']['fixed_sample_diagnosis']={
            'samples':int(Z.shape[1]),'position_covariance':ppos.tolist(),
            'square_roots':{name:root.tolist() for name,root in roots.items()},
            'covariance_reconstruction_max_abs_error':{name:float(np.max(np.abs(root@root.T-ppos))) for name,root in roots.items()},
            'collision_counts':{name:int(mask.sum()) for name,mask in masks.items()},
            'different_collision_mask_columns':np.flatnonzero(masks['python_eigen']!=masks['matlab_positive_definite_cholesky']).tolist(),
            'diagnostic_cholesky_risk':float(masks['matlab_positive_definite_cholesky'].mean()),
            'suggestion':'Use the original MATLAB positive-definite Cholesky choice, with eigen fallback only on failure; requires algorithm-change approval and was not applied.',
            'impact':'Changes deterministic samples and finite-sample risk, possibly edge acceptance/search; no probability threshold change is proposed.'}
        report['all_checks_passed']=not report['differences']
        args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps(report,ensure_ascii=False,indent=2))
        # Retain reproducible input and numerical outputs as audit evidence.
        shutil.copyfile(input_file,args.output.with_suffix('.input.mat'));shutil.copyfile(output_file,args.output.with_suffix('.reference.mat'))
    return 0 if report['all_checks_passed'] else 2

if __name__=='__main__':raise SystemExit(main())
