import numpy as np
from bluerov2 import parameters
from bluerov2.kprm import mean_control, Edge, Roadmap, save_trajectory_graph, load_trajectory_graph
from bluerov2.lbts import default_lbts_options, lbts, propagate_belief
from bluerov2.execution import execute_lbts_plan


def _small_result():
    p=parameters();p.world.obstacles=[];p.ocp.N=3;p.ocp.Tmax=3.;p.ocp.ipopt_max_iter=300
    x0=np.zeros(12);x0[:3]=[0,0,1.5];x1=x0.copy();x1[:3]=[.2,.1,1.5]
    xb,c,N,u,info=mean_control(x0,x1,p);assert info["success"]
    e=Edge(xb,u,np.asarray(info["time"]),c,info);g=Roadmap(np.vstack([x0,x1]),[[1],[]],[[],[0]],[[c],[]],[[e],[]],p)
    o=default_lbts_options(p.model);o.attach_query=False;o.chance.num_samples=16;o.search.verbose=False;o.search.max_expansions=5
    return lbts(g,0,1,o)


def test_lbts_propagation_and_mock_executor(tmp_path):
    result=_small_result();assert result["success"]
    assert result["controlTrajectory"].shape[0]==6
    sent=[]
    io={"read_state":lambda:{"state":result["nominalTrajectory"][:,0],"valid":True,"age":0.0},"send_thruster_forces":lambda f:sent.append(np.asarray(f))}
    ex=execute_lbts_plan(result,io,{"enable_actuation":True,"confirmation":"ENABLE_BLUEROV2_THRUST","real_time":False,"verbose":False})
    assert ex["status"]=="completed" and all(x.shape==(6,) for x in sent)


def test_graph_roundtrip(tmp_path):
    result=_small_result();g=result["roadmap"] if result.get("roadmap") is not None else None
    # The search result intentionally omits the graph by default; construct a
    # tiny graph from the nominal path for the storage contract.
    p=parameters();x=result["nominalTrajectory"];u=result["controlTrajectory"];t=result["time"]
    e=Edge(x,u,t,result["cost"],{"time":t});g=Roadmap(np.vstack([x[:,0],x[:,-1]]),[[1],[]],[[],[0]],[[result["cost"]],[]],[[e],[]],p)
    path=tmp_path/"g.pkl";save_trajectory_graph(path,g);loaded=load_trajectory_graph(path);assert loaded.param.model.num_thrusters==6
