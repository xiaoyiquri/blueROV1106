"""Small deterministic, obstacle-free end-to-end demonstration."""
from __future__ import annotations
import argparse
from pathlib import Path
import numpy as np
from .model import parameters, model_metadata
from .kprm import mean_control, Edge, Roadmap, save_trajectory_graph, query_kinodynamic_prm
from .lbts import lbts, default_lbts_options, monte_carlo_path
from .execution import execute_lbts_plan
from .io import save_result


def mock_io(reference):
    state=np.asarray(reference[:,0]).copy();commands=[]
    def read_state():return {"state":state.copy(),"valid":True,"age":0.0}
    def send(f):commands.append(np.asarray(f).copy())
    def abort():return False
    return {"read_state":read_state,"send_thruster_forces":send,"abort_requested":abort},commands


def run(output="demo_result.pkl", N=20, run_mc=False, plot=False):
    p=parameters();p.world.obstacles=[];p.prm.num_positions=2;p.prm.min_position_separation=.01;p.prm.connection_radius=2.;p.query.connection_radius=2.
    p.ocp.N=N;p.ocp.Tmin=.4;p.ocp.Tmax=4.;p.ocp.ipopt_max_iter=300
    x0=np.zeros(12);x0[:3]=[0.,0.,1.5];xg=x0.copy();xg[:3]=[.35,.20,1.5]
    print("six-thruster metadata:",model_metadata(p.model));print("allocation=\n",p.model.allocation);print("rank",np.linalg.matrix_rank(p.model.allocation),"singular values",np.linalg.svd(p.model.allocation,compute_uv=False))
    xbar,cost,n,u,info=mean_control(x0,xg,p)
    if not info["success"]:raise RuntimeError(f"local connector failed: {info}")
    edge=Edge(xbar,u,np.asarray(info["time"]),cost,info)
    # A two-vertex graph is enough to exercise attachment, directed search and
    # belief propagation while leaving the formal PRM defaults untouched.
    g=Roadmap(np.vstack([x0,xg]),[[1],[]],[[],[0]],[[cost],[]],[[edge],[]],p)
    query=query_kinodynamic_prm(g,x0,xg,p.world)
    print("query path",query["path"],"success",query["success"],"blocked edges",query["blocked_edge_count"])
    if not query["success"]: raise RuntimeError("small query graph has no feasible directed path")
    g=query["roadmap"]
    opts=default_lbts_options(p.model);opts.attach_query=False;opts.chance.num_samples=32;opts.search.max_expansions=20;opts.search.max_popped_edges=50;opts.search.verbose=False;opts.output.include_trajectory_graph=False
    result=lbts(g,g.query_info["start_idx"],g.query_info["goal_idx"],opts)
    print("LBTS success",result["success"],result["statistics"])
    if result["success"]:
        save_result(output,result)
        io,commands=mock_io(result["nominalTrajectory"]);exp=execute_lbts_plan(result,io,{"real_time":False,"enable_actuation":True,"confirmation":"ENABLE_BLUEROV2_THRUST","verbose":False});print("mock execution",exp["status"],"commands",len(commands),"final position error",exp["final_position_error"])
        if run_mc:
            mc=monte_carlo_path(result,{"num_runs":5,"plot":False,"verbose":False});print("MC collision rate",mc["collision_rate"])
        if plot:
            import matplotlib.pyplot as plt
            from .plotting import plot_trajectory, plot_control, plot_execution
            plot_trajectory(result,show=False);plt.savefig(str(Path(output).with_suffix(".trajectory.png")));plt.close()
            plot_control(result,show=False);plt.savefig(str(Path(output).with_suffix(".control.png")));plt.close()
            plot_execution(exp,show=False);plt.savefig(str(Path(output).with_suffix(".error.png")));plt.close()
    return result


def main(argv=None):
    ap=argparse.ArgumentParser();ap.add_argument("--output",default="demo_result.pkl");ap.add_argument("--N",type=int,default=20);ap.add_argument("--mc",action="store_true");ap.add_argument("--plot",action="store_true");args=ap.parse_args(argv);run(args.output,args.N,args.mc,args.plot)


if __name__=="__main__":main()
