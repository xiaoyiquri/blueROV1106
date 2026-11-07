"""Matplotlib plots corresponding to the MATLAB PRM/LBTS figures."""
from __future__ import annotations
import numpy as np


def plot_trajectory(result, show=True, ax=None):
    import matplotlib.pyplot as plt
    if ax is None:
        fig=plt.figure();ax=fig.add_subplot(111,projection="3d")
    X=np.asarray(result.get("nominalTrajectory",result.get("nominal_trajectory")));ax.plot(X[0],X[1],X[2],"b-",label="nominal trajectory")
    ax.scatter([X[0,0]],[X[1,0]],[X[2,0]],c="g",label="start");ax.scatter([X[0,-1]],[X[1,-1]],[X[2,-1]],c="m",label="goal")
    ax.set_xlabel("North [m]");ax.set_ylabel("East [m]");ax.set_zlabel("Depth [m]");ax.invert_zaxis();ax.legend()
    if show:plt.show()
    return ax


def plot_control(result, show=True, ax=None):
    import matplotlib.pyplot as plt
    if ax is None:fig,ax=plt.subplots()
    t=np.asarray(result["time"]);U=np.asarray(result.get("controlTrajectory",result.get("control_trajectory")));ax.step(t[:-1],U.T,where="post");ax.set_xlabel("time [s]");ax.set_ylabel("force [N]");ax.grid(True)
    if show:plt.show()
    return ax


def plot_execution(experiment, show=True, ax=None):
    import matplotlib.pyplot as plt
    if ax is None:fig,ax=plt.subplots()
    t=experiment["scheduled_time"];e=experiment["tracking_error"];ax.plot(t,np.linalg.norm(e[:3],axis=0),label="position error [m]");ax.plot(t,np.rad2deg(np.linalg.norm(e[3:6],axis=0)),label="attitude error [deg]");ax.grid(True);ax.legend();ax.set_xlabel("time [s]")
    if show:plt.show()
    return ax
