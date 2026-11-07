"""Hardware-independent feedback executor with the original callback contract."""
from __future__ import annotations
from types import SimpleNamespace
import time
import numpy as np
from .model import wrap_to_pi
from .kprm import is_position_collision_free


def _get(io, name):
    if isinstance(io, dict): return io.get(name)
    return getattr(io,name,None)


def execute_lbts_plan(result: dict, io, user_options=None) -> dict:
    defaults=dict(enable_actuation=False,confirmation="",real_time=True,wait_poll_period=.01,max_timing_overrun=.25,
                  require_state_age=True,max_state_age=.25,max_start_position_error=.25,max_start_attitude_error=np.deg2rad(15),
                  max_start_velocity_error=.25,max_start_angular_rate_error=np.deg2rad(20),max_position_tracking_error=.75,
                  max_attitude_tracking_error=np.deg2rad(35),max_linear_speed=1.20,max_angular_speed=np.deg2rad(100),
                  abort_outside_free_space=True,saturation_tolerance=1e-10,verbose=True)
    if user_options:
        defaults.update(vars(user_options) if not isinstance(user_options,dict) else user_options)
    o=SimpleNamespace(**defaults)
    if not isinstance(o.enable_actuation,(bool,np.bool_)):raise ValueError("enable_actuation must be a boolean")
    for name in ("max_start_position_error","max_start_attitude_error","max_start_velocity_error","max_start_angular_rate_error","max_position_tracking_error","max_attitude_tracking_error","max_linear_speed","max_angular_speed","saturation_tolerance"):
        if not np.isfinite(getattr(o,name)) or getattr(o,name)<0:raise ValueError(f"{name} must be finite and nonnegative")
    if not result.get("success",False): raise ValueError("a successful LBTS result is required")
    X=np.asarray(result["nominalTrajectory"],float);U=np.asarray(result["controlTrajectory"],float);t=np.asarray(result["time"],float).reshape(-1);K=result["feedbackGains"];model=result["model"]
    if U.ndim != 2:raise ValueError("controlTrajectory must be a two-dimensional array")
    steps=U.shape[1]
    if steps<1 or X.shape!=(12,steps+1) or U.shape[0]!=6 or len(t)!=steps+1 or not np.all(np.isfinite(t)) or abs(t[0])>1e-12 or np.any(np.diff(t)<=0):raise ValueError("invalid six-thruster plan dimensions or time; time must start at zero")
    if len(K)!=steps or any(np.asarray(k).shape!=(6,12) for k in K):raise ValueError("feedbackGains must contain 6x12 matrices")
    if not np.all(np.isfinite(X)) or not np.all(np.isfinite(U)) or any(not np.all(np.isfinite(k)) for k in K):raise ValueError("plan states, controls and gains must be finite")
    if np.shape(model.thrust_min)!=(6,) or np.shape(model.thrust_max)!=(6,):raise ValueError("executor requires six-dimensional thrust limits")
    if not np.all(np.isfinite(model.thrust_min)) or not np.all(np.isfinite(model.thrust_max)) or np.any(np.asarray(model.thrust_min)>np.asarray(model.thrust_max)):raise ValueError("invalid thrust limits")
    if not np.isfinite(o.wait_poll_period) or o.wait_poll_period<=0 or not np.isfinite(o.max_timing_overrun) or o.max_timing_overrun<0 or not np.isfinite(o.max_state_age) or o.max_state_age<=0:raise ValueError("invalid timing or state age options")
    read=_get(io,"read_state")
    if not callable(read):raise ValueError("io.read_state callback is required")
    send=_get(io,"send_thruster_forces")
    if o.enable_actuation and not callable(send):raise ValueError("send_thruster_forces is required when actuation is enabled")
    if o.enable_actuation and o.confirmation!="ENABLE_BLUEROV2_THRUST":raise ValueError("explicit actuation confirmation is required")
    initialize=_get(io,"initialize"); stop=_get(io,"stop"); abort=_get(io,"abort_requested")
    stopped=False
    stop_confirmation=None
    def zero():
        nonlocal stopped,stop_confirmation
        if stopped:return
        stopped=True
        if o.enable_actuation and callable(send):
            try:send(np.zeros(6))
            except Exception:pass
        if callable(stop):
            try:stop_confirmation=stop()
            except Exception as exc:stop_confirmation={"confirmed":False,"reason":str(exc)}
    def wait_until(clock, target):
        while time.monotonic()-clock<target:
            if callable(abort) and bool(abort()):return True
            service=_get(io,"poll")
            if callable(service):service()
            time.sleep(min(o.wait_poll_period,max(target-(time.monotonic()-clock),0)))
        return callable(abort) and bool(abort())
    try:
        if callable(initialize):initialize()
        pre=_sample(read())
        _validate_sample(pre,o,True)
        e=_state_difference(pre["state"],X[:,0])
        reason=_start_reason(e,pre["state"],result.get("world"),o)
        if reason: raise ValueError(f"preflight failed: {reason}")
        if callable(_get(io,"on_start")):_get(io,"on_start")(result,o)
        est=np.full((12,steps+1),np.nan);err=np.full_like(est,np.nan);feed=np.asarray(U).copy();fb=np.full((6,steps),np.nan);un=np.full((6,steps),np.nan);cmd=np.full((6,steps),np.nan);actual=np.full(steps+1,np.nan);age=np.full(steps+1,np.nan);sat=np.zeros((6,steps),bool);status="completed";abort_reason="";done=0;clock=time.monotonic()
        for k in range(steps):
            if o.real_time:
                if wait_until(clock,t[k]):status="aborted";abort_reason="operator_abort";break
                if time.monotonic()-clock-t[k]>o.max_timing_overrun:status="aborted";abort_reason="timing_overrun";break
            if callable(abort) and bool(abort()):status="aborted";abort_reason="operator_abort";break
            s=_sample(read());actual[k]=time.monotonic()-clock if o.real_time else t[k];est[:,k]=s["state"];age[k]=s["age"]
            if o.real_time and actual[k]-t[k]>o.max_timing_overrun:status="aborted";abort_reason="timing_overrun";break
            reason=_runtime_reason(s,X[:,k],result.get("world"),o)
            if reason:status="aborted";abort_reason=reason;break
            er=_state_difference(s["state"],X[:,k]);err[:,k]=er
            with np.errstate(over="ignore",invalid="ignore"):
                fb[:,k]=-np.asarray(K[k])@er;un[:,k]=U[:,k]+fb[:,k]
            if not np.all(np.isfinite(un[:,k])):status="aborted";abort_reason="invalid_control_command";break
            cmd[:,k]=np.clip(un[:,k],model.thrust_min,model.thrust_max);sat[:,k]=np.abs(cmd[:,k]-un[:,k])>o.saturation_tolerance
            # A late wakeup must never compress or replay an already elapsed interval.
            if o.real_time and time.monotonic()-clock>=t[k+1]:
                status="aborted";abort_reason="missed_control_interval";break
            deadline=_get(io,"set_interval_deadline")
            if callable(deadline):deadline(clock+t[k+1] if o.real_time else None)
            output_result=send(cmd[:,k]) if o.enable_actuation else None
            step_callback=_get(io,"on_step")
            if callable(step_callback):
                step_callback({"step":k,"scheduled_time":float(t[k]),"actual_time":float(actual[k]),
                    "nominal_state":X[:,k],"sample":s,"tracking_error":er,"nominal_force_n":U[:,k],
                    "feedback_force_n":fb[:,k],"unconstrained_force_n":un[:,k],
                    "algorithm_force_n":cmd[:,k],"algorithm_saturated":sat[:,k],"output":output_result})
            done=k+1
        if status=="completed":
            if o.real_time and wait_until(clock,t[-1]):status="aborted";abort_reason="operator_abort"
        if status=="completed":
            s=_sample(read());actual[-1]=time.monotonic()-clock if o.real_time else t[-1];est[:,-1]=s["state"];age[-1]=s["age"];reason=_runtime_reason(s,X[:,-1],result.get("world"),o)
            if o.real_time and actual[-1]-t[-1]>o.max_timing_overrun:reason="timing_overrun"
            if reason:status="aborted";abort_reason=reason
            err[:,-1]=_state_difference(s["state"],X[:,-1])
        zero()
        if _get(io,"require_stop_confirmation") and not (isinstance(stop_confirmation,dict) and stop_confirmation.get("confirmed") is True):
            status="stop_unconfirmed"
            abort_reason=(abort_reason+";" if abort_reason else "")+"stop_unconfirmed"
        exp={"status":status,"abort_reason":abort_reason,"actuation_enabled":bool(o.enable_actuation),"executed_steps":done,
             "reference_state":X,"reference_control":U,"scheduled_time":t,"actual_time":actual,"estimated_state":est,"tracking_error":err,
             "feedforward_control":feed,"feedback_control":fb,"unconstrained_command":un,"commanded_thruster_force":cmd,"saturated":sat,"state_age":age,
             "final_position_error":float(np.linalg.norm(err[:3,-1])) if np.all(np.isfinite(err[:3,-1])) else np.nan,
             "final_attitude_error":float(np.linalg.norm(err[3:6,-1])) if np.all(np.isfinite(err[3:6,-1])) else np.nan,"options":o,"stop_confirmation":stop_confirmation,"sequence_completion_is_tracking_success":False}
        finish=_get(io,"on_finish")
        if callable(finish):finish(exp)
        return exp
    finally:
        # Attempt neutralization/cleanup once, including on initialization failure.
        zero()


def _sample(raw):
    if isinstance(raw,dict):
        s=dict(raw);s["state"]=np.asarray(s["state"],float).reshape(12);s.setdefault("valid",True);s.setdefault("age",np.nan);s.setdefault("timestamp",None);return s
    return {"state":np.asarray(raw,float).reshape(12),"valid":True,"age":np.nan,"timestamp":None}


def _validate_sample(s,o,preflight=False):
    if not s.get("valid",True) or np.any(~np.isfinite(s["state"])):raise ValueError("invalid state estimate")
    if np.isfinite(s["age"]) and s["age"]<0:raise ValueError("negative state age")
    if preflight and o.enable_actuation and o.require_state_age and (not np.isfinite(s["age"]) or s["age"]>o.max_state_age):raise ValueError("a fresh state age is required for actuation")


def _state_difference(state,ref):
    d=np.asarray(state)-np.asarray(ref);d[3:6]=wrap_to_pi(d[3:6]);return d


def _start_reason(e,state,world,o):
    if np.linalg.norm(e[:3])>o.max_start_position_error:return "start_position_error"
    if np.linalg.norm(e[3:6])>o.max_start_attitude_error:return "start_attitude_error"
    if np.linalg.norm(e[6:9])>o.max_start_velocity_error:return "start_velocity_error"
    if np.linalg.norm(e[9:12])>o.max_start_angular_rate_error:return "start_angular_rate_error"
    if o.abort_outside_free_space and world is not None and not is_position_collision_free(state[:3],world):return "start_not_in_free_space"
    return ""


def _runtime_reason(s,ref,world,o):
    try:_validate_sample(s,o)
    except Exception:return "invalid_state_estimate"
    if o.enable_actuation and o.require_state_age and (not np.isfinite(s["age"]) or s["age"]>o.max_state_age):return "stale_state_estimate"
    e=_state_difference(s["state"],ref)
    if np.linalg.norm(e[:3])>o.max_position_tracking_error:return "position_tracking_error"
    if np.linalg.norm(e[3:6])>o.max_attitude_tracking_error:return "attitude_tracking_error"
    if np.linalg.norm(s["state"][6:9])>o.max_linear_speed:return "linear_speed_limit"
    if np.linalg.norm(s["state"][9:12])>o.max_angular_speed:return "angular_speed_limit"
    if o.abort_outside_free_space and world is not None and not is_position_collision_free(s["state"][:3],world):return "estimated_collision_or_out_of_bounds"
    return ""
