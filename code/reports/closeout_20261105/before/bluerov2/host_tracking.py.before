"""Track an LBTS nominal path using the existing A50/MANUAL_CONTROL PD backend.

This is a distinct execution mode. It uses the saved DVL task origin and the
legacy host PD gains, not the LBTS six-thruster force/feedback controller.
"""
from __future__ import annotations

import argparse
import importlib
import json
from pathlib import Path
import sys
import time

import numpy as np

from .dynamics import rotation_zyx
from .io import load_result
from .kprm import is_position_collision_free, is_trajectory_collision_free
from .model import wrap_to_pi


def load_backend(directory):
    """Load the user's existing backend without copying or changing its files."""
    directory = Path(directory).resolve()
    names = ("rov_control", "dvl_mavlink_bridge", "rov_host_goto")
    for name in names:
        if not (directory / (name + ".py")).is_file():
            raise ValueError(f"missing legacy backend: {directory / (name + '.py')}")
        cached = sys.modules.get(name)
        if cached is not None and Path(cached.__file__).resolve() != directory / (name + ".py"):
            raise ValueError(f"{name} was already loaded from a different directory")
    sys.path.insert(0, str(directory))
    try:
        return importlib.import_module("rov_host_goto")
    finally:
        sys.path.pop(0)


def load_config(path, backend):
    path = Path(path).resolve()
    config = backend.load_config(path)
    matrix = np.asarray(config["dvl_to_task_ned_rotation"], float).reshape(3, 3)
    if not np.allclose(matrix.T @ matrix, np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(matrix), 1.0, atol=1e-6):
        raise ValueError("DVL installation matrix must be a proper rotation")
    origin_path = Path(config["origin_file"])
    config["origin_file"] = str(origin_path if origin_path.is_absolute() else path.parent / origin_path)
    config["trajectory"] = {
        "max_start_position_error_m": .25,
        "max_start_yaw_error_rad": np.deg2rad(15),
        "max_tracking_error_m": .75,
        "max_tracking_yaw_error_rad": np.deg2rad(35),
        "max_tilt_rad": np.deg2rad(20),
        "max_speed_m_s": 1.2,
        "max_timing_overrun_s": .25,
        "settle_timeout_s": 5.0,
        "arrival_yaw_tolerance_rad": np.deg2rad(10),
        **config.get("trajectory", {}),
    }
    for name, value in config["trajectory"].items():
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not np.isfinite(value) or value <= 0:
            raise ValueError(f"trajectory.{name} must be finite and positive")
    return config


class _RecvBudgetSocket:
    """Bound the legacy feed's drain loop, including a continuously busy socket."""
    def __init__(self, sock):
        self.sock = sock
        self.reset()
    def reset(self):
        self.calls, self.bytes = 0, 0
        self.started = time.monotonic()
    def recv(self, size):
        if self.calls >= 8 or self.bytes >= 65536 or time.monotonic()-self.started >= .005:
            raise BlockingIOError()
        chunk = self.sock.recv(min(size, 65536-self.bytes))
        self.calls += 1
        self.bytes += len(chunk)
        return chunk
    def __getattr__(self, name):
        return getattr(self.sock, name)


def make_feed(backend, config, *, bounded_parse=False):
    class BoundedDvlFeed(backend.DvlFeed):
        def poll(self):
            if self.sock is None:
                self.connect()
            if not isinstance(self.sock, _RecvBudgetSocket):
                self.sock = _RecvBudgetSocket(self.sock)
            self.sock.reset()
            if bounded_parse:
                if len(self.buffer) > 1024*1024:
                    raise backend.ControlError("DVL JSON line buffer exceeded 1 MiB")
                super().poll(max_reports=16, max_line_bytes=16384, max_buffer_bytes=1024*1024)
            else:
                super().poll()
                if len(self.buffer) > 1024*1024:
                    raise backend.ControlError("DVL JSON line buffer exceeded 1 MiB")
    return BoundedDvlFeed(config["dvl_ip"], config["dvl_port"],
        config["dvl_to_task_ned_rotation"], config["yaw_offset_rad"], config["yaw_sign"])


def validate_plan(result, config):
    if not result.get("success", False):
        raise ValueError("a successful LBTS result is required")
    x = np.asarray(result["nominalTrajectory"], float)
    t = np.asarray(result["time"], float)
    if x.ndim != 2 or x.shape[0] != 12 or x.shape[1] < 2 or t.shape != (x.shape[1],):
        raise ValueError("plan must contain state (12,N+1) and time (N+1,)")
    if not np.all(np.isfinite(x)) or not np.all(np.isfinite(t)) or abs(t[0]) > 1e-12 or np.any(np.diff(t) <= 0):
        raise ValueError("plan must be finite with increasing time starting at zero")
    if result["model"].num_thrusters != 6:
        raise ValueError("a standard six-thruster plan is required")
    limits = config["trajectory"]
    if t[-1] + limits["settle_timeout_s"] > config["task_timeout_s"]:
        raise ValueError("plan plus settling time exceeds task_timeout_s")
    if np.max(np.abs(x[3:5])) > limits["max_tilt_rad"]:
        raise ValueError("host tracking supports approximately level trajectories")
    world = result.get("world")
    if world is not None and not is_trajectory_collision_free(x, world):
        raise ValueError("nominal trajectory is outside free space")
    return x, t


def reference_at(x, t, elapsed):
    elapsed = float(np.clip(elapsed, t[0], t[-1]))
    k = min(int(np.searchsorted(t, elapsed, side="right")) - 1, len(t) - 2)
    alpha = (elapsed - t[k]) / (t[k+1] - t[k])
    delta = x[:, k+1] - x[:, k]
    delta[3:6] = wrap_to_pi(delta[3:6])
    reference = x[:, k] + alpha * delta
    reference[3:6] = wrap_to_pi(reference[3:6])
    return reference


def read_sample(feed, origin, config, velocity_frame):
    """Read A50 task position and velocity; keep acquisition age explicit.

    The legacy feed applies the installation matrix to both reports. Native
    sensor/body velocity then needs the current attitude to reach task axes.
    The task option supports a backend that already provides world velocity.
    """
    timestamp, position, velocity, yaw = feed.current(config["dvl_timeout_s"], config["max_position_std_m"])
    raw = feed.latest[1]
    now = time.monotonic()
    age = max(now-timestamp, now-feed.latest_velocity[0])
    if not np.isfinite(age) or not 0 <= age <= config["dvl_timeout_s"]:
        raise ValueError("stale DVL state")
    if not np.isfinite(raw.position_std) or raw.position_std < 0:
        raise ValueError("invalid DVL position standard deviation")
    matrix = np.asarray(config["dvl_to_task_ned_rotation"]).reshape(3, 3)
    roll, pitch = np.deg2rad(raw.attitude[:2])
    relative_yaw = wrap_to_pi(yaw-origin["yaw_target_rad"])
    rotation = matrix @ rotation_zyx(roll, pitch, relative_yaw) @ matrix.T
    body_yaw = float(np.arctan2(rotation[1, 0], rotation[0, 0]))
    tilt = float(np.arccos(np.clip(rotation[2, 2], -1, 1)))
    velocity = np.asarray(velocity, float)
    if velocity_frame == "body":
        velocity = rotation @ velocity
    elif velocity_frame != "task":
        raise ValueError("velocity_frame must be body or task")
    position = np.asarray(position, float)-np.asarray(origin["origin"])
    if position.shape != (3,) or velocity.shape != (3,) or not np.all(np.isfinite(np.r_[position, velocity, body_yaw, tilt])):
        raise ValueError("invalid DVL position, velocity or attitude")
    return {"position": position, "velocity": velocity, "yaw": body_yaw,
            "age": float(age), "tilt": tilt, "timestamp": timestamp}


def manual_axes(reference, sample, origin, config, backend, *, hold_final=False):
    error = reference[:3]-sample["position"]
    reference_velocity = np.zeros(3) if hold_final else rotation_zyx(*reference[3:6]) @ reference[6:9]
    velocity_error = sample["velocity"]-reference_velocity
    # The legacy routine's yaw_target defines the fixed task basis. Keep it
    # fixed while tracking a changing desired heading, then set yaw separately.
    yaw_zero = origin["yaw_target_rad"]
    axes = backend.compute_manual_axes(tuple(error), tuple(velocity_error),
                                      yaw_zero+sample["yaw"], yaw_zero, config)
    yaw_error = float(wrap_to_pi(reference[5]-sample["yaw"]))
    yaw_axis = backend.clamp(config["yaw_kp_level_per_rad"]*yaw_error, config["max_horizontal_level"])
    return (*axes[:3], yaw_axis)


def run_host_tracking(result, controller, feed, config, backend, *, execute=False,
                      confirmation="", velocity_frame="body"):
    if not isinstance(execute, bool) or (execute and confirmation != "ENABLE_BLUEROV2_HOST_TRACKING"):
        raise ValueError("motion requires execute=True and ENABLE_BLUEROV2_HOST_TRACKING")
    if velocity_frame not in ("body", "task"):
        raise ValueError("velocity_frame must be body or task")
    x, t = validate_plan(result, config)
    limits, world = config["trajectory"], result.get("world")
    report = {"status": "aborted", "reason": "", "execution_mode": "host_pd",
              "actuation_enabled": execute, "plan_frame": config["task_frame"],
              "velocity_frame": velocity_frame, "samples": [], "disarm_confirmed": None}
    def service():
        controller.service_io()
        if controller.stop_requested:
            raise backend.ControlError(controller.termination_reason or "operator_abort")
        if not controller.vehicle.heartbeat_fresh(time.monotonic(), controller.heartbeat_timeout):
            raise backend.ControlError("stale MAVLink heartbeat")
    def check_sample(sample, reference, *, preflight=False):
        position_error = float(np.linalg.norm(sample["position"]-reference[:3]))
        if position_error > limits["max_start_position_error_m" if preflight else "max_tracking_error_m"]:
            raise ValueError("start_position_error" if preflight else "position_tracking_error")
        if sample["tilt"] > limits["max_tilt_rad"]:
            raise ValueError("tilt_limit")
        speed_limit = config["origin_max_speed_m_s"] if preflight else limits["max_speed_m_s"]
        if np.linalg.norm(sample["velocity"]) > speed_limit:
            raise ValueError("start_speed_limit" if preflight else "speed_limit")
        if preflight and abs(wrap_to_pi(sample["yaw"]-reference[5])) > limits["max_start_yaw_error_rad"]:
            raise ValueError("start_yaw_error")
        if not preflight and abs(wrap_to_pi(sample["yaw"]-reference[5])) > limits["max_tracking_yaw_error_rad"]:
            raise ValueError("yaw_tracking_error")
        if world is not None and not is_position_collision_free(sample["position"], world):
            raise ValueError("estimated_collision_or_out_of_bounds")
    try:
        controller.connect()
        if controller.vehicle.vehicle_type != 12 or controller.vehicle.autopilot != 3:
            raise ValueError("selected vehicle is not an ArduSub submarine")
        if controller.vehicle.armed is not False:
            raise ValueError("vehicle is already armed or arming state is unknown")
        origin = backend.load_origin_state(Path(config["origin_file"]), config)
        if origin is None:
            raise ValueError("saved task origin is missing; establish it with the existing host tool first")
        report["origin"] = origin
        feed.connect()
        deadline = time.monotonic()+config["origin_sample_duration_s"]
        while True:
            service()
            try:
                sample = read_sample(feed, origin, config, velocity_frame)
                break
            except backend.ControlError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(.01)
        check_sample(sample, x[:, 0], preflight=True)
        report["preflight"] = sample
        controller.record_event("host_trajectory_preflight", execution_mode="host_pd",
                                plan_frame=config["task_frame"], duration_s=float(t[-1]))
        if not execute:
            report["status"] = "checked"
        else:
            controller.request_mode("MANUAL")
            controller.set_state(backend.State.NEUTRAL)
            controller.send_manual((0, 0, 500, 0))
            controller.request_arm()
            controller.set_state(backend.State.MOVING)
            start, next_tick, held_since = time.monotonic(), 0.0, None
            previous = sample
            while True:
                service()
                controller.check_runtime_safety()
                elapsed = time.monotonic()-start
                if elapsed < next_tick:
                    time.sleep(min(.01, next_tick-elapsed))
                    continue
                sample = read_sample(feed, origin, config, velocity_frame)
                elapsed = time.monotonic()-start
                if elapsed-next_tick > limits["max_timing_overrun_s"]:
                    raise ValueError("timing_overrun")
                if np.linalg.norm(sample["position"]-previous["position"]) > config["position_jump_threshold_m"]:
                    raise ValueError("DVL position jump")
                previous = sample
                reference = reference_at(x, t, elapsed)
                check_sample(sample, reference)
                controller.check_runtime_safety()
                if controller.stop_requested:
                    raise backend.ControlError("operator_abort")
                at_end = elapsed >= t[-1]
                axes = manual_axes(reference, sample, origin, config, backend, hold_final=at_end)
                entry = {"elapsed_s": elapsed, "reference": reference, "axes": axes, **sample}
                report["samples"].append(entry)
                controller.record_event("host_trajectory_sample", **json_safe(entry))
                reached = (np.linalg.norm(sample["position"][:2]-x[:2, -1]) <= config["arrival_horizontal_tolerance_m"]
                    and abs(sample["position"][2]-x[2, -1]) <= config["arrival_vertical_tolerance_m"]
                    and abs(wrap_to_pi(sample["yaw"]-x[5, -1])) <= limits["arrival_yaw_tolerance_rad"]
                    and np.linalg.norm(sample["velocity"]) <= config["arrival_speed_threshold_m_s"])
                held_since = (elapsed if held_since is None else held_since) if at_end and reached else None
                if held_since is not None and elapsed-held_since >= config["arrival_hold_s"]:
                    report["status"] = "completed"
                    break
                if elapsed >= t[-1]+limits["settle_timeout_s"]:
                    raise ValueError("goal_not_reached")
                controller.send_manual(axes)
                next_tick = elapsed+.05
    except (RuntimeError, OSError, ValueError) as exc:
        report["reason"] = str(exc)
    finally:
        if controller.control_responsibility:
            try:
                confirmed = controller.disarm_confirmed if controller.disarm_request_attempted else controller.stop_and_disarm()
                report["disarm_confirmed"] = bool(confirmed)
                if not confirmed:
                    report["status"] = "aborted"
                    report["reason"] = report["reason"] or "disarm_unconfirmed"
            except Exception as exc:
                report["status"] = "aborted"
                report["cleanup_error"] = str(exc)
                report["disarm_confirmed"] = False
        for close in (feed.close, controller.close):
            try:
                close()
            except Exception as exc:
                report["status"] = "aborted"
                report["cleanup_error"] = str(exc)
    return report


def json_safe(value):
    if isinstance(value, np.ndarray):
        return json_safe(value.tolist())
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_safe(v) for v in value]
    if isinstance(value, np.generic):
        return value.item()
    return value


def main(argv=None):
    parser = argparse.ArgumentParser(description="Track an LBTS nominal path with the existing A50 host PD controller")
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=Path(__file__).resolve().parents[2]/"host_goto_config.json")
    parser.add_argument("--legacy-dir", type=Path)
    parser.add_argument("--connection")
    parser.add_argument("--velocity-frame", choices=("body", "task"), default="body")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check", action="store_true", help="check connectivity, fixed origin and starting state (default)")
    mode.add_argument("--execute", action="store_true", help="arm and track using host PD")
    parser.add_argument("--confirmation", default="")
    parser.add_argument("--log-dir", type=Path, default=Path("logs"))
    parser.add_argument("--output", type=Path, default=Path("host_tracking_report.json"))
    args = parser.parse_args(argv)
    logger = None
    try:
        if args.execute and args.confirmation != "ENABLE_BLUEROV2_HOST_TRACKING":
            raise ValueError("--execute requires --confirmation ENABLE_BLUEROV2_HOST_TRACKING")
        backend = load_backend(args.legacy_dir or args.config.resolve().parent)
        config = load_config(args.config, backend)
        result = load_result(args.plan)
        validate_plan(result, config)
        logger = backend.SessionLogger(args.log_dir, action="host-trajectory")
        controller = backend.Controller(args.connection or config["connection"],
            int(config.get("baud", 115200)), int(config.get("source_system", 255)),
            int(config.get("source_component", 190)), config.get("target_system"),
            config.get("target_component"), logger, config.get("connect_timeout", 15.0),
            config.get("confirm_timeout", 5.0), config.get("heartbeat_timeout", 3.0))
        feed = make_feed(backend, config)
        backend.install_signal_handlers(controller)
        report = run_host_tracking(result, controller, feed, config, backend, execute=args.execute,
            confirmation=args.confirmation, velocity_frame=args.velocity_frame)
        args.output.write_text(json.dumps(json_safe(report), ensure_ascii=False, indent=2, allow_nan=False)+"\n", encoding="utf-8")
        print(f"{report['status']}: {report['reason'] or 'host_pd'}; report={args.output}")
        return 0 if report["status"] in ("checked", "completed") else 1
    except (OSError, ValueError, KeyError, RuntimeError) as exc:
        print(f"host tracking failed: {exc}", file=sys.stderr)
        return 2
    finally:
        if logger is not None:
            logger.close()


if __name__ == "__main__":
    raise SystemExit(main())
