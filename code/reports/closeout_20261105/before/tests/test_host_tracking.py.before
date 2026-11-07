from pathlib import Path
from types import SimpleNamespace
import json

import numpy as np
import pytest

from bluerov2 import parameters
from bluerov2 import host_tracking as ht


@pytest.fixture
def backend():
    return ht.load_backend(Path(__file__).resolve().parents[2])


@pytest.fixture
def config(backend, tmp_path):
    config = ht.load_config(Path(__file__).resolve().parents[2]/"host_goto_config.json", backend)
    config.update(origin_file=str(tmp_path/"origin.json"), arrival_hold_s=.1,
                  arrival_horizontal_tolerance_m=.05, arrival_vertical_tolerance_m=.05)
    config["trajectory"]["settle_timeout_s"] = .2
    backend.save_origin_state(Path(config["origin_file"]), config, (0, 0, 0), .7)
    return config


def plan():
    return {"success": True, "nominalTrajectory": np.zeros((12, 2)),
            "time": np.array([0., .1]), "model": parameters().model, "world": None}


@pytest.fixture
def rig(monkeypatch, backend):
    clock = SimpleNamespace(now=0.)
    monkeypatch.setattr(ht.time, "monotonic", lambda: clock.now)
    monkeypatch.setattr(ht.time, "sleep", lambda dt: setattr(clock, "now", clock.now+dt))
    class Controller:
        def __init__(self):
            self.vehicle = SimpleNamespace(armed=False, vehicle_type=12, autopilot=3,
                heartbeat_fresh=lambda *args: True)
            self.heartbeat_timeout = 3.
            self.stop_requested = False
            self.termination_reason = ""
            self.control_responsibility = False
            self.disarm_request_attempted = False
            self.disarm_confirmed = False
            self.events, self.sent = [], []
        def connect(self): self.events.append("connect")
        def service_io(self): pass
        def record_event(self, *args, **kwargs): pass
        def request_mode(self, mode): self.events.append(mode)
        def set_state(self, state): pass
        def send_manual(self, axes): self.sent.append(axes)
        def request_arm(self):
            self.events.append("arm")
            self.control_responsibility = True
            self.vehicle.armed = True
        def check_runtime_safety(self):
            if not self.vehicle.armed: raise backend.ControlError("unexpected disarm")
        def stop_and_disarm(self):
            self.events.append("disarm")
            self.disarm_request_attempted = True
            self.disarm_confirmed = True
            self.send_manual((0, 0, 500, 0))
            return True
        def close(self): self.events.append("close")
    class Feed:
        def __init__(self):
            self.position, self.velocity = np.zeros(3), np.zeros(3)
            self.yaw, self.roll, self.pitch = .7, 0., 0.
            self.reads, self.fail_after, self.delay_after = 0, None, None
            self.connected = self.closed = False
        def connect(self): self.connected = True
        def close(self): self.closed = True
        def current(self, *args):
            self.reads += 1
            if self.fail_after and self.reads >= self.fail_after:
                raise backend.ControlError("stale DVL")
            if self.delay_after and self.reads >= self.delay_after:
                clock.now += .5
            raw = SimpleNamespace(attitude=(self.roll, self.pitch, np.rad2deg(self.yaw)), position_std=.01)
            self.latest = (clock.now, raw)
            self.latest_velocity = (clock.now, tuple(self.velocity))
            return clock.now, tuple(self.position), tuple(self.velocity), self.yaw
    return SimpleNamespace(clock=clock, controller=Controller(), feed=Feed())


def run(result, rig, config, backend, **kwargs):
    return ht.run_host_tracking(result, rig.controller, rig.feed, config, backend, **kwargs)


def test_config_origin_is_relative_to_config_not_working_directory(backend):
    parent = Path(__file__).resolve().parents[2]
    config = ht.load_config(parent/"host_goto_config.json", backend)
    assert Path(config["origin_file"]) == parent/"task_origin.json"


def test_interpolation_wraps_yaw_over_pi():
    x = np.zeros((12, 2))
    x[5] = np.deg2rad([179., -179.])
    ref = ht.reference_at(x, np.array([0., 1.]), .5)
    assert abs(abs(ref[5])-np.pi) < 1e-12


def test_changing_desired_yaw_does_not_rotate_the_task_basis(config, backend):
    config = {**config, "horizontal_kp_level_per_m": 100., "horizontal_kd_level_per_m_s": 0.}
    reference = np.zeros(12)
    reference[0], reference[5] = 1., np.pi/2
    sample = {"position": np.zeros(3), "velocity": np.zeros(3), "yaw": np.pi/2}
    axes = ht.manual_axes(reference, sample, {"yaw_target_rad": .7}, config, backend)
    assert axes == (0, -100, 500, 0)


def test_reference_velocity_uses_world_axes(config, backend):
    reference = np.zeros(12)
    reference[5], reference[6] = np.pi/2, 1.
    sample = {"position": np.zeros(3), "velocity": np.array([0., 1., 0.]), "yaw": np.pi/2}
    assert ht.manual_axes(reference, sample, {"yaw_target_rad": .7}, config, backend) == (0, 0, 500, 0)


def test_final_hold_ignores_nonzero_terminal_feedforward_velocity(config, backend):
    reference = np.zeros(12)
    reference[6] = 1.
    sample = {"position": np.zeros(3), "velocity": np.zeros(3), "yaw": 0.}
    assert ht.manual_axes(reference, sample, {"yaw_target_rad": .7}, config, backend, hold_final=True) == (0, 0, 500, 0)


def test_a50_body_velocity_is_rotated_to_task_axes(config, rig):
    rig.feed.yaw = .7+np.pi/2
    rig.feed.velocity = np.array([1., 0., 0.])
    sample = ht.read_sample(rig.feed, {"origin": (0, 0, 0), "yaw_target_rad": .7}, config, "body")
    np.testing.assert_allclose(sample["velocity"], [0, 1, 0], atol=1e-14)


def test_check_uses_legacy_backend_without_arm_or_manual_commands(config, rig, backend):
    report = run(plan(), rig, config, backend)
    assert report["status"] == "checked"
    assert rig.controller.events == ["connect", "close"] and not rig.controller.sent
    assert rig.feed.closed


def test_confirmation_is_checked_before_connect(config, rig, backend):
    with pytest.raises(ValueError, match="ENABLE_BLUEROV2_HOST_TRACKING"):
        run(plan(), rig, config, backend, execute=True)
    assert not rig.controller.events


def test_wrong_start_is_rejected_before_arm(config, rig, backend):
    result = plan()
    result["nominalTrajectory"][0, 0] = 1.
    report = run(result, rig, config, backend, execute=True, confirmation="ENABLE_BLUEROV2_HOST_TRACKING")
    assert report["reason"] == "start_position_error"
    assert "arm" not in rig.controller.events


def test_missing_origin_is_not_created(config, rig, backend):
    Path(config["origin_file"]).unlink()
    report = run(plan(), rig, config, backend)
    assert "origin is missing" in report["reason"]
    assert not Path(config["origin_file"]).exists()


def test_completion_requires_arrival_hold_and_disarms_once(config, rig, backend):
    report = run(plan(), rig, config, backend, execute=True, confirmation="ENABLE_BLUEROV2_HOST_TRACKING")
    assert report["status"] == "completed"
    assert report["samples"][-1]["elapsed_s"] >= .2
    assert report["disarm_confirmed"] is True
    assert rig.controller.events.count("disarm") == 1
    assert rig.controller.sent[-1] == (0, 0, 500, 0)


def test_schedule_end_without_arrival_is_aborted(config, rig, backend):
    result = plan()
    result["nominalTrajectory"][0, -1] = .4
    report = run(result, rig, config, backend, execute=True, confirmation="ENABLE_BLUEROV2_HOST_TRACKING")
    assert report["status"] == "aborted" and report["reason"] == "goal_not_reached"
    assert report["disarm_confirmed"]


@pytest.mark.parametrize("fault", ["stale", "slow", "abort"])
def test_runtime_fault_cleans_up(config, rig, backend, fault):
    if fault == "stale": rig.feed.fail_after = 2
    if fault == "slow": rig.feed.delay_after = 2
    if fault == "abort":
        original = rig.controller.check_runtime_safety
        def check():
            original()
            rig.controller.stop_requested = True
        rig.controller.check_runtime_safety = check
    report = run(plan(), rig, config, backend, execute=True, confirmation="ENABLE_BLUEROV2_HOST_TRACKING")
    assert report["status"] == "aborted"
    assert report["disarm_confirmed"] and rig.feed.closed
    assert all(axes == (0, 0, 500, 0) for axes in rig.controller.sent)


def test_failed_arm_still_disarms(config, rig, backend):
    original = rig.controller.request_arm
    def arm():
        original()
        raise backend.ControlError("arm acknowledgement timed out")
    rig.controller.request_arm = arm
    report = run(plan(), rig, config, backend, execute=True, confirmation="ENABLE_BLUEROV2_HOST_TRACKING")
    assert report["status"] == "aborted" and report["disarm_confirmed"]


def test_failed_disarm_does_not_report_completed(config, rig, backend):
    rig.controller.stop_and_disarm = lambda: False
    report = run(plan(), rig, config, backend, execute=True, confirmation="ENABLE_BLUEROV2_HOST_TRACKING")
    assert report["status"] == "aborted" and report["reason"] == "disarm_unconfirmed"


@pytest.mark.parametrize("times", [[1., 2.], [0., np.nan], [0., 0.]])
def test_invalid_plan_rejected_before_connect(times, config, rig, backend):
    result = plan()
    result["time"] = np.array(times)
    with pytest.raises(ValueError):
        run(result, rig, config, backend)
    assert not rig.controller.events


def test_busy_dvl_socket_cannot_starve_control_loop(config, backend, rig):
    pose = dict(x=0, y=0, z=0, roll=0, pitch=0, yaw=0, std=.01, status=0, ts=1, format="position_local")
    velocity = dict(time=1, vx=0, vy=0, vz=0, fom=.01, status=0, velocity_valid=True,
                    transducers=[{"beam_valid": True}]*4)
    payload = (json.dumps(pose)+"\n"+json.dumps(velocity)+"\n").encode()
    class FloodSocket:
        calls = 0
        def recv(self, size):
            self.calls += 1
            return payload
        def close(self): pass
    feed = ht.make_feed(backend, config)
    sock = FloodSocket()
    feed.sock = sock
    feed.current(.5, 1.)
    assert sock.calls == 8
    assert feed.latest is not None and feed.latest_velocity is not None


def test_dvl_bridge_parser_error_still_disarms(config, rig, backend):
    original = rig.feed.current
    def current(*args):
        if rig.controller.control_responsibility:
            raise RuntimeError("DVL malformed dead-reckoning report")
        return original(*args)
    rig.feed.current = current
    report = run(plan(), rig, config, backend, execute=True, confirmation="ENABLE_BLUEROV2_HOST_TRACKING")
    assert report["status"] == "aborted"
    assert "malformed" in report["reason"] and report["disarm_confirmed"]


def test_connection_permission_error_is_reported_and_closes(config, rig, backend):
    def denied():
        raise backend.ControlError("Operation not permitted")
    rig.controller.connect = denied
    report = run(plan(), rig, config, backend)
    assert report["status"] == "aborted" and "Operation not permitted" in report["reason"]
    assert rig.feed.closed and not rig.controller.sent
