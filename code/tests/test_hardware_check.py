from collections import deque
from types import SimpleNamespace
import json
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from bluerov2 import hardware_check as hw


def attitude(yaw=0.0, roll=0.0, pitch=0.0):
    return dict(roll=roll, pitch=pitch, yaw=yaw,
                rollspeed=0.1, pitchspeed=0.2, yawspeed=0.3)


def position(velocity=(0.0, 1.0, 0.0)):
    return dict(x=1.0, y=2.0, z=3.0, **dict(zip(("vx", "vy", "vz"), velocity)))


def test_east_facing_vehicle_has_forward_east_velocity():
    state = hw.mavlink_state(attitude(yaw=np.pi/2), position())
    np.testing.assert_allclose(state[:3], [1, 2, 3])
    np.testing.assert_allclose(state[6:9], [1, 0, 0], atol=1e-14)
    np.testing.assert_allclose(state[9:], [.1, .2, .3])


def test_full_attitude_rotation_and_body_rates():
    angles = [.2, -.3, .7]
    velocity = [.3, -.1, .2]
    state = hw.mavlink_state(attitude(roll=angles[0], pitch=angles[1], yaw=angles[2]), position(velocity))
    rotation = Rotation.from_euler("ZYX", angles[::-1])
    np.testing.assert_allclose(rotation.apply(state[6:9]), velocity, atol=1e-14)
    np.testing.assert_allclose(state[9:], [.1, .2, .3])


def test_missing_or_nonfinite_mavlink_fields_are_not_zero_filled():
    with pytest.raises(KeyError):
        hw.mavlink_state(attitude(), {})
    with pytest.raises(ValueError, match="finite"):
        hw.mavlink_state(attitude(yaw=np.nan), position())


def test_a50_pose_converts_degrees_and_keeps_position():
    parsed = hw.dvl_report(dict(x=1, y=2, z=3, roll=10, pitch=-20, yaw=90, std=.05, status=0))
    np.testing.assert_allclose(parsed["pose_m_rad"], [1, 2, 3, *np.deg2rad([10, -20, 90])])
    assert "alignment required" in parsed["frame"]


@pytest.mark.parametrize("std", [-1, np.nan, np.inf])
def test_a50_invalid_uncertainty_rejected(std):
    with pytest.raises(ValueError):
        hw.dvl_report(dict(x=0, y=0, z=0, roll=0, pitch=0, yaw=0, std=std, status=0))


class Message:
    def __init__(self, kind, data, source=(1, 1)):
        self.kind, self.data, self.source = kind, data, source
    def get_type(self): return self.kind
    def get_srcSystem(self): return self.source[0]
    def get_srcComponent(self): return self.source[1]
    def to_dict(self): return self.data
    def __getattr__(self, key): return self.data[key]


@pytest.fixture
def fake_link(monkeypatch):
    clock = SimpleNamespace(now=0.0)
    monkeypatch.setattr(hw.time, "monotonic", lambda: clock.now)
    monkeypatch.setattr(hw.time, "sleep", lambda dt: setattr(clock, "now", clock.now+dt))
    master = SimpleNamespace(messages=deque(), closed=False)
    master.recv_match = lambda **_: master.messages.popleft() if master.messages else None
    master.close = lambda: setattr(master, "closed", True)
    mavutil = SimpleNamespace(mavlink_connection=lambda *a, **k: master,
        mavlink=SimpleNamespace(MAV_TYPE_SUBMARINE=12, MAV_AUTOPILOT_ARDUPILOTMEGA=3,
                                MAV_MODE_FLAG_SAFETY_ARMED=128))
    monkeypatch.setitem(sys.modules, "pymavlink", SimpleNamespace(mavutil=mavutil))
    return master, clock


def heartbeat(source=(1, 1)):
    return Message("HEARTBEAT", dict(type=12, autopilot=3, base_mode=0), source)


def test_passive_capture_selects_autopilot_and_converts_state(fake_link):
    master, _ = fake_link
    master.messages.extend([heartbeat(), Message("ATTITUDE", attitude(yaw=np.pi/2)),
        Message("ATTITUDE", attitude(yaw=0), source=(255, 190)),
        Message("LOCAL_POSITION_NED", position())])
    report = hw._collect_mavlink("fake", 115200, .05, .1)
    assert report["complete_fresh_state"] and master.closed
    np.testing.assert_allclose(report["state"][6:9], [1, 0, 0], atol=1e-14)


def test_missing_local_position_is_reported(fake_link):
    master, _ = fake_link
    master.messages.extend([heartbeat(), Message("ATTITUDE", attitude())])
    report = hw._collect_mavlink("fake", 115200, .05, .1)
    assert report["missing_state_messages"] == ["LOCAL_POSITION_NED"]
    assert report["state"] is None and not report["complete_fresh_state"]
    assert master.closed


def test_stale_telemetry_is_not_complete(fake_link):
    master, _ = fake_link
    master.messages.extend([heartbeat(), Message("ATTITUDE", attitude()), Message("LOCAL_POSITION_NED", position())])
    report = hw._collect_mavlink("fake", 115200, .2, .05)
    assert not report["complete_fresh_state"]


def test_multiple_autopilots_do_not_produce_a_state(fake_link):
    master, _ = fake_link
    master.messages.extend([heartbeat(), heartbeat((2, 1))])
    report = hw._collect_mavlink("fake", 115200, .05, .1)
    assert report["state"] is None and "multiple" in report["error"]


def test_dvl_fragmented_json_stream(monkeypatch, fake_link):
    _, clock = fake_link
    pose = dict(x=1, y=2, z=3, roll=0, pitch=0, yaw=90, std=.05, status=0)
    velocity = dict(vx=.1, vy=0, vz=0, fom=.01, status=0, velocity_valid=True,
                    transducers=[dict(beam_valid=True)]*4)
    payload = (json.dumps(pose)+"\n"+json.dumps(velocity)+"\n").encode()
    chunks = deque([payload[:20], payload[20:]])
    class Socket:
        def __enter__(self): return self
        def __exit__(self, *args): pass
        def settimeout(self, timeout): self.timeout = timeout
        def recv(self, size):
            if chunks: return chunks.popleft()
            clock.now += self.timeout
            raise hw.socket.timeout()
    monkeypatch.setattr(hw.socket, "create_connection", lambda *a, **k: Socket())
    report = hw._collect_dvl("fake", 16171, .05, .1)
    assert report["valid_fresh_reports"]
    assert report["report_counts"] == {"position_local": 1, "velocity": 1}


def test_network_denial_is_recorded(monkeypatch):
    def denied(*args, **kwargs):
        raise PermissionError(1, "Operation not permitted")
    monkeypatch.setattr(hw.socket, "create_connection", denied)
    report = hw._collect_dvl("fake", 16171, .05, .1)
    assert not report["connected"] and "Operation not permitted" in report["error"]


@pytest.mark.parametrize("duration", [0, -1, np.nan, np.inf, 61])
def test_invalid_duration_rejected(duration):
    with pytest.raises(ValueError, match="duration"):
        hw.check_hardware("fake", duration=duration)


def test_cli_reads_config_and_writes_report(tmp_path, fake_link, capsys):
    master, _ = fake_link
    master.messages.extend([heartbeat(), Message("ATTITUDE", attitude()),
                            Message("LOCAL_POSITION_NED", position())])
    config = tmp_path/"config.json"
    config.write_text(json.dumps({"connection": "fake", "dvl_ip": "unused"}))
    output = tmp_path/"report.json"
    status = hw.main(["--config", str(config), "--duration", ".05",
                      "--skip-dvl", "--output", str(output)])
    report = json.loads(output.read_text())
    assert status == 0 and report["telemetry_complete"]
    assert report["passive"] and report["dvl"] is None
    assert json.loads(capsys.readouterr().out) == report
