from types import SimpleNamespace

import numpy as np
import pytest

from bluerov2 import parameters
from bluerov2.execution import execute_lbts_plan


def plan():
    return {
        "success": True,
        "nominalTrajectory": np.zeros((12, 2)),
        "controlTrajectory": np.ones((6, 1)),
        "time": np.array([0.0, 0.1]),
        "feedbackGains": [np.zeros((6, 12))],
        "model": parameters().model,
        "world": None,
    }


def options(**kwargs):
    return dict(enable_actuation=True, confirmation="ENABLE_BLUEROV2_THRUST",
                real_time=False, verbose=False, **kwargs)


@pytest.mark.parametrize("times", [[1.0, 1.1], [0.0, np.nan], [0.0, np.inf], [0.0, 0.0]])
def test_invalid_plan_time_rejected_before_initialize(times):
    result = plan()
    result["time"] = times
    initialized = []
    with pytest.raises(ValueError, match="time"):
        execute_lbts_plan(result, {"initialize": lambda: initialized.append(True),
                                  "read_state": lambda: np.zeros(12)})
    assert not initialized


@pytest.mark.parametrize("field", ["nominalTrajectory", "controlTrajectory", "feedbackGains"])
def test_nonfinite_plan_rejected(field):
    result = plan()
    if field == "feedbackGains":
        result[field][0][0, 0] = np.nan
    else:
        result[field][0, 0] = np.nan
    with pytest.raises(ValueError, match="finite"):
        execute_lbts_plan(result, {"read_state": lambda: np.zeros(12)})


def test_stale_preflight_does_not_start_or_send_nonzero():
    events = []
    io = {"read_state": lambda: {"state": np.zeros(12), "age": 1.0},
          "send_thruster_forces": lambda f: events.append(np.asarray(f)),
          "on_start": lambda *_: events.append("start")}
    with pytest.raises(ValueError, match="fresh"):
        execute_lbts_plan(plan(), io, options())
    assert len(events) == 1 and np.array_equal(events[0], np.zeros(6))


def test_initialize_failure_still_stops_once():
    stopped = []
    def initialize():
        raise OSError("connection failed after partial initialization")
    with pytest.raises(OSError):
        execute_lbts_plan(plan(), {"initialize": initialize,
                                  "read_state": lambda: np.zeros(12),
                                  "stop": lambda: stopped.append(True)})
    assert stopped == [True]


def test_cleanup_does_not_send_after_close():
    events = []
    io = {"read_state": lambda: {"state": np.zeros(12), "age": 0.0},
          "send_thruster_forces": lambda f: events.append(("send", np.asarray(f))),
          "stop": lambda: events.append(("close", None))}
    execution = execute_lbts_plan(plan(), io, options())
    assert execution["status"] == "completed"
    assert [event[0] for event in events] == ["send", "send", "close"]
    assert np.array_equal(events[-2][1], np.zeros(6))


def test_operator_abort_during_final_interval(monkeypatch):
    clock = SimpleNamespace(now=0.0)
    monkeypatch.setattr("bluerov2.execution.time.monotonic", lambda: clock.now)
    monkeypatch.setattr("bluerov2.execution.time.sleep", lambda dt: setattr(clock, "now", clock.now + dt))
    sent = []
    io = {"read_state": lambda: {"state": np.zeros(12), "age": 0.0},
          "send_thruster_forces": lambda f: sent.append((clock.now, np.asarray(f))),
          "abort_requested": lambda: clock.now >= 0.03}
    execution = execute_lbts_plan(plan(), io, {**options(), "real_time": True})
    assert execution["status"] == "aborted"
    assert execution["abort_reason"] == "operator_abort"
    assert clock.now < 0.1
    assert np.array_equal(sent[-1][1], np.zeros(6))


@pytest.mark.parametrize("value", ["false", 1, None])
def test_actuation_flag_must_be_boolean(value):
    with pytest.raises(ValueError, match="boolean"):
        execute_lbts_plan(plan(), {"read_state": lambda: np.zeros(12)},
                          {"enable_actuation": value})


@pytest.mark.parametrize("value", [np.nan, np.inf, -1.0])
def test_invalid_tracking_limit_rejected(value):
    with pytest.raises(ValueError, match="max_position_tracking_error"):
        execute_lbts_plan(plan(), {"read_state": lambda: np.zeros(12)},
                          {"max_position_tracking_error": value})


def test_control_arithmetic_overflow_aborts_without_nonzero_output():
    result = plan()
    result["controlTrajectory"][:] = np.finfo(float).max
    result["feedbackGains"][0][:, 0] = -np.finfo(float).max
    state = np.zeros(12)
    state[0] = .1
    sent = []
    io = {"read_state": lambda: {"state": state, "age": 0.0},
          "send_thruster_forces": lambda f: sent.append(np.asarray(f))}
    execution = execute_lbts_plan(result, io, options())
    assert execution["status"] == "aborted"
    assert execution["abort_reason"] == "invalid_control_command"
    assert len(sent) == 1 and np.array_equal(sent[0], np.zeros(6))


def test_slow_state_read_aborts_before_sending_late_command(monkeypatch):
    clock = SimpleNamespace(now=0.0, reads=0)
    monkeypatch.setattr("bluerov2.execution.time.monotonic", lambda: clock.now)
    def read():
        clock.reads += 1
        if clock.reads > 1:
            clock.now += .5
        return {"state": np.zeros(12), "age": 0.0}
    sent = []
    execution = execute_lbts_plan(plan(), {"read_state": read,
        "send_thruster_forces": lambda f: sent.append(np.asarray(f))},
        {**options(), "real_time": True})
    assert execution["abort_reason"] == "timing_overrun"
    assert len(sent) == 1 and np.array_equal(sent[0], np.zeros(6))
