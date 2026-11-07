"""Offline tests for the isolated stock-firmware diagnostic, never a live backend."""
from collections import deque
from types import SimpleNamespace
import time

import pytest
from pymavlink import mavutil

from tools.verify_passthrough_sitl import ProbeLink, pwm_frame
from tools.verify_passthrough_source import function_body


@pytest.mark.parametrize('value',[
    [1500]*5,[1500]*7,[0]*6,[1099]*6,[1901]*6,
    [1500.1]*6,[float('nan')]*6,[float('inf')]*6,[float('-inf')]*6,
])
def test_passthrough_diagnostic_rejects_invalid_frame(value):
    with pytest.raises(ValueError):
        pwm_frame(value)


def test_six_pwm_values_in_one_real_mavlink_packet():
    requested = [1100,1400,1499,1500,1600,1900]
    channels = pwm_frame(requested)
    assert channels == tuple(requested)+(65535,65535)
    mav = mavutil.mavlink.MAVLink(None,srcSystem=255,srcComponent=191)
    msg = mavutil.mavlink.MAVLink_rc_channels_override_message(42,1,*channels)
    wire = msg.pack(mav)
    parser = mavutil.mavlink.MAVLink(None)
    parsed = parser.parse_buffer(wire)
    assert len(parsed) == 1
    assert parsed[0].get_type() == 'RC_CHANNELS_OVERRIDE'
    assert parsed[0].target_system == 42 and parsed[0].target_component == 1
    assert [getattr(parsed[0],f'chan{i}_raw') for i in range(1,9)] == list(channels)
    assert parsed[0].get_srcSystem() == 255 and parsed[0].get_srcComponent() == 191


def test_probelink_sends_only_one_override_for_all_six_channels():
    calls = []
    master = SimpleNamespace(mav=SimpleNamespace(rc_channels_override_send=lambda *a:calls.append(a)))
    link = ProbeLink(master)
    link.override([1500,1501,1502,1503,1504,1505])
    assert calls == [(42,1,1500,1501,1502,1503,1504,1505,65535,65535)]
    assert link.sent_frames == 1
    with pytest.raises(ValueError):
        link.override([2000]*6)
    assert len(calls) == 1


def test_parameter_check_requires_a_new_reply(monkeypatch):
    clock = SimpleNamespace(now=10.)
    monkeypatch.setattr(time,'monotonic',lambda:clock.now)
    monkeypatch.setattr(time,'sleep',lambda dt:setattr(clock,'now',clock.now+dt))
    requests = []
    master = SimpleNamespace(mav=SimpleNamespace(param_request_read_send=lambda *a:requests.append(a)))
    link = ProbeLink(master)
    link.serial = 17
    link.parameters['SERVO1_FUNCTION'] = (17,1.)
    monkeypatch.setattr(link,'poll',lambda:None)
    with pytest.raises(TimeoutError):
        link.parameter('SERVO1_FUNCTION')
    assert requests == [(42,1,b'SERVO1_FUNCTION',-1)]


def test_parameter_write_does_not_accept_wrong_value_reply(monkeypatch):
    clock = SimpleNamespace(now=10.)
    monkeypatch.setattr(time,'monotonic',lambda:clock.now)
    monkeypatch.setattr(time,'sleep',lambda dt:setattr(clock,'now',clock.now+dt))
    requests = []
    master = SimpleNamespace(mav=SimpleNamespace(param_set_send=lambda *a:requests.append(a)))
    link = ProbeLink(master)
    def poll():
        link.serial += 1
        link.parameters['SERVO1_FUNCTION'] = (link.serial,33.)
    monkeypatch.setattr(link,'poll',poll)
    with pytest.raises(TimeoutError):
        link.parameter('SERVO1_FUNCTION',1)
    assert len(requests) == 1 and requests[0][3] == 1.


def test_recorded_message_must_be_from_target_fc(monkeypatch):
    clock = SimpleNamespace(now=10.)
    monkeypatch.setattr(time,'monotonic',lambda:clock.now)
    def message(sys,comp):
        return SimpleNamespace(get_srcSystem=lambda:sys,get_srcComponent=lambda:comp,
                               get_type=lambda:'PARAM_VALUE',param_id='SERVO1_FUNCTION',param_value=1.)
    queue = deque([message(41,1),message(42,191),message(42,1)])
    master = SimpleNamespace(recv_match=lambda **_:queue.popleft() if queue else None,
                            mav=SimpleNamespace(heartbeat_send=lambda *_:None))
    link = ProbeLink(master)
    link.poll()
    assert link.serial == 1 and link.parameters == {'SERVO1_FUNCTION':(1,1.)}


def test_source_extraction_keeps_original_function_body():
    source = 'prefix\nvoid f(int x) {if(x) {x++;}}\nvoid g() {}'
    assert function_body(source,'void f(') == 'void f(int x) {if(x) {x++;}}'
    with pytest.raises(ValueError):
        function_body('void f() {','void f(')
