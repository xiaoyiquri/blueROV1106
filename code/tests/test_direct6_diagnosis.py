"""Read-only command whitelist; no device connections."""
import pytest
from pymavlink import mavutil
from tools.diagnose_direct6 import validate_readonly_message
from bluerov2.thruster_output import encode_frame, QUERY, ACQUIRE, FRAME, STOP, RELEASE

mav=mavutil.mavlink


def test_readonly_version_parameter_and_capability_query_allowed():
    messages=[mav.MAVLink_command_long_message(1,1,520,0,1,0,0,0,0,0,0),
              mav.MAVLink_param_request_read_message(1,1,b'FRAME_CONFIG',-1),
              mav.MAVLink_data64_message(0xD6,34,list(encode_frame(QUERY,(1,1))))]
    for msg in messages:validate_readonly_message(msg,(1,1))


@pytest.mark.parametrize('op',[ACQUIRE,FRAME,STOP,RELEASE])
def test_readonly_rejects_every_actuator_session_operation(op):
    msg=mav.MAVLink_data64_message(0xD6,34,list(encode_frame(op,(1,1))))
    with pytest.raises(ValueError):validate_readonly_message(msg,(1,1))


def test_readonly_rejects_arm_pwm_override_parameters_heartbeat_and_wrong_target():
    messages=[mav.MAVLink_command_long_message(1,1,400,0,1,0,0,0,0,0,0),
              mav.MAVLink_param_set_message(1,1,b'SERVO1_FUNCTION',1,9),
              mav.MAVLink_rc_channels_override_message(1,1,*([1500]*8)),
              mav.MAVLink_heartbeat_message(6,8,0,0,4,3),
              mav.MAVLink_param_request_read_message(2,1,b'FRAME_CONFIG',-1),
              mav.MAVLink_data64_message(0xD6,34,list(encode_frame(QUERY,(2,1))))]
    for msg in messages:
        with pytest.raises(ValueError):validate_readonly_message(msg,(1,1))
