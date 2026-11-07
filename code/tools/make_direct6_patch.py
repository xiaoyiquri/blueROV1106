"""Generate the reviewable patch against exact upstream ArduSub-3.5.4 source files."""
from pathlib import Path
import difflib
import hashlib
import json
import argparse
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument("source_root",type=Path,help="unmodified checkout of ArduSub-3.5.4")
source_root=parser.parse_args().source_root
ROOT=Path(__file__).resolve().parents[1]
changes={}
def edit(name,path,fn):
    before=(source_root/path).read_text()
    expected=json.loads((ROOT/'firmware/ardusub_3_5_4/source_manifest.json').read_text())['files'][path]
    if hashlib.sha256(before.encode()).hexdigest()!=expected:raise ValueError('source differs from verified ArduSub-3.5.4: '+path)
    after=fn(before)
    assert after != before
    changes[path]=(before,after)

edit('Sub.h','ArduSub/Sub.h',lambda s:s.replace('#include "AP_Arming_Sub.h"','#include "AP_Arming_Sub.h"\n#include "direct6_core.h"'))
# Put methods into the existing private section: GCS_MAVLINK_Sub is already a friend.
a,b=changes['ArduSub/Sub.h']
marker='    void motors_output();'
assert marker in b
b=b.replace(marker,marker+'''
    direct6::Core direct6_output;
    uint32_t direct6_last_legacy_ms = 0;
    uint8_t direct6_reply_chan = 255;
    bool direct6_ready() const;
    bool direct6_guard(mavlink_message_t* msg);
    void direct6_handle(mavlink_message_t* msg, mavlink_channel_t chan);
    void direct6_reply();''')
changes['ArduSub/Sub.h']=a,b
edit('motors.cpp','ArduSub/motors.cpp',lambda s:s.replace('    // check if we are performing the motor test','''    direct6_output.tick(AP_HAL::millis(), motors.armed(), direct6_ready());
    if (direct6_output.state != direct6::IDLE) {
        // All six physical MAIN outputs in one HAL batch, bypass all control/mixing.
        hal.rcout->cork();
        for (uint8_t i=0; i<6; i++) hal.rcout->write(i, direct6_output.pwm[i]);
        hal.rcout->push();
        direct6_reply();
        return; // never let motors.output() overwrite the dedicated path
    }
    direct6_reply();
    // check if we are performing the motor test'''))
edit('ArduSub.cpp','ArduSub/ArduSub.cpp',lambda s:s.replace('    ServoRelayEvents.update_events();', '    if (direct6_output.state == direct6::IDLE) ServoRelayEvents.update_events();'))
edit('failsafe.cpp','ArduSub/failsafe.cpp',lambda s:s.replace('void Sub::failsafe_pilot_input_check()\n{', '''void Sub::failsafe_pilot_input_check()
{
    // Direct commands have their own 200 ms execution watchdog; no pilot axes are sent.
    if (direct6_output.state == direct6::HELD) {
        failsafe.pilot_input = false;
        return;
    }'''))
edit('GCS_Mavlink.cpp','ArduSub/GCS_Mavlink.cpp',lambda s:s.replace('    switch (msg->msgid) {\n\n    case MAVLINK_MSG_ID_HEARTBEAT:', '''    if (msg->msgid == MAVLINK_MSG_ID_DATA64) {
        sub.direct6_handle(msg, chan);
        return;
    }
    if (sub.direct6_guard(msg)) return;
    switch (msg->msgid) {

    case MAVLINK_MSG_ID_HEARTBEAT:'''))
extra='''// SPDX-License-Identifier: GPL-3.0-or-later
#include "Sub.h"

bool Sub::direct6_ready() const
{
    // Only standard Vectored on physical MAIN1..MAIN6, with no motor test.
    return g.frame_configuration == AP_Motors6DOF::SUB_FRAME_VECTORED &&
           control_mode == MANUAL && !ap.motor_test;
}

void Sub::direct6_handle(mavlink_message_t* msg, mavlink_channel_t channel)
{
    mavlink_data64_t packet;
    mavlink_msg_data64_decode(msg, &packet);
    if (packet.type != 0xD6 || packet.len != 34) return;
    const bool quiet = direct6_output.state != direct6::IDLE ||
                       AP_HAL::millis()-direct6_last_legacy_ms > 1000;
    if (direct6_output.handle(packet.data, sizeof(packet.data), msg->sysid, msg->compid,
                             mavlink_system.sysid, mavlink_system.compid,
                             AP_HAL::millis(), motors.armed(), direct6_ready() && quiet)) {
        direct6_reply_chan = uint8_t(channel);
        if (direct6_output.state == direct6::HELD) failsafe.pilot_input = false;
    }
}

bool Sub::direct6_guard(mavlink_message_t* msg)
{
    bool actuator_input = false;
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_MANUAL_CONTROL:
    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
    case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:
    case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
    case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT:
    case MAVLINK_MSG_ID_SET_MODE:
    case MAVLINK_MSG_ID_COMMAND_INT:
    case MAVLINK_MSG_ID_PARAM_SET:
        actuator_input = true; break;
    case MAVLINK_MSG_ID_COMMAND_LONG: {
        mavlink_command_long_t p;
        mavlink_msg_command_long_decode(msg, &p);
        if (p.command == MAV_CMD_COMPONENT_ARM_DISARM) {
            // Anyone may disarm. Only the session owner may arm while leased.
            if (p.param1 == 0) return false;
            if (direct6_output.state != direct6::IDLE)
                return msg->sysid != direct6_output.owner_system ||
                       msg->compid != direct6_output.owner_component ||
                       direct6_output.state != direct6::HELD;
            return false;
        }
        if (p.command == MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES) return false;
        actuator_input = true; break;
    }
    default: break;
    }
    if (actuator_input) {
        direct6_last_legacy_ms = AP_HAL::millis();
        return direct6_output.state != direct6::IDLE;
    }
    return false;
}

void Sub::direct6_reply()
{
    if (direct6_reply_chan == 255) return;
    uint16_t readback[6];
    for (uint8_t i=0; i<6; i++) readback[i] = hal.rcout->read(i);
    uint8_t data[64];
    direct6_output.status(data, AP_HAL::millis(), motors.armed(), direct6_ready(), readback);
    mavlink_msg_data64_send(mavlink_channel_t(direct6_reply_chan),
                                       0xD6, 34, data);
    direct6_reply_chan = 255;
}
'''
changes['ArduSub/direct6.cpp']=('',extra)
changes['ArduSub/direct6_core.h']=('',(ROOT/'firmware/ardusub_3_5_4/direct6_core.h').read_text())
patch=''.join(''.join(difflib.unified_diff(a.splitlines(True),b.splitlines(True),fromfile='a/'+p if a else '/dev/null',tofile='b/'+p)) for p,(a,b) in changes.items())
out=ROOT/'firmware/ardusub_3_5_4'
(out/'direct6.patch').write_text(patch)
(out/'source_manifest.json').write_text(json.dumps({'upstream_tag':'ArduSub-3.5.4','upstream_commit':'05fed49b8ee1db3c6bca924e0b334b2e0ece980d','upstream_url':'https://github.com/ArduPilot/ardupilot/tree/ArduSub-3.5.4','files':{p:hashlib.sha256(a.encode()).hexdigest() for p,(a,b) in changes.items() if a}},indent=2)+'\n')
print(out/'direct6.patch')
