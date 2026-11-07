"""Compile exact ArduSub 3.5.4 function bodies with a recording HAL.

This is a source-level regression, not a board/PX4 or complete SITL test.
No connection to a robot is opened and no production source is modified.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def function_body(source, signature):
    start = source.index(signature)
    brace = source.index('{', start)
    depth = 1
    end = brace + 1
    while depth and end < len(source):
        if source[end] == '{':
            depth += 1
        elif source[end] == '}':
            depth -= 1
        end += 1
    if depth:
        raise ValueError('unterminated function: ' + signature)
    return source[start:end]


FUNCTIONS = {
    'libraries/SRV_Channel/SRV_Channel_aux.cpp': ['void SRV_Channel::output_ch(void)'],
    'libraries/AP_Motors/AP_Motors_Class.cpp': ['void AP_Motors::rc_write(uint8_t chan, uint16_t pwm)'],
    'libraries/AP_Motors/AP_Motors6DOF.cpp': ['void AP_Motors6DOF::output_to_motors()'],
    'ArduSub/joystick.cpp': ['void Sub::set_neutral_controls()'],
    'ArduSub/failsafe.cpp': ['void Sub::failsafe_pilot_input_check()'],
}

STUBS = r'''
#include <array>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <vector>
#define HAL_BOARD_SITL 1
#ifndef CONFIG_HAL_BOARD
#define CONFIG_HAL_BOARD 0
#endif
#define FS_PILOT_INPUT_DISABLED 0
#define FS_PILOT_INPUT_DISARM 2
#define ERROR_SUBSYSTEM_INPUT 0
#define ERROR_CODE_FAILSAFE_OCCURRED 0
#define MAV_SEVERITY_CRITICAL 0
#define AP_MOTORS_MAX_NUM_MOTORS 8
struct RCOutput {
    std::array<uint16_t,16> pwm{};
    void write(uint8_t c,uint16_t v) {pwm.at(c)=v;}
    void cork() {}
    void push() {}
};
struct RCInput {
    std::array<uint16_t,16> pwm{};
    void set_overrides(int16_t *v,uint8_t n) {
        for (uint8_t i=0;i<n;i++) if(v[i]>=0) pwm[i]=v[i];
    }
};
namespace AP_HAL {
    uint32_t now=1000;
    uint32_t millis() {return now;}
    struct HAL {RCOutput *rcout;RCInput *rcin;};
}
RCOutput recording_output;
RCInput recording_input;
const AP_HAL::HAL hal{&recording_output,&recording_input};
struct RC_Channel {
    uint16_t radio_in=1500;
    uint16_t get_radio_in() {return radio_in;}
    uint16_t get_radio_trim() {return 1500;}
};
struct RC_Channels {
    static RC_Channel channels[16];
    static RC_Channel *rc_channel(int c) {return &channels[c];}
    static void set_pwm_all() {
        for(int c=0;c<16;c++) channels[c].radio_in=recording_input.pwm[c];
    }
};
RC_Channel RC_Channels::channels[16];
struct SRV_Channels {static bool passthrough_disabled() {return false;}};
struct SRV_Channel {
    enum {k_manual=1,k_rcin1=51,k_rcin16=66,k_motor1=33,k_motor8=40};
    int function=1;
    uint8_t ch_num=0;
    uint16_t output_pwm=1500;
    void output_ch(void);
};
struct AP_Motors {
    uint16_t _motor_map_mask=0x3f;
    uint8_t _motor_map[8]={0,1,2,3,4,5,6,7};
    enum {PWM_TYPE_NORMAL=0,PWM_TYPE_ONESHOT125=1};
    int _pwm_type=PWM_TYPE_NORMAL;
    uint16_t _motor_fast_mask=0x3f;
    void rc_write(uint8_t chan,uint16_t pwm);
};
struct AP_Motors6DOF:AP_Motors {
    enum {SHUT_DOWN,SPIN_WHEN_ARMED,SPOOL_UP,THROTTLE_UNLIMITED,SPOOL_DOWN};
    int _spool_mode=SHUT_DOWN;
    bool motor_enabled[8]={true,true,true,true,true,true,false,false};
    float _thrust_rpyt_out[8]={.2,-.2,.1,-.1,.3,-.3,0,0};
    int16_t calc_thrust_to_pwm(float v) const {return int16_t(1500+400*v);}
    void output_to_motors();
};
struct Sub {
    struct {int failsafe_pilot_input=2;float failsafe_pilot_input_timeout=.2f;} g;
    struct {bool pilot_input=false;uint32_t last_pilot_input_ms=1000;} failsafe;
    int pitchTrim=0,rollTrim=0,disarm_calls=0;
    void Log_Write_Error(int,int) {}
    void gcs_send_text(int,const char*) {}
    void init_disarm_motors() {disarm_calls++;}
    void set_neutral_controls();
    void failsafe_pilot_input_check();
};
'''

MAIN = r'''
int main() {
    const std::array<uint16_t,6> commanded{1540,1460,1550,1450,1560,1440};
    const std::array<uint16_t,6> neutral{1500,1500,1500,1500,1500,1500};
    auto actual=[]() {std::array<uint16_t,6> x{};for(int i=0;i<6;i++) x[i]=recording_output.pwm[i];return x;};
    SRV_Channel channels[6];
    AP_Motors6DOF motors;
    auto passthrough=[&]() {RC_Channels::set_pwm_all();for(auto &c:channels) c.output_ch();};
    auto receive=[&](Sub &s) {
        for(int i=0;i<6;i++) recording_input.pwm[i]=commanded[i];
        s.failsafe.last_pilot_input_ms=AP_HAL::now;
    };
    for(int mode: {1,51}) {
        for(int i=0;i<6;i++) {channels[i].ch_num=i;channels[i].function=mode==1?1:51+i;recording_input.pwm[i]=commanded[i];}
        passthrough();assert(actual()==commanded);
        motors._spool_mode=AP_Motors6DOF::SHUT_DOWN;
        motors.output_to_motors();assert(actual()==neutral);
        passthrough();assert(actual()==commanded);
        motors._spool_mode=AP_Motors6DOF::THROTTLE_UNLIMITED;
        motors.output_to_motors();assert(actual()!=commanded);
    }
    Sub sub;
    AP_HAL::now=1000;receive(sub);passthrough();
    AP_HAL::now=1201;sub.failsafe_pilot_input_check();passthrough();
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    assert(sub.failsafe.pilot_input && sub.disarm_calls==1 && actual()==neutral);
    // Model a delayed accepted RC packet by updating the same fields as GCS_Mavlink.cpp.
    // No arming action is made here; passthrough itself has no arming/session latch.
    AP_HAL::now=1220;receive(sub);sub.failsafe_pilot_input_check();passthrough();
    assert(!sub.failsafe.pilot_input && sub.disarm_calls==1 && actual()==commanded);
    std::puts("{\"direct_body_test\":true,\"motor_loop_overwrites_function_1_and_51_56\":true,\"board_branch_input_timeout_neutralizes\":true,\"accepted_late_override_clears_input_failsafe\":true,\"passthrough_body_has_no_disarmed_gate\":true}");
#else
    assert(sub.disarm_calls==0 && actual()==commanded);
    std::puts("{\"direct_body_test\":true,\"motor_loop_overwrites_function_1_and_51_56\":true,\"stock_sitl_skips_pilot_timeout\":true}");
#endif
}
'''


def verify(source, output):
    source, output = Path(source).resolve(), Path(output).resolve()
    bodies = []
    hashes = {}
    for name, signatures in FUNCTIONS.items():
        raw = (source/name).read_bytes()
        hashes[name] = hashlib.sha256(raw).hexdigest()
        for signature in signatures:
            bodies.append(function_body(raw.decode(), signature))
    if any('direct6' in body for body in bodies):
        raise ValueError('Use the unpatched upstream source, not the direct6 working tree')
    report = {'source':str(source), 'function_source_sha256':hashes,
              'physical_vehicle_used':False, 'evidence_type':'original_function_bodies_with_stub_HAL',
              'limitations':['Not an electrical output test or a complete-board watchdog test',
                             'Stub setup supplies motor enable/map and RC inputs explicitly']}
    output.parent.mkdir(parents=True, exist_ok=True)
    code = STUBS + '\n'.join(bodies) + MAIN
    output.with_suffix('.cpp').write_text(code)
    with tempfile.TemporaryDirectory(prefix='passthrough-source-') as temp:
        src = Path(temp)/'probe.cpp';src.write_text(code)
        for mode, define in [('board_branch',0), ('sitl_branch',1)]:
            exe = Path(temp)/mode
            command = ['g++','-std=c++11','-Wall','-Wextra','-Werror',
                       '-DCONFIG_HAL_BOARD='+str(define),str(src),'-o',str(exe)]
            subprocess.run(command, check=True, capture_output=True, text=True)
            result = subprocess.run([str(exe)], check=True, capture_output=True, text=True)
            report[mode] = json.loads(result.stdout)
    report['stock_354_passthrough_meets_task_output_contract'] = False
    report['all_reproduction_assertions_passed'] = True
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', type=Path, required=True)
    p.add_argument('--output', type=Path, default=Path('reports/passthrough_source.json'))
    a = p.parse_args()
    print(json.dumps(verify(a.source,a.output), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
