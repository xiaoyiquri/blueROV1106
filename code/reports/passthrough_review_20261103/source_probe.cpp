
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
void SRV_Channel::output_ch(void)
{
    int8_t passthrough_from = -1;

    // take care of special function cases
    switch(function)
    {
    case k_manual:              // manual
        passthrough_from = ch_num;
        break;
    case k_rcin1 ... k_rcin16: // rc pass-thru
        passthrough_from = int8_t(function - k_rcin1);
        break;
    case k_motor1 ... k_motor8:
        // handled by AP_Motors::rc_write()
        return;
    }
    if (passthrough_from != -1) {
        // we are doing passthrough from input to output for this channel
        RC_Channel *rc = RC_Channels::rc_channel(passthrough_from);
        if (rc) {
            if (SRV_Channels::passthrough_disabled()) {
                output_pwm = rc->get_radio_trim();
            } else {
                output_pwm = rc->get_radio_in();
            }
        }
    }
    hal.rcout->write(ch_num, output_pwm);
}
void AP_Motors::rc_write(uint8_t chan, uint16_t pwm)
{
    if (_motor_map_mask & (1U<<chan)) {
        // we have a mapped motor number for this channel
        chan = _motor_map[chan];
    }
    if (_pwm_type == PWM_TYPE_ONESHOT125 && (_motor_fast_mask & (1U<<chan))) {
        // OneShot125 uses a PWM range from 125 to 250 usec
        pwm /= 8;
        /*
          OneShot125 ESCs can be confused by pulses below 125 or above
          250, making them fail the pulse type auto-detection. This
          happens at least with BLHeli
        */
        if (pwm < 125) {
            pwm = 125;
        } else if (pwm > 250) {
            pwm = 250;
        }
    }
    hal.rcout->write(chan, pwm);
}
void AP_Motors6DOF::output_to_motors()
{
    int8_t i;
    int16_t motor_out[AP_MOTORS_MAX_NUM_MOTORS];    // final pwm values sent to the motor

    switch (_spool_mode) {
    case SHUT_DOWN:
        // sends minimum values out to the motors
        // set motor output based on thrust requests
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                motor_out[i] = 1500;
            }
        }
        break;
    case SPIN_WHEN_ARMED:
        // sends output to motors when armed but not flying
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                motor_out[i] = 1500;
            }
        }
        break;
    case SPOOL_UP:
    case THROTTLE_UNLIMITED:
    case SPOOL_DOWN:
        // set motor output based on thrust requests
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                motor_out[i] = calc_thrust_to_pwm(_thrust_rpyt_out[i]);
            }
        }
        break;
    }

    // send output to each motor
    hal.rcout->cork();
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            rc_write(i, motor_out[i]);
        }
    }
    hal.rcout->push();
}
void Sub::set_neutral_controls()
{
    int16_t channels[11];

    for (uint8_t i = 0; i < 6; i++) {
        channels[i] = 1500;
    }

    for (uint8_t i = 6; i < 11; i++) {
        channels[i] = 0xffff;
    }

    hal.rcin->set_overrides(channels, 10);

    // Clear pitch/roll trim settings
    pitchTrim = 0;
    rollTrim  = 0;
}
void Sub::failsafe_pilot_input_check()
{
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    if (g.failsafe_pilot_input == FS_PILOT_INPUT_DISABLED) {
        failsafe.pilot_input = false;
        return;
    }

    if (AP_HAL::millis() < failsafe.last_pilot_input_ms + g.failsafe_pilot_input_timeout * 1000.0f) {
        failsafe.pilot_input = false; // We've received an update from the pilot within the timeout period
        return;
    }

    if (failsafe.pilot_input) {
        return; // only act once
    }

    failsafe.pilot_input = true;

    Log_Write_Error(ERROR_SUBSYSTEM_INPUT, ERROR_CODE_FAILSAFE_OCCURRED);
    gcs_send_text(MAV_SEVERITY_CRITICAL, "Lost manual control");

    set_neutral_controls();

    if(g.failsafe_pilot_input == FS_PILOT_INPUT_DISARM) {
        init_disarm_motors();
    }
#endif
}
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
