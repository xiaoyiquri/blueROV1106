// SPDX-License-Identifier: GPL-3.0-or-later
// Shared by the ArduSub patch and host-native tests. No HAL, heap, or motor mixer.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

namespace direct6 {
static const uint16_t NEUTRAL=1500, MIN_PWM=1100, MAX_PWM=1900;
static const uint32_t MAX_LEASE_MS=200;
enum Op { QUERY=0, ACQUIRE=1, FRAME=2, STOP=3, RELEASE=4 };
enum State { IDLE=0, HELD=1, FAULT=2 };
static inline uint16_t get16(const uint8_t* b) { return uint16_t(b[0]) | uint16_t(b[1])<<8; }
static inline uint32_t get32(const uint8_t* b) { return uint32_t(get16(b)) | uint32_t(get16(b+2))<<16; }
static inline void put16(uint8_t* b,uint16_t n) { b[0]=n; b[1]=n>>8; }
static inline void put32(uint8_t* b,uint32_t n) { put16(b,n); put16(b+2,n>>16); }

class Core {
public:
    State state=IDLE;
    uint8_t owner_system=0,owner_component=0;
    uint32_t session=0,sequence=0,deadline=0;
    uint16_t pwm[6]={1500,1500,1500,1500,1500,1500};
    bool ran_armed=false;
    void neutral() { for (uint8_t i=0;i<6;i++) pwm[i]=NEUTRAL; }
    void fault() { state=FAULT; neutral(); }
    void tick(uint32_t now,bool armed,bool allowed) {
        if(state==IDLE) return;
        if(!allowed || int32_t(deadline-now)<=0 || (ran_armed && !armed)) fault();
        if(!armed) neutral();
        if(armed && state==HELD) ran_armed=true;
    }
    bool handle(const uint8_t* b,size_t len,uint8_t sys,uint8_t comp,
                uint8_t target_sys,uint8_t target_comp,uint32_t now,bool armed,bool allowed) {
        if(len<34 || memcmp(b,"BT6!",4) || b[4]!=1 || b[6]!=target_sys || b[7]!=target_comp) return false;
        tick(now,armed,allowed);
        const uint8_t op=b[5];
        if(op==QUERY) return true;
        const uint32_t id=get32(b+8),seq=get32(b+12),until=get32(b+16);
        if(get16(b+20)!=0) return true;
        const bool fresh=int32_t(until-now)>0 && int32_t(until-now)<=int32_t(MAX_LEASE_MS);
        if(op==ACQUIRE) {
            if(state!=IDLE || armed || !allowed || !id || !fresh || seq!=1) return true;
            state=HELD;owner_system=sys;owner_component=comp;session=id;sequence=seq;deadline=until;
            ran_armed=false;neutral();return true;
        }
        if(state==IDLE || sys!=owner_system || comp!=owner_component || id!=session) return true;
        // STOP always neutralizes this owner's session, even if the link delivered it late.
        if(op==STOP) { fault();return true; }
        if(op==RELEASE) {
            if(!armed) { neutral();state=IDLE;session=0;owner_system=0;owner_component=0;ran_armed=false; }
            return true;
        }
        if(op!=FRAME || state!=HELD) return true;
        if(!fresh || seq<=sequence) { fault();return true; }
        uint16_t next[6];
        for(uint8_t i=0;i<6;i++) {
            next[i]=get16(b+22+2*i);
            if(next[i]<MIN_PWM || next[i]>MAX_PWM || (!armed && next[i]!=NEUTRAL)) { fault();return true; }
        }
        // Validate every channel before committing any channel.
        memcpy(pwm,next,sizeof(pwm));sequence=seq;deadline=until;return true;
    }
    void status(uint8_t* b,uint32_t now,bool armed,bool allowed,const uint16_t readback[6]) const {
        memset(b,0,64);memcpy(b,"BS6!",4);b[4]=1;
        b[5]=uint8_t(state)|(armed?4:0)|(allowed?8:0);b[6]=owner_system;b[7]=owner_component;
        put32(b+8,session);put32(b+12,sequence);put32(b+16,now);
        put16(b+20,state==HELD && int32_t(deadline-now)>0 ? uint16_t(deadline-now):0);
        for(uint8_t i=0;i<6;i++) put16(b+22+2*i,readback[i]);
    }
};
}
