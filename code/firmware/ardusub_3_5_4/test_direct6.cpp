#include "direct6_core.h"
#include <assert.h>
#include <stdio.h>
using namespace direct6;
static uint8_t b[64];
static void command(uint8_t op,uint32_t seq,uint32_t deadline,uint16_t p=1500,uint32_t session=42) {
 memset(b,0,sizeof(b));memcpy(b,"BT6!",4);b[4]=1;b[5]=op;b[6]=1;b[7]=1;
 put32(b+8,session);put32(b+12,seq);put32(b+16,deadline);
 for(int i=0;i<6;i++)put16(b+22+2*i,p+i*(p!=1500));
}
static void send(Core& c,uint32_t now,bool armed=true,bool allowed=true,uint8_t sys=255) {
 assert(c.handle(b,64,sys,191,1,1,now,armed,allowed));
}
int main() {
 Core c;
 command(ACQUIRE,1,250);send(c,100,true);assert(c.state==IDLE); // no armed takeover
 send(c,100,false);assert(c.state==HELD);
 command(FRAME,2,260,1510);send(c,110);assert(c.pwm[5]==1515);
 c.tick(259,true,true);assert(c.pwm[0]==1510);
 c.tick(260,true,true);assert(c.state==FAULT && c.pwm[5]==1500);
 command(FRAME,3,400,1550);send(c,261);assert(c.pwm[0]==1500); // timeout latched
 command(RELEASE,4,400);send(c,262,false);assert(c.state==IDLE);
 command(ACQUIRE,1,500);send(c,300,false);
 command(FRAME,2,500,1600);send(c,310,true,true,254);assert(c.pwm[0]==1500); // other owner
 send(c,310);assert(c.pwm[0]==1600);
 command(FRAME,3,510,1610);put16(b+32,1901);send(c,320);assert(c.state==FAULT && c.pwm[0]==1500); // no partial frame
 command(RELEASE,4,600);send(c,330,false);
 command(ACQUIRE,1,600);send(c,400,false);
 command(FRAME,2,610,1520);send(c,410);send(c,411);assert(c.state==FAULT); // replay
 command(RELEASE,3,700);send(c,412,false);
 command(ACQUIRE,1,700);send(c,500,false);
 command(FRAME,2,710,1520);send(c,510);c.tick(511,false,true);assert(c.state==FAULT); // disarm
 command(RELEASE,3,800);send(c,520,false);
 command(ACQUIRE,1,800);send(c,600,false);c.tick(601,false,false);assert(c.state==FAULT); // mode/config
 command(RELEASE,3,900);send(c,610,false);
 command(ACQUIRE,1,900);send(c,700,false);
 command(FRAME,2,850,1540);send(c,860);assert(c.state==FAULT); // delayed frame
 uint16_t rb[6]={1500,1501,1502,1503,1504,1505};c.status(b,861,false,true,rb);
 assert(!memcmp(b,"BS6!",4) && get16(b+32)==1505);
 // Clock rollover preserves signed deadline comparison.
 Core wrap;command(ACQUIRE,1,40);send(wrap,0xfffffff0,false);assert(wrap.state==HELD);
 wrap.tick(39,false,true);assert(wrap.state==HELD);wrap.tick(40,false,true);assert(wrap.state==FAULT);
 puts("direct6 core: ownership, atomic frame, expiry, replay, disarm, mode, rollover PASS");
}
