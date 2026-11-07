from pathlib import Path
from types import SimpleNamespace
import ctypes
import json
import subprocess
import time
from zipfile import ZipFile
import xml.etree.ElementTree as ET
import numpy as np
import pytest

from bluerov2 import parameters
from bluerov2.execution import execute_lbts_plan
from bluerov2.force_run import load_config, DEFAULT_CONFIG, save_force_plan, validate_plan
from bluerov2.io import load_result
from bluerov2.t200 import T200
from bluerov2.thruster_output import ThrusterOutput, WIRE, encode_frame, decode_status, ACQUIRE, FRAME

ROOT=Path(__file__).resolve().parents[1]

@pytest.fixture
def cfg():return load_config(DEFAULT_CONFIG)

@pytest.fixture
def mapper(cfg):return T200(cfg['mapping'],cfg['curve_file'])


def test_official_data_and_conversion(mapper):
    data=json.loads((ROOT/'bluerov2/data/t200_curves.json').read_text())
    assert data['columns'][5]=='Force (Kg f)' and data['kgf_to_n']==9.80665
    assert 'Basic ESC R3' in data['source_notes'] and 'inlet velocity of zero' in data['source_notes']
    assert len(data['tables'])==6 and len(data['tables'][0]['pwm_us'])==201
    np.testing.assert_allclose(mapper.curve(13),(mapper.curve(12)+mapper.curve(14))/2)


def test_processed_curve_matches_independently_read_vendor_workbook_points(mapper):
    source=ROOT/'bluerov2/data/T200-Public-Performance-Data-10-20V-September-2019.xlsx'
    ns={'s':'http://schemas.openxmlformats.org/spreadsheetml/2006/main'}
    with ZipFile(source) as book:
        sheets=ET.fromstring(book.read('xl/workbook.xml')).find('s:sheets',ns)
        sheet_number=next(i+1 for i,s in enumerate(sheets) if s.get('name')=='16 V')
        rows=ET.fromstring(book.read(f'xl/worksheets/sheet{sheet_number}.xml')).findall('s:sheetData/s:row',ns)
        raw={}
        for row in rows[1:]:
            cells={c.get('r')[:-len(str(row.get('r')))]:c.find('s:v',ns) for c in row.findall('s:c',ns)}
            if cells.get('A') is None or cells.get('F') is None:continue
            raw[int(float(cells['A'].text))]=float(cells['F'].text)*9.80665
    for pwm in (1100,1200,1500,1600,1900):
        assert mapper.pwm[pwm==mapper.pwm][0]==pwm
        assert mapper.curve(16)[int((pwm-1100)/4)]==pytest.approx(raw[pwm],abs=.07)


def test_neutral_asymmetry_and_deadband(mapper):
    assert mapper.thrust_to_pwm(0,16,3)==1500
    assert mapper.thrust_to_pwm(.01,16,3)>1500
    assert mapper.thrust_to_pwm(-.01,16,3)<1500
    assert abs(mapper.thrust_to_pwm(10,16,3)-1500)!=abs(mapper.thrust_to_pwm(-10,16,3)-1500)
    f=mapper.curve(16)
    for force in [-20.,-1.,1.,20.]:
        pwm=mapper.thrust_to_pwm(force,16,3)
        assert abs(np.interp(pwm,mapper.pwm,f)-force)<.2
    zeros=mapper.pwm[f==0]
    assert mapper.thrust_to_pwm(1e-12,16,3)>=max(zeros)
    assert mapper.thrust_to_pwm(-1e-12,16,3)<=min(zeros)

@pytest.mark.parametrize('voltage',[9.99,20.01,np.nan,np.inf])
def test_no_voltage_extrapolation(mapper,voltage):
    with pytest.raises(ValueError):mapper.thrust_to_pwm(1,voltage,1)


def test_no_force_extrapolation_and_separate_hardware_limit(mapper):
    with pytest.raises(ValueError):mapper.thrust_to_pwm(100,16,1)
    with pytest.raises(ValueError):mapper.map_forces([100]*6,16)
    lo,hi=mapper.limits(16)
    mapped=mapper.map_forces(hi+.01,16)
    assert np.all(mapped['hardware_saturated'])
    np.testing.assert_equal(mapped['adopted_force_n'],hi)


def test_order_direction_and_bijective_mapping(cfg):
    for i,m in enumerate(cfg['mapping']):m['output_channel']=6-i;m['model_to_curve_sign']=(-1)**i
    m=T200(cfg['mapping'],cfg['curve_file']);f=np.array([1,2,3,4,5,6.])
    mapped=m.map_forces(f,16)
    for i,entry in enumerate(m.mapping):assert mapped['pwm_us'][5-i]==m.thrust_to_pwm(f[i],16,i+1)
    cfg['mapping'][0]['output_channel']=cfg['mapping'][1]['output_channel']
    with pytest.raises(ValueError):T200(cfg['mapping'])

@pytest.mark.parametrize('pwm',[[1500]*5,[1500]*7,[0]*6,[1901]*6,[np.nan]*6,[1500.1]*6])
def test_frame_rejects_invalid_pwm(pwm):
    with pytest.raises(ValueError):encode_frame(FRAME,(1,1),pwm_us=pwm)


def test_frame_contains_all_channels_once():
    p=encode_frame(FRAME,(1,1),42,7,12345,[1501,1502,1503,1497,1498,1499])
    assert len(p)==64
    assert WIRE.unpack_from(p)[-6:]==(1501,1502,1503,1497,1498,1499)

@pytest.fixture(scope='module')
def native(tmp_path_factory):
    d=tmp_path_factory.mktemp('direct6_native')
    code='''#include "direct6_core.h"
extern "C" {
void* make_core(){return new direct6::Core();}
void destroy_core(void* p){delete (direct6::Core*)p;}
void tick(void* p,unsigned now,bool armed){((direct6::Core*)p)->tick(now,armed,true);}
void handle(void* p,unsigned char* b,unsigned now,bool armed){((direct6::Core*)p)->handle(b,64,255,191,1,1,now,armed,true);}
void status(void* p,unsigned char* b,unsigned now,bool armed){auto c=(direct6::Core*)p;c->status(b,now,armed,true,c->pwm);}
}'''
    src=d/'bridge.cpp';src.write_text(code);lib=d/'core.so'
    subprocess.run(['g++','-std=c++11','-shared','-fPIC','-Wall','-Werror','-I'+str(ROOT/'firmware/ardusub_3_5_4'),str(src),'-o',str(lib)],check=True)
    native=ctypes.CDLL(str(lib));native.make_core.restype=ctypes.c_void_p
    native.destroy_core.argtypes=[ctypes.c_void_p]
    native.tick.argtypes=[ctypes.c_void_p,ctypes.c_uint,ctypes.c_bool]
    native.handle.argtypes=[ctypes.c_void_p,ctypes.c_void_p,ctypes.c_uint,ctypes.c_bool]
    native.status.argtypes=[ctypes.c_void_p,ctypes.c_void_p,ctypes.c_uint,ctypes.c_bool]
    return native

@pytest.fixture
def transport(monkeypatch,native,cfg,mapper,tmp_path):
    clock=SimpleNamespace(now=10.)
    monkeypatch.setattr(time,'monotonic',lambda:clock.now)
    monkeypatch.setattr(time,'sleep',lambda dt:setattr(clock,'now',clock.now+dt))
    core=native.make_core()
    class Mav:
        def data64_send(self,packet_type,length,data):
            assert packet_type==0xD6 and length==34
            c.sent.append((clock.now,bytes(data)))
            b=(ctypes.c_ubyte*64)(*data);native.handle(core,b,int(clock.now*1000),c.physical_armed)
            c.pending=True
        def command_long_send(self,ts,tc,command,confirmation,arm,*rest):
            assert command==400  # no servo commands, motor tests, GUIDED targets or MANUAL_CONTROL
            c.physical_armed=bool(arm);c.heartbeat_due=clock.now+c.heartbeat_delay_s
            if c.heartbeat_delay_s==0:c.vehicle.armed=c.physical_armed
            c.pending=True
    class Controller:
        target_system=target_component=1
        confirm_timeout=.4;heartbeat_timeout=3.;reboot_detected=False;stop_requested=False
        def __init__(self):
            self.direct_status=None;self.master=SimpleNamespace(mav=Mav());self.state_messages={}
            self.vehicle=SimpleNamespace(armed=False,last_heartbeat_mono=clock.now,heartbeat_fresh=lambda now,t:True)
            self.pending=False;self.sent=[]
            self.physical_armed=False;self.heartbeat_delay_s=0.;self.heartbeat_due=clock.now
        def service_io(self):
            native.tick(core,int(clock.now*1000),self.physical_armed)
            if clock.now>=self.heartbeat_due:
                self.vehicle.armed=self.physical_armed;self.vehicle.last_heartbeat_mono=clock.now
            if self.pending:
                b=(ctypes.c_ubyte*64)();native.status(core,b,int(clock.now*1000),self.physical_armed)
                self.direct_status={**decode_status(b),'received_mono':clock.now};self.pending=False
    c=Controller()
    cfg['output'].update(firmware_verified=True,channel_mapping_verified=True,lock_file=str(tmp_path/'lock'),test_voltage_v=16.)
    records=[];out=ThrusterOutput(c,mapper,cfg['output'],lambda kind,**data:records.append((kind,data)))
    c.master.mav.data64_send(0xD6,34,list(encode_frame(0,(1,1))))
    c.service_io()
    yield out,c,clock,records
    native.destroy_core(core)


def make_plan(times=(0.,.07,.23)):
    t=np.array(times);n=len(t)-1
    return {'success':True,'nominalTrajectory':np.zeros((12,n+1)),'controlTrajectory':np.ones((6,n)),
        'feedbackGains':[np.arange(72).reshape(6,12)/100 for _ in range(n)],'time':t,'model':parameters().model,'world':None}


def test_existing_feedback_and_unequal_schedule_hold(transport):
    out,c,clock,records=transport;p=make_plan();x=np.zeros(12);x[0]=.1;x[5]=2*np.pi-.1
    steps=[]
    exp=execute_lbts_plan(p,{'read_state':lambda:{'state':x,'age':0.},
        'on_start':lambda *_:out.prepare(),'send_thruster_forces':out.send_thruster_forces,
        'stop':out.stop,'poll':out.poll,'set_interval_deadline':out.set_interval_deadline,
        'on_step':steps.append,'require_stop_confirmation':True},
        {'enable_actuation':True,'confirmation':'ENABLE_BLUEROV2_THRUST','wait_poll_period':.005})
    assert exp['status']=='completed' and exp['stop_confirmation']['confirmed']
    np.testing.assert_allclose(exp['actual_time'],p['time'],atol=.006)
    er=x.copy();er[5]=-.1
    expected=np.clip(p['controlTrajectory'][:,0]-p['feedbackGains'][0]@er,p['model'].thrust_min,p['model'].thrust_max)
    np.testing.assert_allclose(exp['commanded_thruster_force'][:,0],expected,atol=1e-14)
    assert len(steps)==2
    maps=[d for kind,d in records if kind=='force_mapping']
    assert len(maps)==3  # exactly two controller outputs, then neutral; no feedback on refresh
    frame=[WIRE.unpack_from(b) for _,b in c.sent]
    assert sum(v[2]==FRAME for v in frame)>len(steps)
    assert out.stop_result['physical_thrust_confirmed'] is False


def test_firmware_expires_without_host_finally(transport,native):
    out,c,clock,records=transport;out.prepare();out.send_thruster_pwm([1550]*6)
    clock.now+=.21;c.service_io();out.query();c.service_io()
    assert c.direct_status['state']==2 and c.direct_status['driver_readback_pwm_us']==[1500]*6
    with pytest.raises(RuntimeError):out.send_thruster_pwm([1550]*6)
    assert out.stop()['confirmed']


def test_missing_stop_ack_not_success(transport):
    out,c,clock,records=transport;out.prepare()
    c.master.mav.data64_send=lambda *args:None
    result=out.stop()
    assert not result['confirmed'] and not result['neutral_driver_readback_confirmed']


def test_voltage_is_explicit_and_battery_cells_are_summed(transport):
    out,c,clock,_=transport
    assert out.voltage()==(16.,'configured_test_voltage')
    c.state_messages['BATTERY_STATUS']=(clock.now,SimpleNamespace(id=0,voltages=[4000]*4+[65535]*6))
    assert out.voltage()==(16.,'BATTERY_STATUS[0] sum')
    out.config['test_voltage_v']=None;c.state_messages={}
    with pytest.raises(RuntimeError,match='voltage'):out.voltage()


def test_expired_interval_does_not_refresh_even_with_fresh_queries(transport):
    out,c,clock,records=transport;out.prepare()
    deadline=clock.now+.12;out.set_interval_deadline(deadline)
    out.send_thruster_pwm([1540,1460,1550,1450,1560,1440])
    while clock.now<deadline+.25:
        out.query();c.service_io()
        if c.direct_status['state']==2:break
        out.poll();clock.now+=.005
    assert c.direct_status['state']==2 and c.direct_status['driver_readback_pwm_us']==[1500]*6
    frames=[d for kind,d in records if kind=='pwm_frame' and d['op']==FRAME and d['sent_pwm_us']!=[1500]*6]
    assert frames and all(d['host_mono']<deadline for d in frames)
    with pytest.raises(RuntimeError):out.send_thruster_pwm([1540]*6)
    assert out.stop()['confirmed']
    sent=len(c.sent);result=out.stop()
    assert result is out.stop_result and len(c.sent)==sent


def test_normal_interval_transition_has_no_neutral_frame(transport):
    out,c,clock,records=transport;p=make_plan()
    p['controlTrajectory'][:,1]=-2.
    steps=[]
    exp=execute_lbts_plan(p,{'read_state':lambda:{'state':np.zeros(12),'age':0.},
        'on_start':lambda *_:out.prepare(),'send_thruster_forces':out.send_thruster_forces,
        'stop':out.stop,'poll':out.poll,'set_interval_deadline':out.set_interval_deadline,
        'on_step':steps.append,'require_stop_confirmation':True},
        {'enable_actuation':True,'confirmation':'ENABLE_BLUEROV2_THRUST','wait_poll_period':.005})
    assert exp['status']=='completed' and len(steps)==2
    first,last=(s['output']['sent_sequence'] for s in steps)
    frames=[d for kind,d in records if kind=='pwm_frame' and d['op']==FRAME and first<=d['sequence']<=last]
    assert frames and all(d['sent_pwm_us']!=[1500]*6 for d in frames)
    assert frames[-1]['sent_pwm_us']==list(steps[1]['output']['pwm_us'])


def test_live_voltage_precedence_stale_rejection_and_fixed_source_label(transport):
    out,c,clock,records=transport
    c.state_messages['SYS_STATUS']=(clock.now,SimpleNamespace(voltage_battery=15400))
    assert out.voltage()==(15.4,'SYS_STATUS')
    clock.now+=out.config['voltage_max_age_s']+.01
    assert out.voltage()==(16.,'configured_test_voltage')
    out.config['test_voltage_v']=None
    with pytest.raises(RuntimeError,match='no fresh voltage'):out.voltage()


def test_refresh_holds_pwm_without_recalculating_for_new_voltage(transport):
    out,c,clock,records=transport;out.prepare();out.set_interval_deadline(clock.now+.2)
    c.state_messages['SYS_STATUS']=(clock.now,SimpleNamespace(voltage_battery=16000))
    mapped=out.send_thruster_forces([1,-1,2,-2,1.5,-1.5]);held=out.held_pwm.copy()
    c.state_messages['SYS_STATUS']=(clock.now,SimpleNamespace(voltage_battery=12000))
    for _ in range(15):clock.now+=.005;out.poll()
    np.testing.assert_array_equal(out.held_pwm,held)
    assert mapped['voltage_source']=='SYS_STATUS' and mapped['voltage_v']==16.
    assert len([d for kind,d in records if kind=='force_mapping'])==1
    assert out.stop()['confirmed']


def test_algorithm_plan_export_preserves_arrays_and_reference(cfg,mapper,tmp_path):
    origin=tmp_path/'origin.json';origin.write_text('{}')
    p=make_plan();path=tmp_path/'plan.pkl';save_force_plan(path,p,origin)
    restored=load_result(path);validate_plan(restored,cfg,origin,mapper,16.)
    np.testing.assert_array_equal(restored['feedbackGains'],p['feedbackGains'])
    origin.write_text('{"changed":true}')
    with pytest.raises(ValueError,match='reference'):validate_plan(restored,cfg,origin,mapper)


def test_missed_interval_never_replays(monkeypatch):
    clock=SimpleNamespace(now=0.,reads=0);monkeypatch.setattr(time,'monotonic',lambda:clock.now)
    def read():
        clock.reads+=1
        if clock.reads>1:clock.now+=.08
        return {'state':np.zeros(12),'age':0.}
    sent=[]
    exp=execute_lbts_plan(make_plan(),{'read_state':read,'send_thruster_forces':lambda f:sent.append(f)},
        {'enable_actuation':True,'confirmation':'ENABLE_BLUEROV2_THRUST','max_timing_overrun':.25})
    assert exp['abort_reason']=='missed_control_interval'
    assert len(sent)==1 and np.all(sent[0]==0)


def test_stop_confirmation_required_by_new_mode():
    exp=execute_lbts_plan(make_plan(),{'read_state':lambda:{'state':np.zeros(12),'age':0.},
        'send_thruster_forces':lambda _:None,'stop':lambda:{'confirmed':False},'require_stop_confirmation':True},
        {'real_time':False,'enable_actuation':True,'confirmation':'ENABLE_BLUEROV2_THRUST'})
    assert exp['status']=='stop_unconfirmed'


def test_native_core_regressions(tmp_path):
    exe=tmp_path/'test'
    subprocess.run(['g++','-std=c++11','-Wall','-Wextra','-Werror',str(ROOT/'firmware/ardusub_3_5_4/test_direct6.cpp'),'-o',str(exe)],check=True)
    assert 'PASS' in subprocess.check_output([str(exe)],text=True)


def test_state_loss_during_hold_neutralizes_without_more_feedback(transport):
    out,c,clock,records=transport;p=make_plan((0.,.3));start=clock.now;step=[]
    def poll():
        if clock.now-start>.06:raise RuntimeError('state expired during held interval')
        out.poll()
    with pytest.raises(RuntimeError,match='state expired'):
        execute_lbts_plan(p,{'read_state':lambda:{'state':np.zeros(12),'age':0.},
            'on_start':lambda *_:out.prepare(),'send_thruster_forces':out.send_thruster_forces,
            'stop':out.stop,'poll':poll,'on_step':step.append},
            {'enable_actuation':True,'confirmation':'ENABLE_BLUEROV2_THRUST','wait_poll_period':.005})
    assert len(step)==1 and out.stop_result['confirmed']
    assert c.direct_status['driver_readback_pwm_us']==[1500]*6

@pytest.mark.parametrize('shape',[(8,12),(6,11),(12,6)])
def test_feedback_gain_dimension_rejected(shape):
    p=make_plan((0.,.1));p['feedbackGains']=[np.zeros(shape)]
    with pytest.raises(ValueError,match='6x12'):execute_lbts_plan(p,{'read_state':lambda:np.zeros(12)})


def test_actual_pymavlink_wire_roundtrip():
    from pymavlink import mavutil
    class Stream:
        def __init__(self):self.data=b''
        def write(self,data):self.data+=data
    stream=Stream();m=mavutil.mavlink.MAVLink(stream,srcSystem=255,srcComponent=191)
    frame=encode_frame(FRAME,(1,1),42,9,1000,[1511,1512,1513,1514,1515,1516])
    m.data64_send(0xD6,WIRE.size,list(frame))
    receiver=mavutil.mavlink.MAVLink(None);parsed=None
    for byte in stream.data:
        message=receiver.parse_char(bytes([byte]))
        if message is not None:parsed=message
    assert parsed.get_type()=='DATA64' and parsed.len==34 and parsed.type==0xD6
    assert WIRE.unpack_from(bytes(parsed.data))[-6:]==(1511,1512,1513,1514,1515,1516)


def test_operator_abort_before_start_never_acquires(transport):
    out,c,clock,records=transport;c.stop_requested=True
    with pytest.raises(RuntimeError,match='operator_abort'):out.prepare()
    assert not out.acquire_attempted and not out.arm_attempted


def test_operator_abort_still_confirms_stop(transport):
    out,c,clock,records=transport;out.prepare();out.send_thruster_pwm([1520]*6)
    c.stop_requested=True
    assert out.stop()['confirmed']


def test_prepare_waits_for_delayed_armed_heartbeat_and_holds_neutral(transport):
    out,c,clock,_=transport;c.heartbeat_delay_s=.15;start=clock.now
    out.prepare()
    assert clock.now>=start+.15 and c.vehicle.armed is True
    before_confirmation=[WIRE.unpack_from(frame)[-6:] for sent,frame in c.sent if sent<start+.15]
    assert before_confirmation and all(pwm==(1500,)*6 for pwm in before_confirmation)
    out.send_thruster_pwm([1540]*6)
    assert out.stop()['confirmed']
