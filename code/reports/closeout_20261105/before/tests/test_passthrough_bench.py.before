"""Controlled-clock bench transactions. No hardware endpoint is opened here."""
from collections import deque
import copy
import json
from pathlib import Path
import socket
import time
from types import SimpleNamespace

import pytest
from pymavlink import mavutil

from bluerov2.passthrough_bench import (BenchAbort,BenchSession,FUNCTIONS,NEUTRAL,
    PARAMETERS,PATTERN,frame,require_free_udp_endpoint,readonly_controller)


@pytest.fixture
def bench(monkeypatch,tmp_path):
    clock=SimpleNamespace(now=10.)
    monkeypatch.setattr(time,'monotonic',lambda:clock.now)
    monkeypatch.setattr(time,'sleep',lambda dt:setattr(clock,'now',clock.now+dt))
    params={'SYSID_MYGCS':255.,'FRAME_CONFIG':1.,'MOT_PWM_TYPE':0.,'FS_PILOT_INPUT':2.,'FS_PILOT_TIMEOUT':.2}
    for i in range(1,7):
        params.update({f'SERVO{i}_FUNCTION':32.+i,f'SERVO{i}_MIN':1100.,f'SERVO{i}_MAX':1900.,
                       f'SERVO{i}_TRIM':1500.,f'SERVO{i}_REVERSED':0.})
    firmware={'flight_sw_version':(3<<24)|(5<<16)|(4<<8)|255,
              'flight_custom_version':list(b'05fed49b'),'uid':0}
    pending=deque();wire=[]
    c=SimpleNamespace(target_system=1,target_component=1,source_system=255,serial=0,
                      heartbeat_timeout=.5,stop_requested=False,bench_fault=None,messages={},
                      param_records={},records=[],rc=NEUTRAL,last_rc=None,
                      fail_set_once=None,drop_all_set_replies=False,apply_failsafe=True,
                      abort_on_pattern=False,heartbeat_enabled=True,params=params,emit_failsafe_event=True)
    c.vehicle=SimpleNamespace(armed=False,custom_mode=19,mode='MANUAL',autopilot_version=firmware,
                              last_heartbeat_mono=clock.now,heartbeat_fresh=lambda now,age:now-c.vehicle.last_heartbeat_mono<=age)
    parser=mavutil.mavlink.MAVLink(None)
    class Sink:
        def write(self,payload):
            messages=parser.parse_buffer(payload) or []
            for msg in messages:
                data=msg.to_dict();kind=msg.get_type();wire.append((clock.now,kind,data))
                if kind=='PARAM_REQUEST_READ':
                    name=data['param_id'].rstrip('\0')
                    pending.append((name,params[name]))
                elif kind=='PARAM_SET':
                    name=data['param_id'].rstrip('\0');params[name]=data['param_value']
                    if c.drop_all_set_replies:continue
                    if c.fail_set_once==name:c.fail_set_once=None;continue
                    pending.append((name,params[name]))
                elif kind=='RC_CHANNELS_OVERRIDE':
                    c.rc=tuple(data[f'chan{i}_raw'] for i in range(1,7));c.last_rc=clock.now
                    if c.abort_on_pattern and c.rc==PATTERN:c.stop_requested=True
            return len(payload)
    c.master=SimpleNamespace(mav=mavutil.mavlink.MAVLink(Sink(),srcSystem=255,srcComponent=191))
    def service():
        while pending:
            name,value=pending.popleft();c.serial+=1;c.param_records[name]=(c.serial,value)
        if c.heartbeat_enabled:c.vehicle.last_heartbeat_mono=clock.now
        if c.apply_failsafe and c.last_rc is not None and clock.now-c.last_rc>=params['FS_PILOT_TIMEOUT']:
            if c.rc!=NEUTRAL and c.emit_failsafe_event:
                c.records.append((clock.now,'STATUSTEXT',{'text':'Lost manual control'}))
            c.rc=NEUTRAL
        # Mock the sampled readback only, not electrical pulse timing.
        driver=tuple(c.rc[i] if params[FUNCTIONS[i]]==1 else 1500 for i in range(6))
        for kind,prefix,values in [('RC_CHANNELS','chan',c.rc),('SERVO_OUTPUT_RAW','servo',driver)]:
            raw={f'{prefix}{i}_raw':v for i,v in enumerate(values,1)}
            c.messages[kind]=(clock.now,raw);c.records.append((clock.now,kind,raw))
    c.service_io=service
    session=BenchSession(c,tmp_path);snapshot=session.snapshot()
    return session,c,clock,wire,snapshot


def output_messages(wire):
    return [(t,k,d) for t,k,d in wire if k in ('PARAM_SET','RC_CHANNELS_OVERRIDE')]


def test_readonly_snapshot_has_no_actuator_or_parameter_writes(bench):
    session,c,clock,wire,snapshot=bench
    assert set(snapshot['parameters'])==set(PARAMETERS)
    assert not output_messages(wire)
    assert set(k for _,k,_ in wire)=={'COMMAND_LONG','PARAM_REQUEST_READ'}
    assert all(d['command']==520 for _,k,d in wire if k=='COMMAND_LONG')


def test_power_isolation_is_not_inferred_from_user_continuing(bench):
    session,c,clock,wire,snapshot=bench
    with pytest.raises(BenchAbort,match='isolated'):
        session.execute(snapshot)
    assert not output_messages(wire)
    assert not (session.directory/'original_parameters.json').exists()


@pytest.mark.parametrize('name',['ARM','MODE','MANUAL_CONTROL','GUIDED','HEARTBEAT'])
def test_outgoing_gate_rejects_other_control_messages(bench,name):
    session,c,clock,wire,snapshot=bench
    msg=SimpleNamespace(get_type=lambda:name,to_dict=lambda:{'target_system':1,'target_component':1})
    before=len(wire)
    with pytest.raises(BenchAbort):c.master.mav.send(msg)
    assert len(wire)==before


@pytest.mark.parametrize('command',[400,176,183,209])
def test_no_arm_mode_servo_or_motor_test_command(bench,command):
    session,c,clock,wire,snapshot=bench
    with pytest.raises(BenchAbort):
        c.master.mav.command_long_send(1,1,command,0,1,0,0,0,0,0,0)
    assert not output_messages(wire)


@pytest.mark.parametrize('fault',['armed','mode','frame','failsafe','clock','stale','reassigned','nonneutral','nonfinite','wrong_commit'])
def test_preflight_failures_make_no_parameter_or_output_changes(bench,fault):
    session,c,clock,wire,snapshot=bench
    if fault=='armed':c.vehicle.armed=True
    elif fault=='mode':c.vehicle.custom_mode=2
    elif fault=='frame':snapshot['parameters']['FRAME_CONFIG']=2
    elif fault=='failsafe':snapshot['parameters']['FS_PILOT_INPUT']=0
    elif fault=='clock':c.bench_fault='device clock rollback'
    elif fault=='stale':c.heartbeat_enabled=False;clock.now+=1.
    elif fault=='reassigned':snapshot['parameters']['SERVO1_FUNCTION']=1
    elif fault=='nonneutral':c.rc=(1600,)*6
    elif fault=='nonfinite':snapshot['parameters']['SERVO1_MIN']=float('nan')
    elif fault=='wrong_commit':snapshot['firmware']['flight_custom_version']=list(b'unknown!')
    with pytest.raises((BenchAbort,ValueError)):
        session.execute(snapshot,power_isolated=True)
    assert not output_messages(wire)


def test_backup_is_durable_before_first_parameter_write(bench,monkeypatch):
    session,c,clock,wire,snapshot=bench
    monkeypatch.setattr('bluerov2.passthrough_bench.os.fsync',lambda *_:(_ for _ in ()).throw(OSError('disk full')))
    with pytest.raises(OSError):session.execute(snapshot,power_isolated=True)
    assert not output_messages(wire)


def test_fixed_frame_watchdog_observation_and_exact_restore(bench):
    session,c,clock,wire,snapshot=bench
    original={name:snapshot['parameters'][name] for name in FUNCTIONS}
    result=session.execute(snapshot,power_isolated=True)
    assert result['status']=='bench_observations_complete'
    assert result['stop']['telemetry_cleanup_confirmed']
    assert result['stop']['electrical_stop_confirmed'] is False
    assert result['independent_output_validated'] is False
    assert result['electrical_measurement_supplied'] is False
    assert result['pilot_input_neutral_after_silence_observed']
    assert {name:c.params[name] for name in FUNCTIONS}==original
    saved=json.loads((session.directory/'original_parameters.json').read_text())
    assert saved['parameters']==original
    assert any(k=='RC_CHANNELS_OVERRIDE' and tuple(d[f'chan{i}_raw'] for i in range(1,7))==PATTERN for _,k,d in wire)
    assert all(k in ('PARAM_SET','PARAM_REQUEST_READ','COMMAND_LONG','RC_CHANNELS_OVERRIDE') for _,k,_ in wire)
    assert all(d['chan7_raw']==65535 and d['chan8_raw']==65535 for _,k,d in wire if k=='RC_CHANNELS_OVERRIDE')


def test_partial_apply_with_lost_echo_restores_attempted_parameter(bench):
    session,c,clock,wire,snapshot=bench;c.fail_set_once='SERVO3_FUNCTION'
    result=session.execute(snapshot,power_isolated=True)
    assert result['status']=='bench_aborted' and result['stop']['telemetry_cleanup_confirmed']
    assert [c.params[name] for name in FUNCTIONS]==list(range(33,39))
    assert session.changes_attempted==list(FUNCTIONS[:3])


def test_operator_interrupt_stops_and_restores_without_auto_resume(bench):
    session,c,clock,wire,snapshot=bench;c.abort_on_pattern=True
    result=session.execute(snapshot,power_isolated=True)
    assert result['status']=='bench_aborted'
    assert 'operator_abort' in result['reason']
    assert result['stop']['telemetry_cleanup_confirmed']
    assert session.allow_outputs is False
    writes_before=len(output_messages(wire));c.stop_requested=False;clock.now+=1.
    session.poll()
    assert len(output_messages(wire))==writes_before


def test_sitl_style_no_watchdog_does_not_pass_acceptance(bench):
    session,c,clock,wire,snapshot=bench;c.apply_failsafe=False
    result=session.execute(snapshot,power_isolated=True)
    assert result['status']=='watchdog_neutral_not_observed'
    assert not result['pilot_input_neutral_after_silence_observed']
    assert result['stop']['telemetry_cleanup_confirmed']


def test_neutral_telemetry_without_failsafe_event_is_unconfirmed(bench):
    session,c,clock,wire,snapshot=bench;c.emit_failsafe_event=False
    result=session.execute(snapshot,power_isolated=True)
    assert result['pilot_input_neutral_after_silence_observed']
    assert result['pilot_failsafe_event_observed'] is False
    assert result['status']=='failsafe_event_unconfirmed'
    assert result['stop']['telemetry_cleanup_confirmed']


def test_missing_restore_echo_never_reports_cleanup_confirmed(bench):
    session,c,clock,wire,snapshot=bench
    session.check_preflight(snapshot);session.original={name:snapshot['parameters'][name] for name in FUNCTIONS}
    session.allow_outputs=True;session.set_function('SERVO1_FUNCTION',1.)
    c.drop_all_set_replies=True
    result=session.cleanup()
    assert result['telemetry_cleanup_confirmed'] is False
    assert result['parameters_restored'] is False
    assert 'restore_error' in result


def test_recovery_uses_backup_and_rejects_wrong_target(bench,tmp_path):
    session,c,clock,wire,snapshot=bench
    original={name:snapshot['parameters'][name] for name in FUNCTIONS}
    data={'target':[2,1],'firmware':snapshot['firmware'],'parameters':original}
    backup=tmp_path/'backup.json';backup.write_text(json.dumps(data))
    with pytest.raises(BenchAbort,match='target mismatch'):
        session.restore_backup(snapshot,backup,power_isolated=True)
    assert not output_messages(wire)
    data['target']=[1,1];backup.write_text(json.dumps(data))
    c.params['SERVO1_FUNCTION']=1.;snapshot['parameters']['SERVO1_FUNCTION']=1.
    result=session.restore_backup(snapshot,backup,power_isolated=True)
    assert result['status']=='backup_restored'
    assert c.params['SERVO1_FUNCTION']==33.
    assert not any(k=='RC_CHANNELS_OVERRIDE' and tuple(d[f'chan{i}_raw'] for i in range(1,7))!=NEUTRAL for _,k,d in wire)


def test_existing_udp_listener_is_not_shared_or_taken_over():
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as existing:
        existing.bind(('127.0.0.1',0));port=existing.getsockname()[1]
        with pytest.raises(OSError):require_free_udp_endpoint(f'udpin:127.0.0.1:{port}')


def test_refresh_overrun_aborts_without_catchup_burst(bench):
    session,c,clock,wire,snapshot=bench;original=c.service_io;delayed=[]
    def service():
        if session.phase=='pattern' and not delayed:
            clock.now+=.2;delayed.append(True)
        original()
    c.service_io=service
    result=session.execute(snapshot,power_isolated=True)
    assert result['status']=='bench_aborted' and 'timing overrun' in result['reason']
    assert result['stop']['telemetry_cleanup_confirmed']
    assert not any(k=='RC_CHANNELS_OVERRIDE' and tuple(d[f'chan{i}_raw'] for i in range(1,7))==PATTERN for _,k,d in wire)


def test_absent_neutral_driver_echo_prevents_cleanup_success(bench):
    session,c,clock,wire,snapshot=bench;original=c.service_io
    def service():
        original()
        if session.phase=='cleanup':
            c.messages['SERVO_OUTPUT_RAW']=(clock.now,{f'servo{i}_raw':v for i,v in enumerate(PATTERN,1)})
    c.service_io=service
    result=session.execute(snapshot,power_isolated=True)
    assert result['status']=='cleanup_unconfirmed'
    assert result['stop']['telemetry_cleanup_confirmed'] is False
    assert result['stop']['neutral_driver_samples_confirmed'] is False
    assert result['stop']['parameters_restored'] is True


def test_existing_backup_is_never_overwritten(bench):
    session,c,clock,wire,snapshot=bench
    path=session.directory/'original_parameters.json';path.write_text('original evidence')
    with pytest.raises(FileExistsError):session.execute(snapshot,power_isolated=True)
    assert path.read_text()=='original evidence' and not output_messages(wire)


@pytest.mark.parametrize('rollback',[True,False])
def test_device_clock_rollback_latches_and_uint32_wrap_does_not(rollback):
    class Base:
        def __init__(self):self.target_system=self.target_component=1
        def process_message(self,msg):pass
        def _send_heartbeat(self):raise AssertionError('must not send heartbeat')
    c=readonly_controller(SimpleNamespace(Controller=Base))()
    def message(ms):
        return SimpleNamespace(get_srcSystem=lambda:1,get_srcComponent=lambda:1,
            get_type=lambda:'ATTITUDE',to_dict=lambda:{'time_boot_ms':ms})
    c._send_heartbeat()
    c.process_message(message(1000 if rollback else 2**32-3))
    c.process_message(message(1))
    assert bool(c.bench_fault)==rollback
    c.process_message(message(2000))
    assert bool(c.bench_fault)==rollback


def test_default_cli_without_dvl_origin_or_curve_sends_no_output(bench,monkeypatch,tmp_path):
    import bluerov2.passthrough_bench as entry
    session,c,clock,wire,snapshot=bench
    c.vehicle.vehicle_type=12;c.vehicle.autopilot=3
    c.connect=lambda:None;c.close=lambda:None
    logger=SimpleNamespace(write_log=lambda *_a,**_kw:None,close=lambda:None)
    monkeypatch.setattr(entry,'readonly_controller',lambda _backend:lambda *_a,**_kw:c)
    monkeypatch.setattr(entry,'load_communication_backend',lambda _:SimpleNamespace(SessionLogger=lambda *_a,**_kw:logger))
    monkeypatch.setattr(entry,'OUTPUT_LOCK',tmp_path/'test-actuator.lock')
    monkeypatch.setattr(entry.fcntl,'flock',lambda *_:(_ for _ in ()).throw(AssertionError('readonly must not acquire actuator lock')))
    config=tmp_path/'transport.json';config.write_text(json.dumps({'connection':'mock:never-connect','source_system':255}))
    directory=tmp_path/'readonly'
    rc=entry.main(['--connection-config',str(config),'--output-directory',str(directory)])
    report=json.loads((directory/'report.json').read_text())
    assert rc==0 and report['status']=='read_only_completed'
    assert report['parameter_writes']==[] and report['override_frame_count']==0
    assert not output_messages(wire)


def test_power_isolated_branch_still_requires_exclusive_output_lock(bench,monkeypatch,tmp_path):
    import bluerov2.passthrough_bench as entry
    session,c,clock,wire,snapshot=bench
    lock_path=tmp_path/'actuator.lock'
    monkeypatch.setattr(entry,'OUTPUT_LOCK',lock_path)
    config=tmp_path/'connection.json';config.write_text(json.dumps({'connection':'mock:never-connect'}))
    with lock_path.open('a') as owner:
        entry.fcntl.flock(owner,entry.fcntl.LOCK_EX|entry.fcntl.LOCK_NB)
        directory=tmp_path/'blocked-output'
        rc=entry.main(['--connection-config',str(config),'--output-directory',str(directory),'--execute-power-isolated-bench'])
    report=json.loads((directory/'report.json').read_text())
    assert rc==1 and report['status']=='rejected'
    assert 'BlockingIOError' in report['reason']
    assert not output_messages(wire)


def test_cleanup_gate_rejects_nonneutral_frame(bench):
    session,c,clock,wire,snapshot=bench
    session.allow_outputs=True;session.original={n:snapshot['parameters'][n] for n in FUNCTIONS};session.phase='cleanup'
    with pytest.raises(BenchAbort,match='nonneutral'):
        session.send_pwm(PATTERN)
    assert not output_messages(wire)


def test_arming_during_restore_blocks_further_motor_function_writes(bench):
    session,c,clock,wire,snapshot=bench;original=c.service_io
    def service():
        original()
        if session.phase=='cleanup' and any(k=='PARAM_SET' and d['param_value']>1 for _,k,d in wire):
            c.vehicle.armed=True
    c.service_io=service
    result=session.execute(snapshot,power_isolated=True)
    assert result['status']=='cleanup_unconfirmed'
    assert result['stop']['disarmed_heartbeat_confirmed'] is False
    assert result['stop']['telemetry_cleanup_confirmed'] is False
    restored=[d['param_id'] for _,k,d in wire if k=='PARAM_SET' and d['param_value']>1]
    assert restored==['SERVO1_FUNCTION']


@pytest.mark.parametrize('value',[[1500]*5,[1500]*7,[0]*6,[1901]*6,[float('nan')]*6,[1500.2]*6])
def test_bad_frame(value):
    with pytest.raises(ValueError):frame(value)
