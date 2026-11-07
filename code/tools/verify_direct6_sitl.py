"""Exercise the compiled patched ArduSub in an owned, isolated localhost SITL.
No live endpoint argument is accepted. No firmware is uploaded.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import secrets
import signal
import socket
import subprocess
import sys
import tempfile
import time
import traceback
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bluerov2.thruster_output import encode_frame, decode_status, QUERY,ACQUIRE,FRAME,STOP,RELEASE
from bluerov2.thruster_output import ThrusterOutput,controller_class
from bluerov2.force_run import DEFAULT_CONFIG,load_config,jsonable
from bluerov2.host_tracking import load_backend
from bluerov2.t200 import T200
from bluerov2.execution import execute_lbts_plan
from bluerov2.model import parameters
from pymavlink import mavutil

NEUTRAL=[1500]*6
PORT=15760
ROOT=Path(__file__).resolve().parents[1]


class Link:
    def __init__(self, master):
        self.master=master;self.status=None;self.heartbeat=None;self.trace=[]
        self.session=secrets.randbelow(2**32-1)+1;self.sequence=0
        self.held=NEUTRAL;self.maintain=False;self.next_frame=0.;self.next_hb=0.

    def send(self,op,pwm=NEUTRAL):
        # Match the production transport: never extrapolate the FC clock using host time.
        fc_ms=0 if self.status is None else self.status['fc_ms']
        self.sequence+=1
        payload=encode_frame(op,(1,1),self.session,self.sequence,fc_ms+200,pwm)
        self.master.mav.data64_send(0xD6,34,list(payload))

    def poll(self):
        while True:
            msg=self.master.recv_match(blocking=False)
            if msg is None:break
            if msg.get_srcSystem()!=1 or msg.get_srcComponent()!=1:continue
            if msg.get_type()=='DATA64' and msg.type==0xD6:
                s=decode_status(msg.data)
                if s:self.status={**s,'received_mono':time.monotonic()};self.trace.append(self.status.copy())
            if msg.get_type()=='HEARTBEAT':self.heartbeat=msg
        now=time.monotonic()
        if now>=self.next_hb:
            self.master.mav.heartbeat_send(6,8,0,0,4);self.next_hb=now+1.
        if self.maintain and now>=self.next_frame:
            self.send(FRAME,self.held);self.next_frame=now+.04

    def wait(self,predicate,timeout=3.):
        until=time.monotonic()+timeout
        while time.monotonic()<until:
            self.poll()
            if predicate():return
            time.sleep(.002)
        raise AssertionError('SITL condition timed out')

    def spin(self,duration):
        end=time.monotonic()+duration
        while time.monotonic()<end:self.poll();time.sleep(.002)

    def query(self):
        old=self.status
        self.send(QUERY)
        self.wait(lambda:self.status is not old)
        return self.status

    def arm(self,armed):
        self.master.mav.command_long_send(1,1,400,0,int(armed),0,0,0,0,0,0)
        self.wait(lambda:self.heartbeat is not None and bool(self.heartbeat.base_mode&128)==armed)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--binary',type=Path,required=True)
    p.add_argument('--output',type=Path,default=Path('reports/direct6_sitl.json'))
    p.add_argument('--base-port',type=int,default=PORT)
    p.add_argument('--instance',type=int,default=41)
    a=p.parse_args();binary=a.binary.resolve()
    port=a.base_port
    if not 1024<=port<=65000 or not 1<=a.instance<=1000:
        p.error('use a nonprivileged local base port and an isolated instance 1..1000')
    report={'created_utc':datetime.now(timezone.utc).isoformat(),'binary':str(binary),
            'binary_sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),
            'localhost_only':True,'physical_vehicle_used':False,'checks':{},'all_checks_passed':False}
    report['host_source_sha256']={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
        for p in [ROOT/'bluerov2'/name for name in
                  ('__init__.py','communication.py','force_run.py','state_adapter.py','thruster_output.py','execution.py','t200.py')]}
    report['test_script_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    report['firmware_patch_sha256']=hashlib.sha256((ROOT/'firmware/ardusub_3_5_4/direct6.patch').read_bytes()).hexdigest()
    checks=report['checks'];link=None;master=None;proc=None;logger=None
    a.output.parent.mkdir(parents=True,exist_ok=True)
    log=a.output.with_suffix('.console.log')
    try:
        # Fail before connecting if this instance is already occupied; never share
        # a running simulator or terminate someone else's process.
        for kind,address in [(socket.SOCK_STREAM,('0.0.0.0',port)),
                             (socket.SOCK_DGRAM,('0.0.0.0',5501+10*a.instance))]:
            with socket.socket(socket.AF_INET,kind) as probe:probe.bind(address)
        with tempfile.TemporaryDirectory(prefix='direct6-sitl-') as work,log.open('w') as console:
            report.update(work_directory=work,instance=a.instance,endpoint=f'tcp:127.0.0.1:{port}',
                          rc_input_port=5501+10*a.instance)
            defaults=Path(work)/'defaults.parm'
            defaults.write_text('FRAME_CONFIG 1\nARMING_CHECK 0\nSYSID_MYGCS 255\nFS_GCS_ENABLE 0\nFS_PILOT_INPUT 0\n')
            cmd=[str(binary),'-w','--model','vectored','--home=-35.363261,149.165230,0,0',
                 '--base-port',str(port),'--instance',str(a.instance),'--defaults',str(defaults)]
            report['command']=cmd
            proc=subprocess.Popen(cmd,cwd=work,stdout=console,stderr=subprocess.STDOUT)
            # Connect only to the process created above, never to the real robot UDP link.
            end=time.monotonic()+12
            while True:
                if proc.poll() is not None:raise RuntimeError('SITL exited before accepting TCP')
                try:
                    master=mavutil.mavlink_connection(f'tcp:127.0.0.1:{port}',source_system=255,source_component=191)
                    break
                except OSError:
                    if time.monotonic()>end:raise
                    time.sleep(.1)
            link=Link(master);link.wait(lambda:link.heartbeat is not None,10.)
            master.mav.set_mode_send(1,1,19)
            link.spin(1.2)
            status=link.query()
            assert status['state']==0 and status['ready'] and not status['armed']
            checks['query_idle_disarmed_ready']=True
            link.sequence=0;link.send(ACQUIRE)
            link.wait(lambda:link.status['state']==1 and link.status['session']==link.session)
            link.maintain=True;link.arm(True)
            checks['acquire_and_arm_neutral']=True
            pwm=[1540,1460,1550,1450,1560,1440]
            link.held=pwm;link.spin(.15)
            assert link.status['state']==1 and link.status['driver_readback_pwm_us']==pwm
            checks['complete_six_outputs_reach_hal']=True
            # Real legacy MAVLink commands are sent to SITL to test firmware isolation.
            master.mav.manual_control_send(1,1000,1000,1000,1000,0)
            master.mav.set_mode_send(1,1,4)
            master.mav.command_long_send(1,1,183,0,1,1900,0,0,0,0,0)
            master.mav.command_long_send(1,1,209,0,1,0,50,1,0,0,0)
            master.mav.set_position_target_local_ned_send(0,1,1,1,0x0ff8,1,2,3,0,0,0,0,0,0,0,0)
            link.spin(.3)
            assert link.status['state']==1 and link.status['driver_readback_pwm_us']==pwm
            assert link.heartbeat.custom_mode==19
            checks['legacy_manual_guided_servo_motor_test_do_not_overwrite']=True
            original=master.mav.srcSystem
            master.mav.srcSystem=254;link.send(ACQUIRE);master.mav.srcSystem=original
            link.spin(.1)
            assert link.status['owner']==(255,191) and link.status['state']==1
            checks['second_owner_refused']=True
            # Stop only control FRAME transmission. No Python stop/finally is invoked here.
            link.maintain=False;link.poll()
            last=link.status.copy();until=time.monotonic()+.4
            observed=None
            while time.monotonic()<until:
                # Keep ordinary GCS traffic alive while deliberately withholding FRAME.
                master.mav.heartbeat_send(6,8,0,0,4)
                status=link.query()
                if status['state']==2 and status['driver_readback_pwm_us']==NEUTRAL:
                    observed=status;break
                # A previously sent frame may still be in flight when refresh stops.
                # Use the final accepted deadline reported by firmware.
                if status['state']==1:last=status.copy()
                time.sleep(.005)
            assert observed is not None
            extra=observed['fc_ms']-(last['fc_ms']+last['remaining_ms'])
            assert 0 <= extra <= 60
            report['watchdog_neutral_observation_after_deadline_ms']=extra
            checks['firmware_watchdog_neutral_without_host_stop']=True
            checks['heartbeat_and_query_do_not_extend_control_lease']=True
            link.send(FRAME,pwm);link.spin(.04)
            assert link.query()['state']==2 and link.status['driver_readback_pwm_us']==NEUTRAL
            checks['late_frame_cannot_resume_after_fault']=True
            link.send(STOP);link.arm(False);link.send(RELEASE)
            link.wait(lambda:link.status['state']==0 and not link.status['armed'])
            assert link.status['driver_readback_pwm_us']==NEUTRAL
            checks['neutral_disarm_release_confirmed']=True
            # The deliberate legacy-injection test above sets the firmware quiet timer.
            # A new session must satisfy the documented >1 s idle-input condition.
            link.spin(1.1)
            # Test the production Python force mapper, heartbeat handling and stop
            # against the same full firmware process; only the in-memory SITL config is enabled.
            cfg=load_config(DEFAULT_CONFIG);backend=load_backend(cfg['legacy_directory'])
            logger=backend.SessionLogger(a.output.parent/'sitl_session_logs',action='direct6_sitl')
            ctrl=controller_class(backend)(f'tcp:127.0.0.1:{port}',115200,255,191,1,1,logger,5.,3.,3.)
            ctrl.master=master;ctrl.vehicle.target_system=ctrl.vehicle.target_component=1
            ctrl.process_message(link.heartbeat)
            output_config={**cfg['output'],'firmware_verified':True,'channel_mapping_verified':True,
                           'lock_file':str(Path(work)/'output.lock'),'test_voltage_v':16.}
            output=ThrusterOutput(ctrl,T200(cfg['mapping'],cfg['curve_file']),output_config,
                                  lambda kind,**data:logger.write_log(kind,**jsonable(data)))
            try:
                output.prepare()
                assert ctrl.vehicle.armed is True and ctrl.direct_status['armed']
                checks['production_prepare_waits_for_fresh_armed_heartbeat']=True
                output.set_interval_deadline(time.monotonic()+.4)
                mapped=output.send_thruster_forces([1.,-1.,2.,-2.,1.5,-1.5])
                end=time.monotonic()+.2
                while time.monotonic()<end:output.poll();time.sleep(.003)
                assert ctrl.direct_status['driver_readback_pwm_us']==list(mapped['pwm_us'])
                checks['production_forces_to_t200_pwm_to_full_firmware_hal']=True
                report['production_mapping']=jsonable(mapped)
            finally:
                report['production_stop']=jsonable(output.stop())
            assert report['production_stop']['confirmed']
            checks['production_stop_confirms_neutral_disarm_release']=True
            # Execute two unequal original control intervals against the same firmware.
            # State input is explicitly synthetic; this checks output timing, not tracking.
            records=[];steps=[];statuses=[]
            def record(kind,**data):records.append({'kind':kind,**jsonable(data)})
            output=ThrusterOutput(ctrl,T200(cfg['mapping'],cfg['curve_file']),output_config,record)
            plan={'success':True,'nominalTrajectory':np.zeros((12,3)),
                  'controlTrajectory':np.array([[1,-2],[-1,2],[2,-1],[-2,1],[1.5,-1.5],[-1.5,1.5]]),
                  'feedbackGains':[np.zeros((6,12)) for _ in range(2)],
                  'time':np.array([0.,.12,.35]),'model':parameters().model,'world':None}
            def poll():
                output.poll()
                statuses.append(dict(ctrl.direct_status))
            experiment=execute_lbts_plan(plan,{'read_state':lambda:{'state':np.zeros(12),'valid':True,'age':0.},
                'on_start':lambda *_:output.prepare(),'send_thruster_forces':output.send_thruster_forces,
                'stop':output.stop,'poll':poll,'on_step':lambda s:steps.append(jsonable(s)),
                'set_interval_deadline':output.set_interval_deadline,'require_stop_confirmation':True},
                {**cfg['executor'],'enable_actuation':True,'confirmation':'ENABLE_BLUEROV2_THRUST','real_time':True})
            assert experiment['status']=='completed',experiment['abort_reason']
            assert len(steps)==2
            first,last=(s['output']['sent_sequence'] for s in steps)
            active=[r for r in records if r['kind']=='pwm_frame' and r['op']==FRAME and first<=r['sequence']<=last]
            expected=[s['output']['pwm_us'] for s in steps]
            assert expected[0]!=expected[1] and all(r['sent_pwm_us'] in expected for r in active)
            observed=[s for s in statuses if first<=s['sequence']<=output.sequence and s['state']==1]
            assert all(s['driver_readback_pwm_us'] in expected for s in observed)
            assert all(any(s['driver_readback_pwm_us']==p for s in observed) for p in expected)
            assert output.stop() is output.stop_result and output.stop_result['confirmed']
            checks['unequal_intervals_no_extra_neutral_frame_or_sampled_hal_pulse']=True
            checks['normal_plan_completion_active_stop_and_idempotent_cleanup']=True
            report['interval_execution']={'steps':steps,'actual_time':jsonable(experiment['actual_time']),
                'stop':jsonable(output.stop_result),'sent_frames':records,'sampled_status':statuses,
                'limitation':'HAL samples and host frames, not electrical edge measurements; synthetic state'}
            # Continue production poll/refresh after the algorithm supplies no next interval.
            records=[]
            output=ThrusterOutput(ctrl,None,output_config,record)
            try:
                output.prepare();deadline=time.monotonic()+.12;output.set_interval_deadline(deadline)
                output.send_thruster_pwm(pwm)
                end=deadline+.5;observed=None;last=None;poll_errors=[]
                while time.monotonic()<end:
                    ctrl.master.mav.heartbeat_send(6,8,0,0,4)
                    output.query();ctrl.service_io()
                    s=ctrl.direct_status
                    if s['state']==1:last=dict(s)
                    if s['state']==2:
                        observed=dict(s);break
                    try:output.poll(service_io=False)
                    except RuntimeError as exc:poll_errors.append(str(exc))
                    time.sleep(.005)
                assert observed is not None and observed['driver_readback_pwm_us']==NEUTRAL
                frames=[r for r in records if r['kind']=='pwm_frame' and r['op']==FRAME and r['sent_pwm_us']!=NEUTRAL]
                assert frames and all(r['host_mono']<deadline for r in frames)
                extra=observed['fc_ms']-(last['fc_ms']+last['remaining_ms'])
                assert 0<=extra<=60
                report['missing_next_interval']={'host_interval_deadline':deadline,'nonneutral_frames':frames,
                    'last_held_status':last,'fault_status':observed,'observation_after_fc_expiry_ms':extra,
                    'poll_errors':poll_errors,'time_basis':'FC milliseconds for expiry; host monotonic for interval/refresh'}
                checks['algorithm_stall_with_poll_and_heartbeat_cannot_refresh_past_interval']=True
            finally:report['missing_next_interval_stop']=jsonable(output.stop())
            assert report['missing_next_interval_stop']['confirmed']
            logger.close();logger=None
            # Exercise the complete PWM CLI in a fresh interpreter with a minimal
            # transport-only config: no state/model/origin/curve configuration.
            master.close(); master=None
            connection=Path(work)/'connection.json'
            connection.write_text(json.dumps({'connection':f'tcp:127.0.0.1:{port}',
                'source_system':255,'target_system':1,'target_component':1,
                'connect_timeout':5.,'confirm_timeout':3.,'heartbeat_timeout':3.}))
            # Isolate each invocation's logs so a repeated test cannot observe
            # an earlier run's nonneutral frame or HAL readback.
            pwm_logs=a.output.resolve().parent/'pwm_cli_logs'/Path(work).name
            pwm_config=Path(work)/'pwm.json'
            pwm_config.write_text(json.dumps({'legacy_directory':cfg['legacy_directory'],
                'legacy_config':str(connection),'source_component':191,
                'log_directory':str(pwm_logs),
                'required_parameters':cfg['required_parameters'],
                'output':{'lease_ms':200,'refresh_period_s':.04,'lock_file':str(Path(work)/'pwm.lock')}}))
            cli_report=a.output.resolve().with_name('pwm_cli_'+Path(work).name+'.json')
            cli_cmd=[sys.executable,'-m','bluerov2.force_run','--config',str(pwm_config),
                     '--output',str(cli_report),'pwm-test','--pwm',*map(str,pwm),'--duration','.3','--execute']
            cli=subprocess.run(cli_cmd,cwd=ROOT,capture_output=True,text=True,timeout=25)
            report['pwm_cli']={'command':cli_cmd,'returncode':cli.returncode,'stdout':cli.stdout,'stderr':cli.stderr,
                               'report':str(cli_report),'logs':str(pwm_logs)}
            executed=json.loads(cli_report.read_text())
            assert cli.returncode==0 and executed['status']=='sequence_completed', executed
            checks['pwm_cli_no_dvl_origin_model_mapping_or_curve_config']=True
            assert executed['stop']['confirmed'] and executed['stop']['stop_status']['driver_readback_pwm_us']==NEUTRAL
            checks['pwm_cli_stop_neutral_disarm_release_confirmed']=True
            rows=[]
            for path in pwm_logs.glob('*.jsonl'):
                rows += [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
            assert any(row.get('record_kind')=='driver_readback' and row.get('driver_readback_pwm_us')==pwm for row in rows), 'PWM CLI HAL readback absent'
            checks['pwm_cli_held_frame_reaches_all_six_hal_channels']=True
            interrupt_logs=a.output.resolve().parent/'pwm_interrupt_logs'/Path(work).name
            interrupt_cfg=json.loads(pwm_config.read_text())
            interrupt_cfg['log_directory']=str(interrupt_logs)
            pwm_config.write_text(json.dumps(interrupt_cfg))
            interrupted_report=a.output.resolve().with_name('pwm_interrupted_'+Path(work).name+'.json')
            interrupted_cmd=cli_cmd.copy()
            interrupted_cmd[interrupted_cmd.index('--output')+1]=str(interrupted_report)
            interrupted_cmd[interrupted_cmd.index('--duration')+1]='.8'
            child=subprocess.Popen(interrupted_cmd,cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
            seen=False
            try:
                until=time.monotonic()+15
                while time.monotonic()<until and child.poll() is None:
                    for path in interrupt_logs.glob('*.jsonl'):
                        # The last line may still be in the process of being written.
                        for line in path.read_text().splitlines(keepends=True):
                            if not line.endswith('\n'):continue
                            row=json.loads(line)
                            if row.get('record_kind')=='pwm_frame' and row.get('op')==FRAME and row.get('sent_pwm_us')==pwm:
                                seen=True
                    if seen:break
                    time.sleep(.01)
                assert seen,'interruption test did not reach a nonneutral PWM command'
                child.send_signal(signal.SIGTERM)
                stdout,stderr=child.communicate(timeout=12)
            finally:
                if child.poll() is None:child.kill();child.communicate(timeout=3)
            interrupted=json.loads(interrupted_report.read_text())
            report['pwm_interrupted']={'returncode':child.returncode,'stdout':stdout,'stderr':stderr,'report':str(interrupted_report),'logs':str(interrupt_logs)}
            assert child.returncode==1 and interrupted['status']=='aborted' and 'operator_abort' in interrupted['reason'],interrupted
            assert interrupted['stop']['confirmed'],interrupted
            checks['pwm_cli_sigterm_stops_and_confirms_neutral_disarm_release']=True
            thrust_cfg=json.loads(pwm_config.read_text())
            thrust_cfg.update(mapping=cfg['mapping'],curve_file=cfg['curve_file'])
            thrust_cfg['output']=dict(output_config)
            thrust_cfg['log_directory']=str(a.output.resolve().parent/'thrust_cli_logs'/Path(work).name)
            pwm_config.write_text(json.dumps(thrust_cfg))
            thrust_report=a.output.resolve().with_name('thrust_cli_'+Path(work).name+'.json')
            forces=[1.,-1.,2.,-2.,1.5,-1.5]
            thrust_cmd=[sys.executable,'-m','bluerov2.force_run','--config',str(pwm_config),'--output',str(thrust_report),
                        'thrust-test','--forces-n',*map(str,forces),'--duration','.3','--execute']
            child=subprocess.run(thrust_cmd,cwd=ROOT,capture_output=True,text=True,timeout=25)
            executed=json.loads(thrust_report.read_text())
            report['thrust_cli']={'command':thrust_cmd,'returncode':child.returncode,'stdout':child.stdout,
                                 'stderr':child.stderr,'report':str(thrust_report)}
            assert child.returncode==0 and executed['status']=='sequence_completed' and executed['stop']['confirmed'],executed
            expected=T200(cfg['mapping'],cfg['curve_file']).map_forces(forces,executed['output']['voltage_v'])
            assert executed['output']['pwm_us']==list(expected['pwm_us'])
            assert executed['output']['voltage_source'] in ('SYS_STATUS','BATTERY_STATUS[0] sum','configured_test_voltage')
            rows=[json.loads(line) for path in Path(thrust_cfg['log_directory']).glob('*.jsonl') for line in path.read_text().splitlines()]
            assert any(r.get('record_kind')=='driver_readback' and r['driver_readback_pwm_us']==executed['output']['pwm_us'] for r in rows)
            checks['thrust_cli_same_mapper_backend_channel_order_voltage_source_and_stop']=True
            report['all_checks_passed']=True
    except Exception as exc:
        report['error']=f'{type(exc).__name__}: {exc}'
        report['traceback']=traceback.format_exc()
    finally:
        if logger is not None:logger.close()
        if master is not None:master.close()
        if proc is not None and proc.poll() is None:
            proc.terminate()
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:proc.kill();proc.wait(timeout=3)
        if proc is not None:report['owned_sitl_exit_code']=proc.returncode
        if link is not None:
            trace=a.output.with_suffix('.status.jsonl')
            trace.write_text(''.join(json.dumps(row)+'\n' for row in link.trace))
            report['status_trace']=str(trace)
        a.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False,indent=2))
    return 0 if report['all_checks_passed'] else 1


if __name__=='__main__':raise SystemExit(main())
