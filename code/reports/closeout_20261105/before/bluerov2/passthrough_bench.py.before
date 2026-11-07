"""Stock 3.5.4 electrical bench diagnostic, NOT a trajectory/motion backend.

Default: read-only. Nonneutral output requires explicitly isolated actuator power.
No arm/disarm, mode, GUIDED, MANUAL_CONTROL, direct6, DVL, origin or T200 imports.
"""
import argparse
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import math
from numbers import Real
import os
from pathlib import Path
import signal
import socket
import time

from .communication import load_communication_backend, load_connection_config

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONNECTION = ROOT.parent/'host_goto_config.json'
OUTPUT_LOCK = Path('/tmp/bluerov2-passthrough-bench.lock')
NEUTRAL = (1500,)*6
PATTERN = (1540,1460,1550,1450,1560,1440)
FUNCTIONS = tuple(f'SERVO{i}_FUNCTION' for i in range(1,7))
PARAMETERS = ('SYSID_MYGCS','FRAME_CONFIG','MOT_PWM_TYPE','FS_PILOT_INPUT','FS_PILOT_TIMEOUT') + tuple(
    f'SERVO{i}_{suffix}' for i in range(1,7) for suffix in ('FUNCTION','MIN','MAX','TRIM','REVERSED'))


class BenchAbort(RuntimeError):
    pass


def frame(values):
    if len(values)!=6 or any(not isinstance(v,Real) or not math.isfinite(v) or v!=int(v) or not 1100<=v<=1900 for v in values):
        raise ValueError('six finite integer PWM values in 1100..1900 required')
    return tuple(map(int,values))+(65535,65535)


def durable_json(path, data):
    # The original parameter values must reach disk BEFORE the first PARAM_SET.
    with Path(path).open('x',encoding='utf-8') as stream:
        json.dump(data,stream,ensure_ascii=False,indent=2,allow_nan=False)
        stream.write('\n');stream.flush();os.fsync(stream.fileno())
    descriptor=os.open(str(Path(path).parent),os.O_RDONLY|os.O_DIRECTORY)
    try:os.fsync(descriptor)
    finally:os.close(descriptor)


def readonly_controller(backend):
    class ReadOnlyController(backend.Controller):
        def __init__(self,*a,**kw):
            super().__init__(*a,**kw)
            self.serial=0;self.messages={};self.param_records={};self.records=[]
            self.boot_clocks={};self.bench_fault=None

        def _send_heartbeat(self):
            # Receiving telemetry must not keep another GCS failsafe alive.
            pass

        def send_manual(self,*_):
            raise BenchAbort('MANUAL_CONTROL prohibited in passthrough bench')

        def process_message(self,msg):
            super().process_message(msg)
            if (msg.get_srcSystem(),msg.get_srcComponent())!=(self.target_system,self.target_component):
                return
            now=time.monotonic();kind=msg.get_type();data=msg.to_dict();self.serial+=1
            if 'time_boot_ms' in data:
                current=int(data['time_boot_ms']);previous=self.boot_clocks.get(kind)
                if previous is not None and current<previous and previous-current<2**31:
                    self.bench_fault='device clock rollback/reboot: '+kind
                self.boot_clocks[kind]=current
            self.messages[kind]=(now,data)
            self.records.append((now,kind,data))
            if kind=='PARAM_VALUE':
                name=data['param_id']
                if isinstance(name,bytes):name=name.decode('ascii')
                self.param_records[name.rstrip('\0')]=(self.serial,float(data['param_value']))
    return ReadOnlyController


class BenchSession:
    """Small diagnostic transaction; never reusable as a motion controller."""
    def __init__(self,controller,directory,log=lambda *_a,**_k:None):
        self.c=controller;self.directory=Path(directory);self.log=log
        self.original={};self.changes_attempted=[];self.allow_outputs=False
        self.last_frame_mono=None;self.phase='readonly';self.sent=[]
        self._send_original=controller.master.mav.send
        controller.master.mav.send=self._guarded_send

    def _guarded_send(self,msg,*a,**kw):
        data=msg.to_dict();kind=msg.get_type()
        target=(data.get('target_system'),data.get('target_component'))
        if target!=(self.c.target_system,self.c.target_component):
            raise BenchAbort('message targets a different flight controller')
        if kind=='PARAM_REQUEST_READ':
            name=data['param_id'];name=name.decode() if isinstance(name,bytes) else name
            if name.rstrip('\0') not in PARAMETERS:raise BenchAbort('parameter read outside bench scope')
        elif kind=='COMMAND_LONG':
            if data['command']!=520 or data['param1']!=1 or any(data[f'param{i}']!=0 for i in range(2,8)):
                raise BenchAbort('only the read-only version request is allowed')
        elif kind=='PARAM_SET':
            name=data['param_id'];name=name.decode() if isinstance(name,bytes) else name;name=name.rstrip('\0')
            value=float(data['param_value'])
            if self.c.vehicle.armed is not False:raise BenchAbort('parameter write prohibited while armed/unknown')
            if not self.allow_outputs or name not in self.original or value not in (1.,self.original[name]):
                raise BenchAbort('parameter write not backed up or outside six FUNCTION values')
        elif kind=='RC_CHANNELS_OVERRIDE':
            values=tuple(data[f'chan{i}_raw'] for i in range(1,9))
            if not self.allow_outputs or values!=frame(values[:6]):raise BenchAbort('invalid/unpermitted override')
            if any(data.get(f'chan{i}_raw',0) not in (0,65535) for i in range(9,19)):
                raise BenchAbort('extended override outside six-channel bench scope')
            if values[:6] not in (NEUTRAL,PATTERN):raise BenchAbort('bench sends only its bounded fixed pattern or neutral')
            if self.phase not in ('neutral','pattern','cleanup'):raise BenchAbort('override outside active bench phase')
            if self.phase in ('neutral','cleanup') and values[:6]!=NEUTRAL:raise BenchAbort('nonneutral override during neutral/cleanup phase')
        else:
            raise BenchAbort('message prohibited in bench: '+kind)
        self.sent.append({'host_mono':time.monotonic(),'phase':self.phase,'message_type':kind,'message':data})
        self.log('bench_request',phase=self.phase,message_type=kind,raw=data)
        return self._send_original(msg,*a,**kw)

    def poll(self,*,cleanup=False):
        self.c.service_io()
        if not cleanup:
            if self.c.stop_requested:raise BenchAbort('operator_abort')
            if self.c.bench_fault:raise BenchAbort(self.c.bench_fault)
            if not self.c.vehicle.heartbeat_fresh(time.monotonic(),self.c.heartbeat_timeout):
                raise BenchAbort('flight controller heartbeat stale')
            if self.allow_outputs and self.c.vehicle.armed is not False:
                raise BenchAbort('vehicle armed or arm state unknown during disarmed bench')
            if self.allow_outputs and self.c.vehicle.custom_mode!=19:
                raise BenchAbort('flight mode changed during disarmed bench')

    def wait(self,predicate,timeout=2.,*,cleanup=False):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            self.poll(cleanup=cleanup)
            if predicate():return
            time.sleep(.005)
        raise BenchAbort('bench response/confirmation timed out')

    def read_parameter(self,name,*,cleanup=False):
        serial=self.c.serial
        self.c.master.mav.param_request_read_send(self.c.target_system,self.c.target_component,name.encode(),-1)
        self.wait(lambda:name in self.c.param_records and self.c.param_records[name][0]>serial,cleanup=cleanup)
        value=self.c.param_records[name][1]
        if not math.isfinite(value):raise BenchAbort('nonfinite parameter: '+name)
        return value

    def set_function(self,name,value,*,cleanup=False):
        if name not in FUNCTIONS:raise BenchAbort('not a thruster FUNCTION parameter')
        # Record the attempted write even if its echo is lost: restoration is still required.
        if name not in self.changes_attempted:self.changes_attempted.append(name)
        serial=self.c.serial
        self.c.master.mav.param_set_send(self.c.target_system,self.c.target_component,name.encode(),float(value),9)
        self.wait(lambda:name in self.c.param_records and self.c.param_records[name][0]>serial
                  and self.c.param_records[name][1]==value,cleanup=cleanup)
        if self.read_parameter(name,cleanup=cleanup)!=value:
            raise BenchAbort('FUNCTION readback mismatch: '+name)

    def latest_vector(self,kind,prefix,*,after=-math.inf,max_age=.25):
        record=self.c.messages.get(kind)
        if record is None:return None
        stamp,data=record
        if stamp<after or time.monotonic()-stamp>max_age:return None
        try:values=tuple(float(data[f'{prefix}{i}_raw']) for i in range(1,7))
        except (KeyError,TypeError,ValueError):return None
        if not all(math.isfinite(v) for v in values):return None
        return values

    def input_matches(self,values,*,after=-math.inf):
        return any(self.latest_vector(kind,'chan',after=after)==tuple(values) for kind in ('RC_CHANNELS','RC_CHANNELS_RAW'))

    def output_matches(self,values,*,after=-math.inf):
        return self.latest_vector('SERVO_OUTPUT_RAW','servo',after=after)==tuple(values)

    def snapshot(self):
        self.c.master.mav.command_long_send(self.c.target_system,self.c.target_component,520,0,1,0,0,0,0,0,0)
        values={name:self.read_parameter(name) for name in PARAMETERS}
        self.wait(lambda:bool(self.c.vehicle.autopilot_version))
        return {'target':[self.c.target_system,self.c.target_component],
                'firmware':self.c.vehicle.autopilot_version,'parameters':values,
                'armed':self.c.vehicle.armed,'mode':self.c.vehicle.mode}

    def check_preflight(self,snapshot,*,recovery=False):
        self.poll();p=snapshot['parameters'];v=snapshot['firmware']
        version=int(v.get('flight_sw_version',0))
        if tuple((version>>shift)&255 for shift in (24,16,8))!=(3,5,4):
            raise BenchAbort('electrical bench is scoped to inspected ArduSub 3.5.4')
        commit=bytes(v.get('flight_custom_version',[])).rstrip(b'\0')
        if commit!=b'05fed49b':raise BenchAbort('unrecognized firmware custom version')
        if self.c.vehicle.armed is not False or self.c.vehicle.custom_mode!=19:
            raise BenchAbort('bench requires an already disarmed MANUAL vehicle; no automatic mode/arm changes')
        if p['FRAME_CONFIG']!=1 or p['MOT_PWM_TYPE']!=0:raise BenchAbort('not standard vectored PWM configuration')
        if p['SYSID_MYGCS']!=self.c.source_system:raise BenchAbort('configured source system does not match SYSID_MYGCS')
        if p['FS_PILOT_INPUT']!=2 or not .1<=p['FS_PILOT_TIMEOUT']<=3.:
            raise BenchAbort('pilot input failsafe is not the inspected disarm configuration')
        for i,value in enumerate(PATTERN,1):
            allowed=(1.,32+i) if recovery else (32+i,)
            if p[f'SERVO{i}_FUNCTION'] not in allowed:raise BenchAbort('refusing an unexpectedly reassigned thruster output')
            if p[f'SERVO{i}_TRIM']!=1500 or not p[f'SERVO{i}_MIN']<=value<=p[f'SERVO{i}_MAX']:
                raise BenchAbort('PWM pattern incompatible with configured electrical output bounds')
        if not recovery:self.wait(lambda:self.input_matches(NEUTRAL) and self.output_matches(NEUTRAL))

    def send_pwm(self,values):
        values=frame(values)
        self.c.master.mav.rc_channels_override_send(self.c.target_system,self.c.target_component,*values)
        self.last_frame_mono=time.monotonic()

    def hold(self,values,duration,phase):
        if not math.isfinite(duration) or not 0<duration<=1.:raise ValueError('bench hold must be in (0,1] seconds')
        self.phase=phase;end=time.monotonic()+duration;next_send=time.monotonic()
        while time.monotonic()<end:
            self.poll();now=time.monotonic()
            if now-next_send>.08:raise BenchAbort('bench refresh timing overrun; no catch-up burst')
            if now>=next_send:
                self.send_pwm(values);next_send=now+.04
            time.sleep(min(.005,max(0.,end-time.monotonic())))

    def cleanup(self):
        result={'neutral_input_echo_confirmed':False,'neutral_driver_samples_confirmed':False,
                'parameters_restored':False,'disarmed_heartbeat_confirmed':False,
                'telemetry_cleanup_confirmed':False,'electrical_stop_confirmed':False}
        if not self.changes_attempted:
            result['no_output_transaction_started']=True
            return result
        self.phase='cleanup';since=time.monotonic()
        try:
            self.send_pwm(NEUTRAL)
            self.wait(lambda:self.input_matches(NEUTRAL,after=since) and self.output_matches(NEUTRAL,after=since),cleanup=True)
            result['neutral_input_echo_confirmed']=True;result['neutral_driver_samples_confirmed']=True
        except Exception as exc:
            result['neutral_error']=f'{type(exc).__name__}: {exc}'
        try:
            self.wait(lambda:self.c.vehicle.armed is False and self.c.vehicle.last_heartbeat_mono is not None
                      and self.c.vehicle.last_heartbeat_mono>=since,cleanup=True)
            result['disarmed_heartbeat_confirmed']=True
            # Never restore normal motor functions while a different actor has armed the vehicle.
            for name in self.changes_attempted:
                self.set_function(name,self.original[name],cleanup=True)
            result['restored_values']={name:self.read_parameter(name,cleanup=True) for name in self.original}
            result['parameters_restored']=result['restored_values']==self.original
        except Exception as exc:
            result['restore_error']=f'{type(exc).__name__}: {exc}'
        result['disarmed_heartbeat_confirmed']=bool(result['disarmed_heartbeat_confirmed']
            and self.c.vehicle.armed is False
            and self.c.vehicle.heartbeat_fresh(time.monotonic(),self.c.heartbeat_timeout))
        result['telemetry_cleanup_confirmed']=all(result[k] for k in (
            'neutral_input_echo_confirmed','neutral_driver_samples_confirmed','parameters_restored','disarmed_heartbeat_confirmed'))
        self.allow_outputs=False
        return result

    def execute(self,snapshot,*,power_isolated=False,hold_s=.3):
        if not power_isolated:raise BenchAbort('explicit isolated actuator-power confirmation is required')
        if not math.isfinite(hold_s) or not 0<hold_s<=1.:raise ValueError('bench hold must be in (0,1] seconds')
        self.check_preflight(snapshot)
        self.original={name:snapshot['parameters'][name] for name in FUNCTIONS}
        durable_json(self.directory/'original_parameters.json',{
            'target':snapshot['target'],'firmware':snapshot['firmware'],'parameters':self.original,
            'actuator_power_isolation':'operator assertion; not automatically measurable'})
        self.allow_outputs=True
        self.phase='configure'
        result={'status':'bench_started','independent_output_validated':False,
                'electrical_measurement_supplied':False,'pattern_pwm_us':PATTERN,'stop':None}
        try:
            for name in FUNCTIONS:self.set_function(name,1.)
            self.hold(NEUTRAL,.2,'neutral')
            since=time.monotonic()
            self.hold(PATTERN,hold_s,'pattern')
            result['pattern_input_echo_confirmed']=self.input_matches(PATTERN,after=since)
            result['pattern_driver_samples_confirmed']=self.output_matches(PATTERN,after=since)
            if not result['pattern_input_echo_confirmed'] or not result['pattern_driver_samples_confirmed']:
                raise BenchAbort('complete pattern was not observed in fresh input/output telemetry')
            self.phase='silence';last_send=self.last_frame_mono
            timeout=snapshot['parameters']['FS_PILOT_TIMEOUT']+1.
            deadline=time.monotonic()+timeout
            result['pilot_input_neutral_after_silence_observed']=False
            while time.monotonic()<deadline:
                self.poll()
                if self.input_matches(NEUTRAL,after=last_send) and self.output_matches(NEUTRAL,after=last_send):
                    result['pilot_input_neutral_after_silence_observed']=True
                    result['neutral_delay_host_observed_s']=time.monotonic()-last_send
                    break
                time.sleep(.005)
            # Exact 3.5.4 emits this STATUSTEXT when its pilot-input failsafe acts.
            # Record it separately; a neutral sample alone could have another cause.
            result['pilot_failsafe_event_observed']=any(t>=last_send and kind=='STATUSTEXT'
                and str(raw.get('text','')).rstrip('\0')=='Lost manual control'
                for t,kind,raw in self.c.records)
            if not result['pilot_input_neutral_after_silence_observed']:
                result['status']='watchdog_neutral_not_observed'
            elif not result['pilot_failsafe_event_observed']:
                result['status']='failsafe_event_unconfirmed'
            else:result['status']='bench_observations_complete'
        except BaseException as exc:
            result['status']='bench_aborted';result['reason']=f'{type(exc).__name__}: {exc}'
        finally:
            result['stop']=self.cleanup()
            if not result['stop']['telemetry_cleanup_confirmed']:result['status']='cleanup_unconfirmed'
        return result

    def restore_backup(self,snapshot,path,*,power_isolated=False):
        if not power_isolated:raise BenchAbort('recovery also requires physically isolated actuator power')
        backup=json.loads(Path(path).read_text())
        if backup.get('target')!=snapshot['target']:raise BenchAbort('backup flight controller target mismatch')
        for field in ('flight_sw_version','flight_custom_version','uid'):
            if backup.get('firmware',{}).get(field)!=snapshot['firmware'].get(field):
                raise BenchAbort('backup flight controller identity/version mismatch')
        original=backup.get('parameters',{})
        if set(original)!=set(FUNCTIONS) or any(original[name]!=33+i for i,name in enumerate(FUNCTIONS)):
            raise BenchAbort('backup is not from this standard six-channel bench')
        self.check_preflight(snapshot,recovery=True)
        self.original=original
        self.changes_attempted=[name for name in FUNCTIONS if snapshot['parameters'][name]!=original[name]]
        if not self.changes_attempted:return {'status':'already_restored','parameter_writes_required':False}
        self.allow_outputs=True
        result=self.cleanup()
        return {'status':'backup_restored' if result['telemetry_cleanup_confirmed'] else 'cleanup_unconfirmed','stop':result}


def require_free_udp_endpoint(endpoint):
    if endpoint.startswith('udpin:'):
        host,port=endpoint[len('udpin:'):].rsplit(':',1)
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as probe:
            # No SO_REUSEADDR: do not steal data from an existing controller/QGC.
            probe.bind((host,int(port)))


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--connection-config',type=Path,default=DEFAULT_CONNECTION)
    p.add_argument('--output-directory',type=Path)
    p.add_argument('--execute-power-isolated-bench',action='store_true',help='ONLY with all actuator power physically isolated; never a motion command')
    p.add_argument('--restore-backup',type=Path,help='restore this bench backup instead of sending its test pattern; requires the power-isolated flag')
    args=p.parse_args(argv)
    directory=args.output_directory or ROOT/'reports'/('passthrough_bench_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ'))
    directory.mkdir(parents=True,exist_ok=False)
    report={'created_utc':datetime.now(timezone.utc).isoformat(),'mode':'isolated_electrical_bench' if args.execute_power_isolated_bench else 'readonly',
            'bench_source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            'operator_power_isolation_asserted':args.execute_power_isolated_bench,
            'actuation_to_powered_thrusters_authorized':False,'physical_power_isolation_automatically_verified':False,
            'arm_or_mode_changes_requested':False,'electrical_output_verified':False,'trajectory_or_motion_verified':False,
            'independent_output_verified':False,'status':'not_connected'}
    controller=logger=session=None;handlers={};lock=None
    try:
        config=load_connection_config(args.connection_config)
        require_free_udp_endpoint(config['connection'])
        if args.execute_power_isolated_bench:
            lock=OUTPUT_LOCK.open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        backend=load_communication_backend(ROOT.parent);logger=backend.SessionLogger(directory,action='passthrough_bench')
        controller=readonly_controller(backend)(config['connection'],config['baud'],config['source_system'],191,
            config.get('target_system'),config.get('target_component'),logger,8.,3.,3.)
        def interrupted(signum,_frame):
            controller.stop_requested=True;controller.termination_reason=f'signal_{signum}'
        handlers={s:signal.signal(s,interrupted) for s in (signal.SIGINT,signal.SIGTERM)}
        controller.connect()
        if controller.vehicle.vehicle_type!=12 or controller.vehicle.autopilot!=3:raise BenchAbort('not an ArduSub flight controller')
        session=BenchSession(controller,directory,logger.write_log)
        report['snapshot']=session.snapshot()
        if args.restore_backup:
            report['bench']=session.restore_backup(report['snapshot'],args.restore_backup,power_isolated=args.execute_power_isolated_bench)
            report['status']=report['bench']['status']
        elif args.execute_power_isolated_bench:
            report['bench']=session.execute(report['snapshot'],power_isolated=True)
            report['status']=report['bench']['status']
        else:report['status']='read_only_completed'
    except BaseException as exc:
        report['status']='rejected';report['reason']=f'{type(exc).__name__}: {exc}'
        if session is not None and session.changes_attempted:
            report['exception_cleanup']=session.cleanup()
    finally:
        if session is not None:
            report['outgoing_message_types']=[entry['message_type'] for entry in session.sent]
            report['parameter_writes']=[entry for entry in session.sent if entry['message_type']=='PARAM_SET']
            report['override_frame_count']=sum(entry['message_type']=='RC_CHANNELS_OVERRIDE' for entry in session.sent)
            report['observations']=[{'host_mono':t,'type':kind,'message':raw} for t,kind,raw in controller.records
                                    if kind in ('SERVO_OUTPUT_RAW','RC_CHANNELS','RC_CHANNELS_RAW','STATUSTEXT','HEARTBEAT')]
        if controller is not None:controller.close()
        if logger is not None:logger.close()
        if lock is not None:lock.close()
        for sig,old in handlers.items():signal.signal(sig,old)
        report['completed_utc']=datetime.now(timezone.utc).isoformat()
        path=directory/'report.json';path.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
        print(json.dumps({'status':report['status'],'report':str(path),'reason':report.get('reason')},ensure_ascii=False))
    return 0 if report['status'] in ('read_only_completed','bench_observations_complete','backup_restored','already_restored') else 1


if __name__=='__main__':
    raise SystemExit(main())
