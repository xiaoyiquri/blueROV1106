"""Probe stock PWM passthrough in an owned localhost ArduSub SITL only.

This diagnostic deliberately exercises an unsupported output path in simulation.
It is NOT a robot executor. There is no endpoint option, no arming command,
and no direct6 import/message. Parameter writes affect the child simulator only.
"""
import argparse
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import traceback

from pymavlink import mavutil

NEUTRAL = (1500,)*6
ROOT = Path(__file__).resolve().parents[1]


def pwm_frame(values):
    """The first six physical channels in ONE RC_CHANNELS_OVERRIDE packet.

    UINT16_MAX means ignore for channels 7/8 in this MAVLink message, not neutral.
    No normalization, clipping, force sign change or MANUAL_CONTROL conversion.
    """
    import math
    if len(values) != 6 or any(not math.isfinite(v) or v != int(v) or not 1100 <= v <= 1900 for v in values):
        raise ValueError('six finite integer PWM values in 1100..1900 are required')
    return tuple(map(int, values)) + (65535,65535)


class ProbeLink:
    def __init__(self, master, system=42):
        self.master, self.system = master, system
        self.next_hb = self.next_refresh = 0.
        self.held = None
        self.heartbeat = None
        self.latest = {}
        self.parameters = {}
        self.rows = []
        self.serial = 0
        self.sent_frames = 0

    def override(self, values):
        self.master.mav.rc_channels_override_send(self.system,1,*pwm_frame(values))
        self.sent_frames += 1

    def poll(self):
        # Bound parsing so input traffic cannot starve refresh/stop operations.
        until = time.monotonic()+.005
        for _ in range(256):
            if time.monotonic() >= until:
                break
            msg = self.master.recv_match(blocking=False)
            if msg is None:
                break
            if (msg.get_srcSystem(),msg.get_srcComponent()) != (self.system,1):
                continue
            self.serial += 1
            kind = msg.get_type()
            stamp = time.monotonic()
            self.latest[kind] = (self.serial,stamp,msg)
            if kind == 'HEARTBEAT':
                self.heartbeat = msg
            elif kind == 'PARAM_VALUE':
                name = msg.param_id.decode() if isinstance(msg.param_id,bytes) else msg.param_id
                self.parameters[name.rstrip('\0')] = (self.serial,float(msg.param_value))
            elif kind == 'SERVO_OUTPUT_RAW':
                self.rows.append({'host_mono':stamp,'fc_time_usec':int(msg.time_usec),
                                  'driver_readback_pwm_us':[int(getattr(msg,f'servo{i}_raw')) for i in range(1,7)]})
        now = time.monotonic()
        if now >= self.next_hb:
            self.master.mav.heartbeat_send(6,8,0,0,4)
            self.next_hb = now+1.
        if self.held is not None and now >= self.next_refresh:
            self.override(self.held)
            self.next_refresh = now+.04

    def wait(self, predicate, timeout=3.):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            self.poll()
            if predicate():
                return
            time.sleep(.002)
        raise TimeoutError('isolated SITL probe condition timed out')

    def spin(self, seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            self.poll()
            time.sleep(.002)

    def parameter(self, name, value=None):
        serial = self.serial
        if value is None:
            self.master.mav.param_request_read_send(self.system,1,name.encode(),-1)
        else:
            self.master.mav.param_set_send(self.system,1,name.encode(),float(value),mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        self.wait(lambda:name in self.parameters and self.parameters[name][0]>serial
                  and (value is None or abs(self.parameters[name][1]-value)<1e-5))
        return self.parameters[name][1]

    def rc(self):
        for kind in ('RC_CHANNELS_RAW','RC_CHANNELS'):
            if kind in self.latest:
                return [int(getattr(self.latest[kind][2],f'chan{i}_raw')) for i in range(1,7)]
        return None


def run(binary, output, *, bench_check=False):
    binary, output = Path(binary).resolve(), Path(output).resolve()
    # Reserve/check a free local port before launching; no external UDP is used.
    with socket.socket() as sock:
        sock.bind(('127.0.0.1',0));port = sock.getsockname()[1]
    report = {'created_utc':datetime.now(timezone.utc).isoformat(),
              'binary':str(binary),'binary_sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),
              'localhost_only':True,'physical_vehicle_used':False,'arm_command_sent':False,
              'firmware_runtime_patch':'refer_to_build_manifest_and_binary_sha256',
              'hal_observation_trace_requested':bool(os.environ.get('PASSTHROUGH_TRACE_FILE')),
              'probe_source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'cases':{},'completed':False,
              'limitations':['Stock 3.5.4 excludes pilot input failsafe in SITL at compile time',
                             'SERVO_OUTPUT_RAW is sampled HAL readback, not continuous electrical measurement',
                             'No board watchdog/ESC/physical thrust validation is claimed']}
    output.parent.mkdir(parents=True,exist_ok=True)
    proc = master = link = None
    original = {}
    restored = False
    log = output.with_suffix('.console.log')
    trace_path = output.with_suffix('.jsonl')
    rows = []
    try:
        with tempfile.TemporaryDirectory(prefix='stock-passthrough-sitl-') as work,log.open('w') as console:
            defaults = Path(work)/'defaults.parm'
            defaults.write_text('FRAME_CONFIG 1\nARMING_CHECK 0\nSYSID_THISMAV 42\nSYSID_MYGCS 255\nFS_GCS_ENABLE 0\nFS_PILOT_INPUT 2\nFS_PILOT_TIMEOUT 0.2\n')
            command = [str(binary),'-w','--model','vectored','--home=-35.363261,149.165230,0,0',
                       '--base-port',str(port),'--defaults',str(defaults)]
            report['command'] = command
            proc = subprocess.Popen(command,cwd=work,stdout=console,stderr=subprocess.STDOUT)
            deadline = time.monotonic()+15
            while True:
                if proc.poll() is not None:
                    raise RuntimeError('owned SITL child exited before accepting TCP')
                try:
                    master = mavutil.mavlink_connection(f'tcp:127.0.0.1:{port}',source_system=255,source_component=191)
                    break
                except OSError:
                    if time.monotonic() >= deadline:
                        raise
                    time.sleep(.1)
            link = ProbeLink(master)
            link.wait(lambda:link.heartbeat is not None,10.)
            assert not link.heartbeat.base_mode & 128
            master.mav.command_long_send(42,1,520,0,1,0,0,0,0,0,0)
            link.wait(lambda:'AUTOPILOT_VERSION' in link.latest)
            version = int(link.latest['AUTOPILOT_VERSION'][2].flight_sw_version)
            assert (version>>24 &255,version>>16 &255,version>>8 &255) == (3,5,4)
            report['firmware_version'] = [3,5,4]
            master.mav.request_data_stream_send(42,1,mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,50,1)
            link.held = NEUTRAL
            link.spin(.3)
            original = {f'SERVO{i}_FUNCTION':link.parameter(f'SERVO{i}_FUNCTION') for i in range(1,7)}
            report['original_parameters'] = original.copy()
            expected = [1540,1460,1550,1450,1560,1440]
            for name, functions in [('function_1',[1]*6),('function_51_to_56',list(range(51,57)))]:
                for i, function in enumerate(functions,1):
                    link.parameter(f'SERVO{i}_FUNCTION',function)
                link.spin(1.1)
                link.held = expected
                link.next_refresh = 0.
                link.spin(.25)
                start = len(link.rows)
                link.spin(.6)
                observed = link.rows[start:]
                assert observed, 'no SERVO_OUTPUT_RAW telemetry'
                assert link.rc() == expected, 'override did not reach RC inputs'
                unique = sorted({tuple(row['driver_readback_pwm_us']) for row in observed})
                case = {'configured_functions':functions,'sent_pwm_us':expected,
                        'rc_input_pwm_us':link.rc(),'disarmed':not bool(link.heartbeat.base_mode&128),
                        'readback_samples':len(observed),'unique_driver_readback_pwm_us':unique,
                        'all_sampled_outputs_match_request':all(list(v)==expected for v in unique)}
                # A second component with the SAME sysid is also accepted; no component ownership exists.
                link.held = None
                other = [1510,1520,1530,1540,1550,1560]
                master.mav.srcComponent = 192
                link.override(other)
                master.mav.srcComponent = 191
                link.spin(.3)
                assert link.rc() == other
                case['same_sysid_other_component_can_replace_input'] = True
                # Continue heartbeat but omit all control frames beyond configured pilot timeout.
                link.spin(.35)
                case['input_after_control_silence_s'] = .65
                case['input_after_control_silence_pwm_us'] = link.rc()
                assert link.rc() == other  # Expected stock SITL compile-time limitation, not a safety pass.
                case['stock_sitl_override_persists_without_control_packets'] = True
                report['cases'][name] = case
                rows.extend(dict(row,case=name) for row in observed)
                link.held = NEUTRAL
                link.next_refresh = 0.
                link.spin(.15)
            # Restore exact saved values, never assumed defaults. Verify explicit read requests.
            for name, value in original.items():
                link.parameter(name,value)
            link.spin(.2)
            actual = {name:link.parameter(name) for name in original}
            report['restored_parameters'] = actual
            restored = actual == original
            assert restored and link.rc() == list(NEUTRAL)
            if bench_check:
                # Reuse this owned simulator; no other SITL framework or physical
                # endpoint is involved. Release its one TCP client for the real CLI.
                restored=False
                master.close();master=None;link=None
                connection=Path(work)/'bench_connection.json'
                connection.write_text(json.dumps({'connection':f'tcp:127.0.0.1:{port}',
                    'source_system':255,'target_system':42,'target_component':1}))
                directory=output.parent/('bench_cli_'+Path(work).name)
                cli=[sys.executable,'-m','bluerov2.passthrough_bench',
                     '--connection-config',str(connection),'--output-directory',str(directory),
                     '--execute-power-isolated-bench']
                result=subprocess.run(cli,cwd=ROOT,capture_output=True,text=True,timeout=35)
                executed=json.loads((directory/'report.json').read_text())
                report['bench_cli']={'command':cli,'returncode':result.returncode,
                    'stdout':result.stdout,'stderr':result.stderr,'report':str(directory/'report.json'),
                    'uses_only_this_owned_localhost_simulator':True,'physical_vehicle_used':False}
                # Stock 3.5.4 deliberately omits pilot input failsafe in SITL.
                # It is a REQUIRED negative result, not a physical-watchdog pass.
                assert result.returncode==1 and executed['status']=='watchdog_neutral_not_observed',executed
                assert executed['bench']['pattern_input_echo_confirmed']
                assert executed['bench']['pattern_driver_samples_confirmed']
                assert executed['bench']['stop']['telemetry_cleanup_confirmed']
                assert all(executed['bench']['stop']['restored_values'][name]==value for name,value in original.items())
                assert not executed['electrical_output_verified'] and not executed['trajectory_or_motion_verified']
                restored=True
                report['restored_parameters']=executed['bench']['stop']['restored_values']
                report['bench_cli']['bench_source_sha256']=executed['bench_source_sha256']
                report['bench_cli']['expected_missing_sitl_watchdog_reported']=True
                report['bench_cli']['pattern_and_exact_restore_confirmed']=True
            report['completed'] = True
    except Exception as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
        report['traceback'] = traceback.format_exc()
    finally:
        # Only the simulator created above is reachable. Stop/recovery is best-effort on failure.
        if link is not None and master is not None and not restored:
            try:
                link.held = NEUTRAL;link.next_refresh = 0.;link.spin(.15)
                for name, value in original.items():
                    link.parameter(name,value)
                restored = all(link.parameter(name)==value for name,value in original.items())
            except Exception as exc:
                report['cleanup_error'] = f'{type(exc).__name__}: {exc}'
        report['saved_parameters_restored_in_simulator'] = restored
        if master is not None:
            master.close()
        if proc is not None:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill();proc.wait(timeout=5)
        # No aggregate hardware-ready flag: known source-level failures remain.
        report['stock_354_passthrough_meets_task_output_contract'] = False
        trace_path.write_text(''.join(json.dumps(row)+'\n' for row in rows))
        output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--binary',type=Path,required=True)
    p.add_argument('--output',type=Path,default=Path('reports/passthrough_sitl.json'))
    p.add_argument('--bench-check',action='store_true',help='also run the power-isolated bench CLI against this owned simulator')
    a = p.parse_args()
    # This old SITL also owns fixed internal simulator ports. TCP base-port alone
    # does not isolate two instances, so serialize this diagnostic's children.
    with (Path(tempfile.gettempdir())/'bluerov2-stock-passthrough-sitl.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        result = run(a.binary,a.output,bench_check=a.bench_check)
    print(json.dumps(result,ensure_ascii=False,indent=2))
    return 0 if result['completed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
