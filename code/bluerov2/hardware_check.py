"""Passive telemetry diagnostics retained for the existing project (no actuator messages)."""
import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import socket
import time
import numpy as np
from .dynamics import rotation_zyx


def mavlink_state(attitude,position):
    angles=np.array([attitude[k] for k in ('roll','pitch','yaw')])
    world_velocity=np.array([position[k] for k in ('vx','vy','vz')])
    result=np.r_[[position[k] for k in ('x','y','z')],angles,rotation_zyx(*angles).T@world_velocity,
                 [attitude[k] for k in ('rollspeed','pitchspeed','yawspeed')]]
    if not np.all(np.isfinite(result)):raise ValueError('state must be finite')
    return result


def dvl_report(data):
    if 'velocity_valid' in data:
        velocity=[float(data[k]) for k in ('vx','vy','vz')]
        valid=data.get('status',0)==0 and data['velocity_valid'] and bool(data.get('transducers')) and all(b.get('beam_valid',False) for b in data['transducers'])
        if not np.all(np.isfinite(velocity)):raise ValueError('nonfinite DVL velocity')
        return {'kind':'velocity','velocity_m_s':velocity,'valid':bool(valid),'frame':'native DVL sensor; alignment required'}
    std=float(data['std'])
    pose=[float(data[k]) for k in ('x','y','z')]+np.deg2rad([data[k] for k in ('roll','pitch','yaw')]).tolist()
    if not np.isfinite(std) or std<0 or not np.all(np.isfinite(pose)):raise ValueError('invalid DVL pose/uncertainty')
    return {'kind':'position_local','pose_m_rad':pose,'valid':data['status']==0,'std_m':std,'frame':'fixed DVL reset world; alignment required'}


def _collect_mavlink(endpoint,baud,duration,max_age):
    from pymavlink import mavutil
    report={'endpoint':endpoint,'connected':False,'state':None,'complete_fresh_state':False}
    master=None;messages={};counts=Counter();candidates=set()
    try:
        master=mavutil.mavlink_connection(endpoint,baud=baud);report['connected']=True
        deadline=time.monotonic()+duration
        while time.monotonic()<deadline:
            msg=master.recv_match(blocking=False)
            if msg is None:time.sleep(.002);continue
            kind=msg.get_type();source=(msg.get_srcSystem(),msg.get_srcComponent());counts[kind]+=1
            if kind=='HEARTBEAT' and msg.type==12 and msg.autopilot==3:candidates.add(source)
            if len(candidates)==1 and source==next(iter(candidates)):messages[kind]=(time.monotonic(),msg.to_dict())
        if len(candidates)>1:raise ValueError('multiple flight controllers')
        required=('ATTITUDE','LOCAL_POSITION_NED')
        missing=[k for k in required if k not in messages];report['missing_state_messages']=missing
        if not missing:
            report['state']=mavlink_state(messages['ATTITUDE'][1],messages['LOCAL_POSITION_NED'][1]).tolist()
            report['complete_fresh_state']=all(time.monotonic()-messages[k][0]<=max_age for k in (*required,'HEARTBEAT'))
        report.update(message_counts=dict(counts),candidates=sorted(candidates),messages={k:v[1] for k,v in messages.items()})
    except Exception as exc:report['error']=f'{type(exc).__name__}: {exc}'
    finally:
        if master is not None:master.close()
    return report


def _collect_dvl(host,port,duration,max_age):
    report={'endpoint':f'{host}:{port}','connected':False,'valid_fresh_reports':False};counts=Counter();latest={}
    try:
        with socket.create_connection((host,port),timeout=2) as stream:
            report['connected']=True;stream.settimeout(min(.01,duration));buffer=b'';end=time.monotonic()+duration
            while time.monotonic()<end:
                try:chunk=stream.recv(8192)
                except socket.timeout:continue
                if not chunk:break
                buffer+=chunk
                if len(buffer)>1024*1024:raise ValueError('DVL buffer exceeded 1 MiB')
                while b'\n' in buffer:
                    line,buffer=buffer.split(b'\n',1)
                    if not line.strip():continue
                    data=dvl_report(json.loads(line));kind=data['kind'];counts[kind]+=1;latest[kind]=(time.monotonic(),data)
            report['valid_fresh_reports']=all(k in latest and latest[k][1]['valid'] and time.monotonic()-latest[k][0]<=max_age for k in ('position_local','velocity'))
    except Exception as exc:report['error']=f'{type(exc).__name__}: {exc}'
    report.update(report_counts=dict(counts),reports={k:v[1] for k,v in latest.items()})
    return report


def check_hardware(connection,baud=115200,dvl_ip='192.168.2.95',dvl_port=16171,duration=5.,max_age=.5,skip_dvl=False):
    if not np.isfinite(duration) or not 0<duration<=60:raise ValueError('duration must be in (0,60]')
    with ThreadPoolExecutor(max_workers=2) as pool:
        a=pool.submit(_collect_mavlink,connection,baud,duration,max_age)
        b=None if skip_dvl else pool.submit(_collect_dvl,dvl_ip,dvl_port,duration,max_age)
        mav=a.result();dvl=None if b is None else b.result()
    return {'passive':True,'mavlink':mav,'dvl':dvl,'telemetry_complete':mav['complete_fresh_state'] and (dvl is None or dvl['valid_fresh_reports'])}


def main(argv=None):
    p=argparse.ArgumentParser();p.add_argument('--config',type=Path,default=Path('../host_goto_config.json'));p.add_argument('--duration',type=float,default=5.);p.add_argument('--skip-dvl',action='store_true');p.add_argument('--output',type=Path,default=Path('hardware_check_report.json'))
    args=p.parse_args(argv);c=json.loads(args.config.read_text())
    report=check_hardware(c['connection'],c.get('baud',115200),c.get('dvl_ip','192.168.2.95'),c.get('dvl_port',16171),args.duration,skip_dvl=args.skip_dvl)
    args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(json.dumps(report,ensure_ascii=False))
    return 0 if report['telemetry_complete'] else 1

if __name__=='__main__':raise SystemExit(main())
