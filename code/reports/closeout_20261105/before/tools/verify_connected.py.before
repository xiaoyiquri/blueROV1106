"""Read-only capture of the existing A50, MAVLink and GV7 links. Never sends commands."""
import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import json
from pathlib import Path
import socket
import struct
import sys
import time
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from bluerov2.force_run import DEFAULT_CONFIG, load_config
from bluerov2.hardware_check import dvl_report


def stats(values):
    a = np.asarray(values, float)
    a = a[np.isfinite(a)]
    return None if not len(a) else dict(min=float(a.min()), median=float(np.median(a)),
                                        p95=float(np.percentile(a, 95)), max=float(a.max()))


def stream_stats(rows, duration):
    stamps = [r['monotonic_s'] for r in rows]
    gaps = np.diff(stamps)
    return dict(count=len(rows), rate_hz=len(rows)/duration, receipt_gap_s=stats(gaps),
                last=None if not rows else rows[-1]['data'])


class MipParser:
    """Only decodes documented data fields in the vendored MicroStrain SDK."""
    def __init__(self):
        self.buffer = b''
        self.bad_checksums = 0

    def feed(self, chunk):
        self.buffer += chunk
        result = []
        while len(self.buffer) >= 4:
            start = self.buffer.find(b'\x75\x65')
            if start < 0:
                self.buffer = self.buffer[-1:]
                break
            self.buffer = self.buffer[start:]
            if len(self.buffer) < 4:
                break
            size = self.buffer[3] + 6
            if len(self.buffer) < size:
                break
            packet = self.buffer[:size]
            a = b = 0
            for byte in packet[:-2]:
                a = (a+byte) & 255
                b = (b+a) & 255
            if packet[-2:] != bytes((a,b)):
                self.bad_checksums += 1
                self.buffer = self.buffer[1:]
                continue
            self.buffer = self.buffer[size:]
            fields = {}
            offset = 4
            while offset < size-2:
                length, descriptor = packet[offset:offset+2]
                if length < 2 or offset+length > size-2:
                    fields['malformed'] = True
                    break
                payload = packet[offset+2:offset+length]
                key = f'0x{descriptor:02x}'
                fields[key] = {'size':len(payload), 'hex':payload.hex()}
                if descriptor == 0xD3 and len(payload) == 12:
                    tow, week, valid = struct.unpack('>dHH', payload)
                    fields[key].update(tow_s=tow, week=week, valid_flags=valid)
                if descriptor == 0xD5 and len(payload) == 8:
                    fields[key]['reference_ns'] = struct.unpack('>Q', payload)[0]
                if packet[2] == 0x80 and descriptor in (4,5,6,7,8) and len(payload) == 12:
                    fields[key]['vector'] = list(struct.unpack('>3f', payload))
                    fields[key]['units'] = {4:'g',5:'rad/s',6:'gauss',7:'rad per sample',8:'g*s per sample'}[descriptor]
                if packet[2] == 0x82 and descriptor in (5,13,14) and len(payload) == 14:
                    values = struct.unpack('>3fH', payload)
                    fields[key].update(vector=list(values[:3]), valid_flags=values[3])
                    fields[key]['units'] = {5:'rad',13:'m/s^2',14:'rad/s'}[descriptor]
                if packet[2] == 0x82 and descriptor == 0x10 and len(payload) == 6:
                    fields[key]['status_words'] = list(struct.unpack('>3H',payload))
                if packet[2] == 0x82 and descriptor == 3 and len(payload) == 18:
                    values = struct.unpack('>4fH',payload)
                    fields[key].update(quaternion_wxyz=list(values[:4]), valid_flags=values[4])
                offset += length
            result.append({'descriptor_set':f'0x{packet[2]:02x}', 'fields':fields})
        return result


def tcp_capture(host, port, duration, kind):
    rows = []
    report = {'endpoint':f'{host}:{port}', 'bytes_sent':0}
    parser = MipParser()
    try:
        with socket.create_connection((host,port),timeout=3) as sock:
            report['connected'] = True
            sock.settimeout(.02)
            end = time.monotonic()+duration
            buffer = b''
            while time.monotonic() < end:
                try:
                    chunk = sock.recv(16384)
                except socket.timeout:
                    continue
                if not chunk:
                    raise ConnectionError('TCP peer closed')
                if kind == 'gv7':
                    data = parser.feed(chunk)
                else:
                    buffer += chunk
                    if len(buffer) > 1048576:
                        raise ValueError('DVL buffer overflow')
                    data = []
                    while b'\n' in buffer:
                        line, buffer = buffer.split(b'\n',1)
                        if line.strip():
                            data.append(json.loads(line))
                for item in data:
                    rows.append({'monotonic_s':time.monotonic(), 'host_unix_s':time.time(), 'data':item})
    except Exception as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
    if kind == 'gv7':
        report['bad_checksums'] = parser.bad_checksums
    return rows, report


def mav_capture(endpoint, baud, duration):
    from pymavlink import mavutil
    rows = []
    report = {'endpoint':endpoint, 'bytes_sent':0}
    master = None
    try:
        master = mavutil.mavlink_connection(endpoint, baud=baud, source_system=253, source_component=192)
        end = time.monotonic()+duration
        while time.monotonic() < end:
            msg = master.recv_match(blocking=False)
            if msg is None:
                time.sleep(.002)
                continue
            if msg.get_type() == 'BAD_DATA':
                continue
            rows.append({'monotonic_s':time.monotonic(), 'host_unix_s':time.time(),
                         'source':[msg.get_srcSystem(),msg.get_srcComponent()], 'data':msg.to_dict()})
    except Exception as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
    finally:
        if master is not None:
            master.close()
    return rows, report


def timing_check(dvl, attitude, cfg):
    streams = [('position',r) if 'ts' in r['data'] else ('velocity',r) for r in dvl]
    streams += [('gyro',r) for r in attitude]
    streams.sort(key=lambda row:row[1]['monotonic_s'])
    latest = {}
    counters = Counter()
    for kind,row in streams:
        latest[kind] = row['monotonic_s']
        if len(latest) < 3:
            continue
        stamps = list(latest.values())
        if max(stamps)-min(stamps) > cfg['max_age_s']:
            counters['receipt_age_exceeded'] += 1
        elif max(stamps)-min(stamps) > cfg['max_receipt_skew_s']:
            counters['receipt_skew_exceeded'] += 1
        else:
            counters['receipt_timing_eligible'] += 1
    return {'event_checks':dict(counters), 'does_not_certify_coordinates_or_acquisition_latency':True}


def summarize(dvl,mav,gv7,duration,cfg):
    vel = [r for r in dvl if 'velocity_valid' in r['data']]
    dr = [r for r in dvl if 'ts' in r['data']]
    sources = sorted({tuple(r['source']) for r in mav if r['data']['mavpackettype']=='HEARTBEAT'
                      and r['data']['type']==12 and r['data']['autopilot']==3})
    fc = [r for r in mav if len(sources)==1 and tuple(r['source'])==sources[0]]
    attitude = [r for r in fc if r['data']['mavpackettype']=='ATTITUDE']
    mav_by_kind = {k:[r for r in fc if r['data']['mavpackettype']==k]
                   for k in sorted({r['data']['mavpackettype'] for r in fc})}
    processing = [(r['data']['time_of_transmission']-r['data']['time_of_validity'])/1e6
                  for r in vel if all(k in r['data'] for k in ('time_of_transmission','time_of_validity'))]
    clock_offset = [r['host_unix_s']-r['data']['time_of_transmission']/1e6 for r in vel
                    if 'time_of_transmission' in r['data']]
    timestamps = {k:[r['data'][field]/scale for r in rows if field in r['data']]
                  for k,rows,field,scale in [('velocity',vel,'time_of_validity',1e6),('pose',dr,'ts',1)]}
    return {
        'target_candidates':sources,
        'dvl':{'velocity':stream_stats(vel,duration),'pose':stream_stats(dr,duration),
               'velocity_valid_count':sum(dvl_report(r['data'])['valid'] for r in vel),
               'pose_valid_count':sum(dvl_report(r['data'])['valid'] for r in dr),
               'device_delta_s':{k:stats(np.diff(v)) for k,v in timestamps.items()},
               'device_timestamp_rollbacks':{k:int(np.sum(np.diff(v)<0)) for k,v in timestamps.items()},
               'device_processing_latency_s':stats(processing),
               'host_minus_dvl_transmission_s':stats(clock_offset),
               'clock_synchronized':bool(clock_offset and max(abs(x) for x in clock_offset)<.05)},
        'mavlink':{k:stream_stats(rows,duration) for k,rows in mav_by_kind.items()},
        'gv7':{k:stream_stats([r for r in gv7 if r['data']['descriptor_set']==k],duration)
               for k in sorted({r['data']['descriptor_set'] for r in gv7})},
        'gv7_fields':{f'{ds}/{descriptor}':{
            'count':sum(r['data']['descriptor_set']==ds and descriptor in r['data']['fields'] for r in gv7),
            'last':next(r['data']['fields'][descriptor] for r in reversed(gv7)
                        if r['data']['descriptor_set']==ds and descriptor in r['data']['fields'])}
            for ds,descriptor in sorted({(r['data']['descriptor_set'],key) for r in gv7 for key in r['data']['fields']})},
        'receipt_timing':timing_check(dvl,attitude,cfg['state']),
        'state_execution_approved':False,
        'approval_reason':'Passive data proves availability only; installation, origin, acquisition latency and model/output verification remain required.',
    }


def main():
    p = argparse.ArgumentParser(description='只读核查 DVL / 飞控 IMU / GV7；不发送设备命令')
    p.add_argument('--duration',type=float,default=20.)
    p.add_argument('--config',type=Path,default=DEFAULT_CONFIG)
    p.add_argument('--output',type=Path,default=Path('reports/connected_validation.json'))
    a = p.parse_args()
    if not np.isfinite(a.duration) or not 1 <= a.duration <= 60:
        p.error('duration must be 1..60 s')
    cfg=load_config(a.config)
    legacy=json.loads(Path(cfg['legacy_config']).read_text())
    started=datetime.now(timezone.utc).isoformat()
    with ThreadPoolExecutor(max_workers=3) as pool:
        dv=pool.submit(tcp_capture,legacy['dvl_ip'],legacy['dvl_port'],a.duration,'dvl')
        mv=pool.submit(mav_capture,legacy['connection'],legacy['baud'],a.duration)
        gv=pool.submit(tcp_capture,'192.168.2.2',5555,a.duration,'gv7')
        dvl,dl=dv.result(); mav,ml=mv.result(); gv7,gl=gv.result()
    a.output.parent.mkdir(parents=True,exist_ok=True)
    raw=a.output.with_suffix('.raw.jsonl')
    with raw.open('w') as f:
        for name,rows in [('dvl',dvl),('mavlink',mav),('gv7',gv7)]:
            for row in rows:
                f.write(json.dumps({'link':name,**row})+'\n')
    report={'started_utc':started,'duration_s':a.duration,'read_only':True,
            'actuator_commands_sent':False,'device_commands_sent':False,
            'links':{'dvl':dl,'mavlink':ml,'gv7':gl},'raw_capture':str(raw),
            **summarize(dvl,mav,gv7,a.duration,cfg)}
    a.output.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:report[k] for k in ('started_utc','links','target_candidates','receipt_timing','state_execution_approved')},ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
