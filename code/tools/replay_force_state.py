"""Replay an existing read-only capture; no sockets or actuator connections are opened."""
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from bluerov2.force_run import DEFAULT_CONFIG, load_config
from bluerov2.host_tracking import load_backend
from bluerov2.state_adapter import StateAdapter


def stats(values):
    a = np.asarray(values, float)
    if not len(a):return None
    return dict(count=len(a), min=float(a.min()), median=float(np.median(a)),
                p95=float(np.percentile(a, 95)), max=float(a.max()))


class ReplaySocket:
    """Original parser receives one report at its recorded host time."""
    def __init__(self):self.chunk = None
    def recv(self, _):
        if self.chunk is None:raise BlockingIOError
        chunk, self.chunk = self.chunk, None
        return chunk


def replay(raw_path, config_path=DEFAULT_CONFIG, poll_s=.005):
    if not np.isfinite(poll_s) or poll_s <= 0:raise ValueError('poll_s must be positive')
    raw_path = Path(raw_path); cfg = load_config(config_path)
    legacy = json.loads(Path(cfg['legacy_config']).read_text()); backend = load_backend(cfg['legacy_directory'])
    origin_file = Path(legacy['origin_file'])
    if not origin_file.is_absolute():origin_file = Path(cfg['legacy_config']).parent/origin_file
    origin = json.loads(origin_file.read_text())
    all_rows = [json.loads(line) for line in raw_path.read_text().splitlines() if line.strip()]
    dvl = [r for r in all_rows if r['link'] == 'dvl']
    mav = [r for r in all_rows if r['link'] == 'mavlink' and r.get('source') == [1, 1]
           and r['data']['mavpackettype'] in ('ATTITUDE', 'HEARTBEAT')]
    heartbeats = [r for r in mav if r['data']['mavpackettype'] == 'HEARTBEAT']
    if not heartbeats:raise ValueError('capture has no target heartbeat')
    events = sorted(dvl+mav, key=lambda r:r['monotonic_s'])
    start = min(r['monotonic_s'] for r in heartbeats); end = max(r['monotonic_s'] for r in events)
    sock = ReplaySocket()
    feed = backend.DvlFeed('offline-replay', 0, legacy['dvl_to_task_ned_rotation'],
                           legacy['yaw_offset_rad'], legacy['yaw_sign']); feed.sock = sock
    clock = SimpleNamespace(now=start); last_hb = [float('-inf')]
    ctrl = SimpleNamespace(state_messages={}, service_io=lambda: None, heartbeat_timeout=3., reboot_detected=False,
        vehicle=SimpleNamespace(heartbeat_fresh=lambda now, timeout: 0 <= now-last_hb[0] <= timeout))
    status_counts = Counter(); candidate_counts = Counter(); transitions = []; previous_status = None
    candidate_age = []; candidate_receipt_skew = []; candidate_acquisition_skew = []
    qualified_ages = []; first_valid = None; first_fault = None; index = 0
    def apply(row):
        clock.now = row['monotonic_s']; data = row['data']
        if row['link'] == 'dvl':
            sock.chunk = (json.dumps(data)+'\n').encode(); feed.poll()
        elif data['mavpackettype'] == 'HEARTBEAT':last_hb[0] = row['monotonic_s']
        else:ctrl.state_messages['ATTITUDE'] = (row['monotonic_s'], SimpleNamespace(**data))
    with patch('time.monotonic', lambda:clock.now):
        while index < len(events) and events[index]['monotonic_s'] <= start:
            apply(events[index]); index += 1
        clock.now = start
        # This permits numerical replay only, never certifies the physical frame.
        adapter = StateAdapter(feed, ctrl, origin, origin_file, {**cfg['state'], 'frame_verified':True})
        for tick in np.arange(start, end+poll_s/2, poll_s):
            while index < len(events) and events[index]['monotonic_s'] <= tick:
                apply(events[index]); index += 1
            clock.now = float(tick)
            if feed.latest and feed.latest_velocity and 'ATTITUDE' in ctrl.state_messages:
                receipts = [feed.latest[0], feed.latest_velocity[0], ctrl.state_messages['ATTITUDE'][0]]
                metadata = feed.latest_velocity_metadata
                latency = (metadata['time_of_transmission']-metadata['time_of_validity'])/1e6
                age = max(*(tick-r for r in receipts), tick-feed.latest_velocity[0]+latency)
                skew = max(receipts)-min(receipts)
                acquisition_skew = abs(feed.latest[1].raw_time-metadata['time_of_validity']/1e6)
                candidate_age.append(age); candidate_receipt_skew.append(skew); candidate_acquisition_skew.append(acquisition_skew)
                if age >= cfg['state']['max_age_s']:candidate_counts['age_exceeded'] += 1
                elif skew > cfg['state']['max_receipt_skew_s']:candidate_counts['receipt_skew_exceeded'] += 1
                elif acquisition_skew > cfg['state']['max_receipt_skew_s']:candidate_counts['acquisition_skew_exceeded'] += 1
                else:candidate_counts['time_gates_only_eligible'] += 1
            sample = adapter.read_state(); status_counts[sample['status']] += 1
            if sample['valid']:
                qualified_ages.append(sample['age'])
                if first_valid is None:first_valid = dict(elapsed_s=float(tick-start), age_s=sample['age'])
            if sample['status'] == 'fault' and first_fault is None:
                first_fault = dict(elapsed_s=float(tick-start), reason=sample['reason'],
                    receipt_mono=None if adapter.cache is None else adapter.cache['receipt_mono'])
            status = (sample['status'], sample.get('reason'), sample.get('candidate_pending_reason'))
            if status != previous_status:
                transitions.append(dict(elapsed_s=float(tick-start), status=status[0], reason=status[1], pending=status[2]))
                previous_status = status
    streams = {'pose':[r for r in dvl if 'ts' in r['data']],
               'velocity':[r for r in dvl if 'velocity_valid' in r['data']],
               'gyro':[r for r in mav if r['data']['mavpackettype'] == 'ATTITUDE']}
    latency = [(r['data']['time_of_transmission']-r['data']['time_of_validity'])/1e6 for r in streams['velocity']]
    return {'created_utc':datetime.now(timezone.utc).isoformat(), 'input':str(raw_path),
        'input_sha256':hashlib.sha256(raw_path.read_bytes()).hexdigest(),
        'adapter_sha256':hashlib.sha256((ROOT/'bluerov2/state_adapter.py').read_bytes()).hexdigest(),
        'offline_only':True, 'physical_vehicle_used':False, 'actuator_commands_sent':False,
        'installation_certified':False, 'frame_verified_override_scope':'in-memory numerical replay only; saved config unchanged',
        'poll_period_s':poll_s, 'observation_duration_s':end-start,
        'source_receipt_gap_s':{name:stats(np.diff([r['monotonic_s'] for r in rows])) for name, rows in streams.items()},
        'known_velocity_processing_s':stats(latency),
        'candidate_time_diagnostics_entire_capture':{'counts':dict(candidate_counts), 'age_s':stats(candidate_age),
            'receipt_skew_s':stats(candidate_receipt_skew), 'dvl_acquisition_skew_s':stats(candidate_acquisition_skew),
            'scope':'time gates of latest raw components, not cached task success or coordinate certification'},
        'single_task_current_adapter':{'counts':dict(status_counts), 'first_valid':first_valid, 'first_fault':first_fault,
            'qualified_age_s':stats(qualified_ages), 'transitions':transitions, 'automatically_restarted':False},
        'limitations':['Historical capture; no new sensor or vehicle measurement.',
            'Original handler receipt times are replayed; unknown transport/pose latency remains unmeasured.',
            'Independent FC and DVL acquisition clocks are not synchronized by this replay.',
            'Original task origin is used for numerical replay only; no physical reset/frame acceptance implied.']}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('capture', type=Path)
    p.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    p.add_argument('--output', type=Path, default=Path('reports/state_replay.json'))
    args = p.parse_args(); report = replay(args.capture, args.config)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({'report':str(args.output), 'first_fault':report['single_task_current_adapter']['first_fault']}, ensure_ascii=False))
    return 0

if __name__ == '__main__':raise SystemExit(main())
