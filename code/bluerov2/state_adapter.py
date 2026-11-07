"""Qualified 12-state snapshots from the existing A50/ArduSub links; no prediction."""
from pathlib import Path
import hashlib
import time
import numpy as np
from scipy.spatial.transform import Rotation
from .dynamics import rotation_zyx
from .model import wrap_to_pi


class StatePending(Exception):
    """A new complete qualified snapshot is not available yet."""


class StateFault(Exception):
    """An explicit terminal fault; a cached snapshot cannot conceal it."""


def proper_rotation(value):
    r = np.asarray(value, float).reshape(3, 3)
    if not np.all(np.isfinite(r)) or not np.allclose(r.T@r,np.eye(3),atol=1e-8) or not np.isclose(np.linalg.det(r),1):
        raise ValueError('state transform must be a proper rotation')
    return r


class StateAdapter:
    def __init__(self, feed, controller, origin, origin_file, config):
        if config.get('frame_verified') is not True:
            raise ValueError('state frame, velocity convention and installation have not been verified')
        self.feed, self.controller, self.config = feed, controller, dict(config)
        self.origin = np.asarray(origin['origin'], float).reshape(3).copy()
        self.lever = np.asarray(config['body_to_dvl_m'], float).reshape(3).copy()
        if not np.all(np.isfinite(self.origin)) or not np.all(np.isfinite(self.lever)):
            raise ValueError('origin and DVL lever must be finite')
        self.world_rotation = proper_rotation(config['world_from_dvl_world']).copy()
        self.body_rotation = proper_rotation(config['body_from_dvl_sensor']).copy()
        self.feed_inverse = proper_rotation(feed.matrix).T.copy()
        self.velocity_frame = config['velocity_frame']
        if self.velocity_frame not in ('sensor','dvl_world'):
            raise ValueError('velocity_frame must be sensor or dvl_world')
        for name in ('max_age_s','max_receipt_skew_s','max_position_std_m',
                     'position_jump_margin_m','attitude_jump_margin_rad',
                     'max_plausible_speed_m_s','max_plausible_rate_rad_s'):
            value = config[name]
            if not np.isfinite(value) or value <= 0:
                raise ValueError('invalid state.'+name)
        self.reference_period = config.get('reference_check_period_s', 1.)
        if not np.isfinite(self.reference_period) or self.reference_period <= 0:
            raise ValueError('reference_check_period_s must be positive')
        self.origin_file = Path(origin_file)
        self.reference_digest = hashlib.sha256(self.origin_file.read_bytes()).hexdigest()
        self.next_reference_check = time.monotonic()+self.reference_period
        self.cache = None
        self.previous = None
        self.fault = None
        self.device_progress = {}
        self.parsed = {}
        self.last_candidate_key = None
        self.last_pending = None
        self.last_read_mono = None

    def _check_reference(self, now):
        if now < self.next_reference_check:
            return
        self.next_reference_check = now+self.reference_period
        try:
            digest = hashlib.sha256(self.origin_file.read_bytes()).hexdigest()
        except OSError as exc:
            raise StateFault('task reference unavailable: '+str(exc)) from exc
        if digest != self.reference_digest:
            raise StateFault('task reference changed')

    def _device_receipt(self, name, stamp, received, now):
        if not np.isfinite(stamp) or stamp < 0 or not np.isfinite(received) or not 0 <= received <= now:
            raise StateFault('invalid '+name+' device/receipt timestamp')
        previous = self.device_progress.get(name)
        if previous is not None and stamp < previous[0]:
            raise StateFault(name+' device timestamp rollback/reboot')
        if previous is not None and received < previous[1]:
            raise StateFault(name+' receipt timestamp rollback')
        # Repeated device samples cannot acquire a new age through a new receipt.
        if previous is None or stamp > previous[0]:
            self.device_progress[name] = (stamp, received)
        return min(received, self.device_progress[name][1])

    def _pose(self, record, now):
        received, raw, _, _ = record
        position = np.asarray(raw.position,float).reshape(3)
        angles = np.deg2rad(np.asarray(raw.attitude,float).reshape(3))
        std = float(raw.position_std)
        if not np.all(np.isfinite(np.r_[position,angles,std,raw.status])) or std < 0:
            raise StateFault('nonfinite/invalid DVL pose or uncertainty')
        if raw.raw_time is None:
            raise StatePending('missing DVL device timestamp')
        stamp = float(raw.raw_time)
        receipt = self._device_receipt('dvl_dead_reckoning',stamp,float(received),now)
        world_body = self.world_rotation@rotation_zyx(*angles)@self.body_rotation.T
        angles = Rotation.from_matrix(world_body).as_euler('ZYX')[::-1]
        position = self.world_rotation@position-self.origin-world_body@self.lever
        if self.cache is not None:
            dt = max(receipt-self.cache['receipt_mono']['position_attitude'],0.)
            old = self.cache['state']
            cfg = self.config
            if np.linalg.norm(position-old[:3]) > cfg['position_jump_margin_m']+dt*cfg['max_plausible_speed_m_s']:
                raise StateFault('DVL position/reference discontinuity')
            if np.linalg.norm(wrap_to_pi(angles-old[3:6])) > cfg['attitude_jump_margin_rad']+dt*cfg['max_plausible_rate_rad_s']:
                raise StateFault('DVL attitude/reference discontinuity')
        return {'position':position,'angles':angles,'world_body':world_body,
                'receipt':receipt,'device':stamp,'std':std,'status':raw.status}

    def _gyro(self, record, now):
        received, msg = record
        stamp = float(msg.time_boot_ms)
        receipt = self._device_receipt('flight_attitude',stamp,float(received),now)
        rates = np.array([msg.rollspeed,msg.pitchspeed,msg.yawspeed],float)
        if not np.all(np.isfinite(rates)):
            raise StateFault('nonfinite body angular rates')
        return {'rates':rates,'receipt':receipt,'device':stamp}

    def _velocity(self, record, metadata, now):
        received, transformed = record
        vector = np.asarray(transformed,float).reshape(3)
        if not np.all(np.isfinite(vector)):
            raise StateFault('nonfinite DVL velocity')
        if not isinstance(metadata,dict) or any(metadata.get(k) is None for k in ('time_of_validity','time_of_transmission')):
            raise StatePending('missing DVL velocity acquisition/transmission timestamp')
        validity = float(metadata['time_of_validity'])
        transmission = float(metadata['time_of_transmission'])
        receipt = self._device_receipt('dvl_velocity',validity,float(received),now)
        if validity <= 0 or not np.isfinite(transmission) or transmission < validity:
            raise StateFault('invalid DVL velocity device timestamps')
        # Check the transmission clock too; it is independent of local receive time.
        self._device_receipt('dvl_transmission',transmission,float(received),now)
        return {'native':self.feed_inverse@vector,'receipt':receipt,'device':validity,
                'transmission':transmission,'processing_s':(transmission-validity)/1e6}

    def _observe(self, now):
        dr = self.feed.latest
        velocity = self.feed.latest_velocity
        attitude = self.controller.state_messages.get('ATTITUDE')
        metadata = getattr(self.feed,'latest_velocity_metadata',None)
        observations = {}; keys = []; pending = []
        # Every available device is inspected before a temporary incomplete bundle is used.
        # Thus, a missing velocity report cannot hide a flight-controller time rollback.
        for name,record,parser in (('pose',dr,self._pose),('gyro',attitude,self._gyro),
                                   ('velocity',velocity,lambda r,n:self._velocity(r,metadata,n))):
            if record is None:
                pending.append('missing '+('body angular rates: ATTITUDE' if name=='gyro' else name))
                keys.append(None);continue
            key = (id(record), record[0])
            if name=='pose':key += (record[1].raw_time,)
            if name=='gyro':key += (record[1].time_boot_ms,)
            if name=='velocity':key += (None if metadata is None else metadata.get('time_of_validity'),
                                       None if metadata is None else metadata.get('time_of_transmission'))
            keys.append(key)
            try:
                if name not in self.parsed or self.parsed[name][0] != key:
                    parsed = parser(record,now)
                    self.parsed[name] = (key,parsed)
                observations[name] = self.parsed[name][1]
            except StatePending as exc:
                pending.append(str(exc))
        quality = (self.feed.velocity_valid,self.feed.beams_valid)
        return observations, (tuple(keys),quality), pending, quality

    def _candidate(self, observations, pending, quality, now):
        if pending:
            raise StatePending('; '.join(pending))
        p,v,g = (observations[k] for k in ('pose','velocity','gyro'))
        cfg = self.config
        if p['status'] != 0 or p['std'] > cfg['max_position_std_m'] or not all(quality):
            raise StatePending('DVL pose/velocity/beam quality is not qualified')
        receipts = dict(position_attitude=p['receipt'],velocity=v['receipt'],gyro=g['receipt'])
        ages = [now-x for x in receipts.values()]
        age = max(*ages, now-v['receipt']+v['processing_s'])
        if age >= cfg['max_age_s']:
            raise StatePending('state data expired or device timestamp stopped advancing')
        skew = max(receipts.values())-min(receipts.values())
        if skew > cfg['max_receipt_skew_s']:
            raise StatePending('state component receipt time skew exceeded')
        if abs(p['device']-v['device']/1e6) > cfg['max_receipt_skew_s']:
            raise StatePending('DVL pose/velocity acquisition time skew exceeded')
        velocity = self.body_rotation@v['native'] if self.velocity_frame=='sensor' else p['world_body'].T@self.world_rotation@v['native']
        velocity = velocity-np.cross(g['rates'],self.lever)
        state = np.r_[p['position'],p['angles'],velocity,g['rates']]
        if not np.all(np.isfinite(state)):
            raise StateFault('nonfinite state component')
        state.setflags(write=False)
        return {'state':state,'timestamp':now,'receipt_mono':receipts,
                'receipt_skew_s':skew,'processing_s':v['processing_s'],
                'device_timestamp':{'dvl':p['device'],'flight_boot_ms':g['device'],
                                    'velocity_validity_us':v['device'],'velocity_transmission_us':v['transmission']},
                'quality':{'position_std_m':p['std'],'position_status':p['status'],
                           'velocity_valid':bool(quality[0]),'beams_valid':bool(quality[1])},
                'reference_id':self.reference_digest}

    def _cache_age(self, now):
        c = self.cache
        ages = {k:now-v for k,v in c['receipt_mono'].items()}
        velocity_age = ages['velocity']+c['processing_s']
        age = max(*ages.values(),velocity_age)
        if min(ages.values()) < 0 or age >= self.config['max_age_s']:
            raise StateFault('qualified state cache expired')
        return ages, velocity_age, age

    def _cached(self, now, reused):
        c = self.cache
        ages, velocity_age, age = self._cache_age(now)
        return {'state':c['state'].copy(),'valid':True,'status':'cached' if reused else 'new',
                'cached':reused,'age':float(age),'timestamp':c['timestamp'],
                'receipt_mono':dict(c['receipt_mono']),'component_age_s':ages,
                'receipt_skew_s':c['receipt_skew_s'],'device_timestamp':dict(c['device_timestamp']),
                'quality':dict(c['quality']),'reference_id':c['reference_id'],
                'velocity_acquisition_age_lower_bound_s':velocity_age,
                'candidate_pending_reason':self.last_pending,
                'age_basis':'original host receipts plus known DVL velocity processing; unknown transport/pose latency excluded; lower bound only'}

    def _invalid(self, status, reason, now):
        return {'state':np.full(12,np.nan),'valid':False,'status':status,'age':float('inf'),
                'timestamp':now,'reason':reason,'reference_id':self.reference_digest}

    def read_state(self, *, service_io=True):
        now = time.monotonic()
        if self.fault is not None:
            return self._invalid('fault',self.fault,now)
        try:
            if self.last_read_mono is not None and now < self.last_read_mono:
                raise StateFault('host monotonic clock rollback')
            self.last_read_mono = now
            if service_io:self.controller.service_io()
            now = time.monotonic()
            self._check_reference(now)
            if not self.controller.vehicle.heartbeat_fresh(now,self.controller.heartbeat_timeout):
                raise StateFault('flight controller heartbeat expired')
            if getattr(self.controller,'reboot_detected',False):
                raise StateFault('flight controller reboot detected')
            if hasattr(self.feed,'sock') and self.feed.sock is None:
                raise StateFault('DVL connection lost; no reconnect during a task')
            self.feed.poll()
            observations,key,pending,quality = self._observe(time.monotonic())
            now = time.monotonic()
            # Once an existing cache expires, a newly arriving sample cannot revive the task.
            if self.cache is not None:self._cache_age(now)
            reused = True
            if key != self.last_candidate_key:
                self.last_candidate_key = key
                try:
                    candidate = self._candidate(observations,pending,quality,now)
                except StatePending as exc:
                    self.last_pending = str(exc)
                else:
                    self.cache = candidate  # atomic commit only after all checks pass
                    self.previous = (candidate['timestamp'],candidate['state'])
                    self.last_pending = None
                    reused = False
            if self.cache is None:
                return self._invalid('waiting',self.last_pending or 'waiting for a complete qualified state',now)
            return self._cached(now,reused)
        except StateFault as exc:
            self.fault = str(exc)
        except Exception as exc:
            # Malformed data, configuration and programming errors must never mean temporary loss.
            self.fault = type(exc).__name__+': '+str(exc)
        return self._invalid('fault',self.fault,time.monotonic())
