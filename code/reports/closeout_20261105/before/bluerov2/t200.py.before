"""Offline, asymmetric T200 static thrust lookup. Forces are on the vehicle, in N."""
from pathlib import Path
import json
import numpy as np

DATA = Path(__file__).with_name('data')/'t200_curves.json'


def vector6(value, name):
    a = np.asarray(value, dtype=float)
    if a.shape != (6,) or not np.all(np.isfinite(a)):
        raise ValueError(f'{name} must be a finite six-element vector')
    return a


class T200:
    def __init__(self, mapping, path=DATA):
        self.mapping = mapping
        if len(mapping) != 6 or sorted(m['model_index'] for m in mapping) != list(range(6)) or sorted(m['motor_id'] for m in mapping) != list(range(1,7)) or sorted(m['output_channel'] for m in mapping) != list(range(1,7)):
            raise ValueError('mapping must biject model indices 0..5, motors 1..6 and physical channels 1..6')
        self.mapping = sorted(mapping, key=lambda m: m['model_index'])
        for m in self.mapping:
            if m['model_to_curve_sign'] not in (-1, 1):
                raise ValueError('each model_to_curve_sign must be explicitly +1 or -1')
        data = json.loads(Path(path).read_text())
        if data['format'] != 't200-static-v1' or data['neutral_us'] != 1500:
            raise ValueError('unsupported T200 data/neutral')
        self.voltages = np.asarray([t['voltage_v'] for t in data['tables']])
        self.pwm = np.asarray(data['tables'][0]['pwm_us'])
        self.force = np.asarray([t['force_n'] for t in data['tables']])
        if not np.all(np.isfinite(self.force)) or np.any(np.diff(self.force, axis=1) < 0) or not np.all(np.diff(self.voltages)>0):
            raise ValueError('invalid processed curves')

    def curve(self, voltage_v):
        if not np.isfinite(voltage_v) or not self.voltages[0] <= voltage_v <= self.voltages[-1]:
            raise ValueError('voltage outside official 10..20 V calibration; extrapolation forbidden')
        hi = min(int(np.searchsorted(self.voltages, voltage_v, side='right')), len(self.voltages)-1)
        lo = max(0, hi-1)
        a = (voltage_v-self.voltages[lo])/(self.voltages[hi]-self.voltages[lo])
        return (1-a)*self.force[lo] + a*self.force[hi]

    def limits(self, voltage_v):
        f = self.curve(voltage_v)
        lo, hi = np.empty(6), np.empty(6)
        for i, m in enumerate(self.mapping):
            lo[i], hi[i] = sorted((m['model_to_curve_sign']*f[0], m['model_to_curve_sign']*f[-1]))
        return lo, hi

    def thrust_to_pwm(self, force_n, voltage_v, thruster_id):
        if not np.isfinite(force_n):
            raise ValueError('nonfinite thrust')
        m = next((m for m in self.mapping if m['motor_id'] == thruster_id), None)
        if m is None:
            raise ValueError('thruster_id must be a configured factory motor number')
        curve = self.curve(voltage_v)
        force = float(force_n)*m['model_to_curve_sign']
        if force == 0:
            return 1500
        if force < curve[0]-1e-9 or force > curve[-1]+1e-9:
            raise ValueError('requested thrust exceeds T200 hardware capability')
        reverse = force < 0
        indices = np.flatnonzero(self.pwm <= 1500)[::-1] if reverse else np.flatnonzero(self.pwm >= 1500)
        magnitude = -curve[indices] if reverse else curve[indices]
        # Anchor nonzero interpolation at the outer deadband edge, never at neutral.
        zero = np.flatnonzero(magnitude == 0)
        start = int(zero[-1])
        fs, ps = [0.], [self.pwm[indices[start]]]
        for f, p in zip(magnitude[start+1:], self.pwm[indices[start+1:]]):
            if f > fs[-1]:
                fs.append(f); ps.append(p)
        return int(np.rint(np.interp(abs(force), fs, ps)))

    def map_forces(self, forces_n, voltage_v, hardware_clip_tolerance_n=.05):
        requested = vector6(forces_n, 'forces_n')
        lo, hi = self.limits(voltage_v)
        adopted = np.clip(requested, lo, hi)
        if np.max(np.abs(requested-adopted)) > hardware_clip_tolerance_n:
            raise ValueError('requested thrust exceeds hardware capability; refusing to mask an infeasible plan')
        pwm, estimated = np.empty(6, int), np.empty(6)
        curve = self.curve(voltage_v)
        for i, m in enumerate(self.mapping):
            p = self.thrust_to_pwm(adopted[i], voltage_v, m['motor_id'])
            pwm[m['output_channel']-1] = p
            estimated[i] = m['model_to_curve_sign']*np.interp(p, self.pwm, curve)
        return {'requested_force_n': requested.copy(), 'adopted_force_n': adopted,
                'static_estimated_force_n': estimated, 'pwm_us': pwm,
                'hardware_saturated': np.abs(requested-adopted)>1e-10,
                'voltage_v': float(voltage_v)}
