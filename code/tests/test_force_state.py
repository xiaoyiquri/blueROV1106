from pathlib import Path
from types import SimpleNamespace
import time
import numpy as np
import pytest
from bluerov2.force_run import load_config, DEFAULT_CONFIG, wait_initial_state
from bluerov2.state_adapter import StateAdapter
from bluerov2.dynamics import rotation_zyx
from bluerov2.host_tracking import load_backend, make_feed
from bluerov2.thruster_output import controller_class


@pytest.fixture
def state_link(monkeypatch, tmp_path):
    clock = SimpleNamespace(now=10.)
    monkeypatch.setattr(time, 'monotonic', lambda: clock.now)
    monkeypatch.setattr(time, 'sleep', lambda dt: setattr(clock, 'now', clock.now+dt))
    cfg = load_config(DEFAULT_CONFIG)['state']; cfg['frame_verified'] = True
    origin = tmp_path/'origin.json'; origin.write_text('{}')
    raw = SimpleNamespace(position=[1., 2., 3.], attitude=[10., -20., 90.],
                          raw_time=100., position_std=.02, status=0)
    feed = SimpleNamespace(matrix=np.eye(3).ravel().tolist(), latest=(10., raw, raw.position, np.pi/2),
        latest_velocity=(10., np.array([.2, .1, -.05])), velocity_valid=True, beams_valid=True, poll=lambda: None,
        latest_velocity_metadata={'time_of_validity':100000000, 'time_of_transmission':100100000})
    msg = SimpleNamespace(time_boot_ms=5000, rollspeed=.1, pitchspeed=.2, yawspeed=.3)
    c = SimpleNamespace(service_io=lambda: None, state_messages={'ATTITUDE':(10., msg)}, heartbeat_timeout=3.,
                        vehicle=SimpleNamespace(heartbeat_fresh=lambda *_: True))
    adapter = StateAdapter(feed, c, {'origin':[.5, 1., 1.]}, origin, cfg)
    return adapter, feed, c, clock, origin


def publish(feed, ctrl, clock, **changes):
    """Publish new source records as the real parser does; keep receipt history honest."""
    raw = SimpleNamespace(**vars(feed.latest[1]))
    raw.raw_time = 100.+clock.now-10.
    for name, value in changes.items():setattr(raw, name, value)
    feed.latest = (clock.now, raw, raw.position, np.deg2rad(raw.attitude[2]))
    feed.latest_velocity = (clock.now, feed.latest_velocity[1].copy())
    stamp = round((100.+clock.now-10.)*1e6)
    feed.latest_velocity_metadata = {'time_of_validity':stamp, 'time_of_transmission':stamp+100000}
    old = ctrl.state_messages.get('ATTITUDE', (None, None))[1]
    msg = SimpleNamespace(**vars(old)) if old else SimpleNamespace(rollspeed=.1, pitchspeed=.2, yawspeed=.3)
    msg.time_boot_ms = round(5000+(clock.now-10.)*1000)
    ctrl.state_messages['ATTITUDE'] = (clock.now, msg)


def test_complete_state_units_and_body_velocity_no_double_rotation(state_link):
    a, f, c, clock, _ = state_link; s = a.read_state()
    assert s['valid'] and s['age'] == .1 and s['receipt_skew_s'] == 0
    np.testing.assert_allclose(s['state'][:3], [.5, 1., 2.])
    np.testing.assert_allclose(s['state'][3:6], np.deg2rad([10, -20, 90]), atol=1e-14)
    np.testing.assert_allclose(s['state'][6:], [.2, .1, -.05, .1, .2, .3])


def test_world_velocity_rotated_once_with_full_attitude(state_link):
    a, f, c, _, path = state_link
    a = StateAdapter(f, c, {'origin':[.5, 1., 1.]}, path, {**a.config, 'velocity_frame':'dvl_world'})
    s = a.read_state(); r = rotation_zyx(*s['state'][3:6])
    np.testing.assert_allclose(r@s['state'][6:9], f.latest_velocity[1], atol=1e-14)


def test_independent_installation_and_lever_arm_transform(state_link):
    a, f, c, _, path = state_link
    a = StateAdapter(f, c, {'origin':[.5, 1., 1.]}, path,
        {**a.config, 'body_from_dvl_sensor':rotation_zyx(0, 0, np.pi/2), 'body_to_dvl_m':[.1, 0, 0]})
    s = a.read_state(); assert s['valid']
    expected = a.body_rotation@f.latest_velocity[1]-np.cross([.1, .2, .3], a.lever)
    np.testing.assert_allclose(s['state'][6:9], expected)


def test_missing_rates_are_waiting_not_zero_filled(state_link):
    a, f, c, _, _ = state_link; c.state_messages = {}; s = a.read_state()
    assert not s['valid'] and s['status'] == 'waiting' and a.fault is None
    assert np.all(np.isnan(s['state'])) and 'missing' in s['reason']


@pytest.mark.parametrize('change', ['skew', 'expired'])
def test_stale_component_and_time_skew_before_first_cache(state_link, change):
    a, f, c, clock, _ = state_link
    if change == 'skew':c.state_messages['ATTITUDE'] = (9.8, c.state_messages['ATTITUDE'][1])
    else:clock.now = 10.3
    s = a.read_state()
    assert not s['valid'] and s['status'] == 'waiting' and change in s['reason']
    assert a.cache is None and a.fault is None


@pytest.mark.parametrize('change', ['origin', 'fc_boot', 'dvl_time', 'position', 'attitude'])
def test_reference_reboot_and_jump_are_latched(state_link, change):
    a, f, c, clock, path = state_link; assert a.read_state()['valid']
    if change == 'origin':
        for i in range(1, 10):
            clock.now = 10.+i*.1; publish(f, c, clock); assert a.read_state()['valid']
        path.write_text('{"new":1}'); clock.now = 11.; publish(f, c, clock)
    else:
        clock.now += .01; publish(f, c, clock)
        if change == 'fc_boot':c.state_messages['ATTITUDE'][1].time_boot_ms = 1
        if change == 'dvl_time':f.latest[1].raw_time = 1
        if change == 'position':f.latest[1].position = [20, 0, 0]
        if change == 'attitude':f.latest[1].attitude = [10, -20, -20]
    assert a.read_state()['status'] == 'fault' and a.fault
    path.write_text('{}'); publish(f, c, clock)
    assert not a.read_state()['valid']


def test_real_legacy_controller_subclass_filters_source_and_never_manual(tmp_path):
    backend = load_backend(Path(__file__).resolve().parents[2]); logger = backend.SessionLogger(tmp_path)
    c = controller_class(backend)('fake', 115200, 255, 191, 1, 1, logger)
    c.vehicle.target_system = c.vehicle.target_component = 1
    class Msg:
        def get_type(self):return 'ATTITUDE'
        def get_srcSystem(self):return 1
        def get_srcComponent(self):return 1
        def to_dict(self):return dict(roll=.1, pitch=.2, yaw=.3, rollspeed=.4, pitchspeed=.5, yawspeed=.6)
        def __getattr__(self, k):return self.to_dict()[k]
    try:
        c.process_message(Msg()); assert 'ATTITUDE' in c.state_messages
        with pytest.raises(RuntimeError, match='MANUAL_CONTROL'):c.send_manual((0, 0, 500, 0))
    finally:logger.close()


def test_replayed_device_timestamp_is_not_fresh_even_with_new_receipts(state_link):
    a, f, c, clock, _ = state_link; first = a.read_state(); assert first['valid']
    clock.now += .05
    f.latest = (clock.now, *f.latest[1:]); f.latest_velocity = (clock.now, f.latest_velocity[1])
    c.state_messages['ATTITUDE'] = (clock.now, c.state_messages['ATTITUDE'][1])
    sample = a.read_state()
    assert sample['valid'] and sample['receipt_mono'] == first['receipt_mono']
    assert sample['age'] == pytest.approx(.15)
    clock.now = 10.15
    sample = a.read_state()
    assert not sample['valid'] and 'expired' in sample['reason']


def test_unverified_frame_cannot_produce_valid_state(state_link):
    a, f, c, clock, path = state_link
    with pytest.raises(ValueError, match='not been verified'):
        StateAdapter(f, c, {'origin':[0, 0, 0]}, path, {**a.config, 'frame_verified':False})


@pytest.mark.parametrize('change', ['missing', 'replay', 'rollback', 'latency', 'skew'])
def test_velocity_device_age_and_progress_required(state_link, change):
    a, f, c, clock, _ = state_link; first = a.read_state(); assert first['valid']
    clock.now += .01
    if change == 'missing':f.latest_velocity_metadata = None
    elif change == 'latency':f.latest_velocity_metadata = {**f.latest_velocity_metadata, 'time_of_transmission':100400000}
    elif change == 'skew':
        publish(f, c, clock); f.latest[1].raw_time += .2
    elif change == 'rollback':f.latest_velocity_metadata = {**f.latest_velocity_metadata, 'time_of_validity':99999999}
    else:clock.now += .26
    sample = a.read_state()
    if change in ('rollback', 'replay'):
        assert not sample['valid'] and a.fault
    else:
        assert sample['valid'] and sample['cached'] and a.fault is None
        np.testing.assert_array_equal(sample['state'], first['state'])
        assert sample['timestamp'] == first['timestamp'] and sample['candidate_pending_reason']
        clock.now = 10.151; assert not a.read_state()['valid'] and a.fault


def test_fast_reads_reuse_cache_and_original_timestamps(state_link):
    a, f, c, clock, _ = state_link; first = a.read_state()
    for i in range(1, 20):
        clock.now = 10.+i*.005; s = a.read_state()
        assert s['status'] == 'cached' and s['age'] == pytest.approx(.1+i*.005)
        for field in ('timestamp', 'receipt_mono', 'device_timestamp', 'quality', 'reference_id'):
            assert s[field] == first[field]
    assert a.fault is None


def test_ten_hz_sources_can_be_polled_at_two_hundred_hz(state_link):
    a, f, c, clock, _ = state_link; updates = 0; reuses = 0
    for i in range(200):
        clock.now = 10.+i*.005
        if i % 20 == 0:publish(f, c, clock)
        s = a.read_state(); assert s['valid']
        updates += s['status'] == 'new'; reuses += s['status'] == 'cached'
    assert updates == 10 and reuses == 190


@pytest.mark.parametrize('change', ['missing_pose', 'missing_velocity', 'missing_gyro', 'quality', 'uncertainty'])
def test_pending_candidate_uses_qualified_cache_atomically(state_link, change):
    a, f, c, clock, _ = state_link; first = a.read_state(); clock.now += .02
    publish(f, c, clock, position=[1.01, 2., 3.])
    if change == 'missing_pose':f.latest = None
    if change == 'missing_velocity':f.latest_velocity = None
    if change == 'missing_gyro':c.state_messages = {}
    if change == 'quality':f.beams_valid = False
    if change == 'uncertainty':f.latest[1].position_std = 6.
    s = a.read_state()
    assert s['valid'] and s['cached'] and s['age'] > first['age']
    np.testing.assert_array_equal(a.cache['state'], first['state'])
    assert a.cache['receipt_mono'] == first['receipt_mono']


def test_new_qualified_snapshot_replaces_entire_cache_and_copies_do_not_alias(state_link):
    a, f, c, clock, _ = state_link; first = a.read_state(); first['state'][0] = 999.
    clock.now += .02; publish(f, c, clock, position=[1.01, 2., 3.])
    s = a.read_state(); assert s['status'] == 'new'
    assert s['state'][0] == pytest.approx(.51) and s['timestamp'] == clock.now
    assert set(s['receipt_mono'].values()) == {clock.now}
    s['receipt_mono']['gyro'] = 0.
    assert a.read_state()['receipt_mono']['gyro'] == clock.now


def test_no_partial_cache_commit_before_sync_recovers(state_link):
    a, f, c, clock, _ = state_link; first = a.read_state(); clock.now += .02
    publish(f, c, clock, position=[1.01, 2., 3.])
    f.latest_velocity_metadata = {'time_of_validity':100190000, 'time_of_transmission':100290000}
    f.latest[1].raw_time = 100.36
    pending = a.read_state(); assert pending['valid'] and pending['cached']
    np.testing.assert_array_equal(pending['state'], first['state'])
    clock.now = 10.04; publish(f, c, clock); f.latest[1].raw_time = 100.37
    f.latest_velocity_metadata = {'time_of_validity':100370000, 'time_of_transmission':100470000}
    s = a.read_state(); assert s['valid'] and s['status'] == 'new'


@pytest.mark.parametrize('fault', ['nan_pose', 'nan_rate', 'nan_velocity', 'device_reboot', 'monotonic', 'program_error', 'disconnect'])
def test_explicit_fault_never_uses_cache_and_stays_latched(state_link, fault):
    a, f, c, clock, _ = state_link; assert a.read_state()['valid']; clock.now += .01; publish(f, c, clock)
    if fault == 'nan_pose':f.latest[1].position = [np.nan, 2., 3.]
    if fault == 'nan_rate':c.state_messages['ATTITUDE'][1].yawspeed = np.inf
    if fault == 'nan_velocity':f.latest_velocity[1][0] = np.nan
    if fault == 'device_reboot':c.reboot_detected = True
    if fault == 'monotonic':clock.now = 9.
    if fault == 'program_error':
        def broken():raise AssertionError('parse bug')
        f.poll = broken
    if fault == 'disconnect':f.sock = None
    s = a.read_state(); assert not s['valid'] and s['status'] == 'fault' and a.fault
    if fault == 'program_error':assert 'AssertionError: parse bug' in s['reason']
    f.poll = lambda: None; c.reboot_detected = False; clock.now = 10.02
    publish(f, c, clock); assert not a.read_state()['valid']


def test_missing_component_cannot_hide_other_device_rollback(state_link):
    a, f, c, clock, _ = state_link; assert a.read_state()['valid']; clock.now += .01
    publish(f, c, clock); f.latest_velocity = None; c.state_messages['ATTITUDE'][1].time_boot_ms = 1
    assert 'rollback' in a.read_state()['reason']


def test_initial_wait_timeout_and_later_first_qualified_state(state_link):
    a, f, c, clock, _ = state_link; attitude = c.state_messages.pop('ATTITUDE')
    with pytest.raises(RuntimeError, match='wait timed out'):
        wait_initial_state(a.read_state, timeout=.03)
    assert a.cache is None and a.fault is None
    c.state_messages['ATTITUDE'] = attitude; publish(f, c, clock)
    assert wait_initial_state(a.read_state)['valid']


def test_startup_hard_fault_is_not_retried(state_link):
    a, f, c, clock, _ = state_link; c.reboot_detected = True; start = clock.now
    with pytest.raises(RuntimeError, match='reboot'):wait_initial_state(a.read_state)
    assert clock.now == start


def test_static_transforms_not_recomputed_and_origin_check_is_low_frequency(state_link, monkeypatch):
    import bluerov2.state_adapter as module
    a, f, c, clock, path = state_link; calls = {'rotation':0, 'file':0}
    rotate = module.rotation_zyx; read_bytes = Path.read_bytes
    def counted_rotation(*args):calls['rotation'] += 1; return rotate(*args)
    def counted_file(p):calls['file'] += 1; return read_bytes(p)
    monkeypatch.setattr(module, 'rotation_zyx', counted_rotation)
    monkeypatch.setattr(Path, 'read_bytes', counted_file)
    monkeypatch.setattr(np.linalg, 'solve', lambda *_: pytest.fail('static feed inverse repeated'))
    assert a.read_state()['valid']
    for i in range(1, 20):clock.now = 10.+i*.005; assert a.read_state()['valid']
    assert calls == {'rotation':1, 'file':0}
    for i in range(1, 11):clock.now = 10.+i*.1; publish(f, c, clock); assert a.read_state()['valid']
    assert calls['file'] == 1


def test_legacy_parser_preserves_real_acquisition_metadata():
    import json
    backend = load_backend(Path(__file__).resolve().parents[2])
    feed = backend.DvlFeed('unused', 16171, np.eye(3).ravel().tolist(), 0., 1.)
    data = {'time':108., 'vx':.1, 'vy':.2, 'vz':.3, 'velocity_valid':True, 'status':0,
        'transducers':[{'beam_valid':True}]*4, 'time_of_validity':1716816187586209,
        'time_of_transmission':1716816187685960}
    class Sock:
        def __init__(self):self.done = False
        def recv(self, _):
            if self.done:raise BlockingIOError
            self.done = True; return (json.dumps(data)+'\n').encode()
    feed.sock = Sock(); feed.poll()
    assert feed.latest_velocity_metadata['time_of_validity'] == data['time_of_validity']
    assert feed.latest_velocity_metadata['time_of_transmission'] == data['time_of_transmission']
    np.testing.assert_allclose(feed.latest_velocity[1], [.1, .2, .3])


def test_direct_feed_bounds_report_parse_work_but_legacy_default_is_unchanged():
    backend = load_backend(Path(__file__).resolve().parents[2])
    config = dict(dvl_ip='unused', dvl_port=16171, dvl_to_task_ned_rotation=np.eye(3).ravel().tolist(), yaw_offset_rad=0., yaw_sign=1.)
    class Sock:
        def __init__(self):self.done = False
        def recv(self, _):
            if self.done:raise BlockingIOError
            self.done = True; return b'{}\n'*17
    legacy = backend.DvlFeed('unused', 16171, config['dvl_to_task_ned_rotation'], 0., 1.)
    legacy.sock = Sock(); legacy.poll(); assert legacy.buffer == ''
    bounded = make_feed(backend, config, bounded_parse=True); bounded.sock = Sock()
    with pytest.raises(backend.ControlError, match='report budget'):bounded.poll()


def test_asynchronous_candidate_cannot_refresh_snapshot_age(state_link):
    a, f, c, clock, _ = state_link
    # Keep processing latency small enough to observe a receipt skew failure
    # while the original complete snapshot is still inside the unchanged 0.25 s gate.
    f.latest_velocity_metadata['time_of_transmission'] = 100010000
    first = a.read_state(); clock.now = 10.16
    raw = SimpleNamespace(**vars(f.latest[1])); raw.raw_time = 100.16
    f.latest = (clock.now, raw, raw.position, np.pi/2)
    f.latest_velocity = (clock.now, f.latest_velocity[1])
    f.latest_velocity_metadata = {'time_of_validity':100160000, 'time_of_transmission':100170000}
    s = a.read_state()
    assert s['valid'] and s['cached'] and 'receipt time skew' in s['candidate_pending_reason']
    assert s['age'] == pytest.approx(.17) and s['receipt_mono'] == first['receipt_mono']
    clock.now = 10.25
    assert not a.read_state()['valid']


def test_hold_monitors_cache_expiry_before_refresh_and_never_restarts(state_link):
    from bluerov2 import parameters
    from bluerov2.execution import execute_lbts_plan
    a, f, c, clock, _ = state_link
    initial = a.read_state()['state']
    plan = {'success':True, 'nominalTrajectory':np.column_stack([initial, initial]),
        'controlTrajectory':np.ones((6, 1)), 'feedbackGains':[np.zeros((6, 12))],
        'time':np.array([0., .4]), 'model':parameters().model, 'world':None}
    events = []; steps = []; next_refresh = [clock.now]
    def send(force):events.append(('send', clock.now, np.asarray(force).copy()))
    def poll():
        sample = a.read_state()
        if not sample['valid']:raise RuntimeError(sample['reason'])
        if clock.now >= next_refresh[0]:
            events.append(('refresh', clock.now, None)); next_refresh[0] = clock.now+.04
    def stop():events.append(('stop', clock.now, None)); return {'confirmed':True}
    with pytest.raises(RuntimeError, match='cache expired'):
        execute_lbts_plan(plan, {'read_state':a.read_state, 'send_thruster_forces':send,
            'poll':poll, 'stop':stop, 'on_step':steps.append, 'require_stop_confirmation':True},
            {'enable_actuation':True, 'confirmation':'ENABLE_BLUEROV2_THRUST', 'wait_poll_period':.005})
    assert len(steps) == 1 and events[-1][0] == 'stop'
    assert np.all(events[-2][2] == 0) and clock.now < 10.16
    assert all(t < 10.15 for kind, t, _ in events if kind == 'refresh')
    clock.now += .01; publish(f, c, clock)
    assert not a.read_state()['valid']
