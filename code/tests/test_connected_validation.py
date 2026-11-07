import ast
import importlib.util
from pathlib import Path
import struct
from types import SimpleNamespace
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('verify_connected',ROOT/'tools/verify_connected.py')
v=importlib.util.module_from_spec(spec);spec.loader.exec_module(v)


def packet(ds, fields):
    payload=b''.join(bytes((len(data)+2,desc))+data for desc,data in fields)
    raw=bytes((0x75,0x65,ds,len(payload)))+payload
    a=b=0
    for x in raw:
        a=(a+x)&255;b=(b+a)&255
    return raw+bytes((a,b))


def test_mip_checksum_fragmentation_and_units():
    p=v.MipParser()
    raw=packet(0x80,[(5,struct.pack('>3f',.1,-.2,.3))])
    assert p.feed(b'noise'+raw[:3])==[]
    result=p.feed(raw[3:])
    assert result[0]['fields']['0x05']['units']=='rad/s'
    np.testing.assert_allclose(result[0]['fields']['0x05']['vector'],[.1,-.2,.3])
    corrupt=bytearray(raw);corrupt[-1]^=1
    assert p.feed(bytes(corrupt))==[] and p.bad_checksums==1
    assert len(p.feed(raw))==1


def test_mip_delta_theta_is_not_rate_and_invalid_time_stays_invalid():
    p=v.MipParser()
    data=p.feed(packet(0x80,[(7,struct.pack('>3f',1,2,3)),(0xD3,struct.pack('>dHH',100.,0,0))]))[0]
    assert data['fields']['0x07']['units']=='rad per sample'
    assert data['fields']['0xd3']['valid_flags']==0


def test_non_equal_sampling_and_receipt_skew_reported():
    cfg={'max_age_s':.25,'max_receipt_skew_s':.15}
    dvl=[{'monotonic_s':x,'data':data} for x,data in
         [(1,{'ts':1}),(1.01,{'velocity_valid':True}),(1.3,{'velocity_valid':True})]]
    gyro=[{'monotonic_s':1.02,'data':{}}]
    r=v.timing_check(dvl,gyro,cfg)
    assert r['event_checks']['receipt_timing_eligible']==1
    assert r['event_checks']['receipt_age_exceeded']==1
    assert r['does_not_certify_coordinates_or_acquisition_latency']


def test_recording_bridge_si_units_and_unavailable_orientation():
    source=ROOT.parent/'recording/src/recording/recording/mavlink_bridge.py'
    tree=ast.parse(source.read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='MavlinkBridgeNode')
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_publish_imu')
    def make_imu():
        return SimpleNamespace(header=SimpleNamespace(),linear_acceleration=SimpleNamespace(),
             angular_velocity=SimpleNamespace(),orientation=SimpleNamespace(),orientation_covariance=[0.]*9)
    ns={'Imu':make_imu}
    exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),ns)
    published=[]
    node=SimpleNamespace(get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:42)),
                         imu_pub=SimpleNamespace(publish=published.append))
    msg=SimpleNamespace(xacc=1000,yacc=-1000,zacc=0,xgyro=1000,ygyro=-2000,zgyro=50)
    ns['_publish_imu'](node,msg)
    imu=published[0]
    assert imu.linear_acceleration.x==9.80665 and imu.linear_acceleration.y==-9.80665
    assert imu.angular_velocity.x==1. and imu.angular_velocity.y==-2.
    assert imu.orientation_covariance[0]==-1.
