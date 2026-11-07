"""Read-only live version/parameter/capability check. Never arm, change mode, or write parameters."""
import argparse
import json
from pathlib import Path
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bluerov2.force_run import load_config, DEFAULT_CONFIG
from bluerov2.communication import load_communication_backend, load_connection_config
from bluerov2.passthrough_bench import require_free_udp_endpoint
from bluerov2.thruster_output import controller_class,encode_frame,QUERY
from datetime import datetime, timezone
from collections import Counter


def validate_readonly_message(msg, target):
    """Fail closed before MAVLink serialization: only metadata reads and QUERY."""
    data=msg.to_dict();kind=msg.get_type()
    if kind=='DATA64':
        if data['type']!=0xD6 or data['len']!=34 or bytes(data['data'])!=encode_frame(QUERY,target):
            raise ValueError('only neutral, sessionless direct6 QUERY is allowed')
    else:
        if (data.get('target_system'),data.get('target_component'))!=target:
            raise ValueError('read-only request target mismatch')
        if kind=='COMMAND_LONG':
            if data['command']!=520 or data['param1']!=1 or any(data[f'param{i}']!=0 for i in range(2,8)):
                raise ValueError('only version query command allowed')
        elif kind!='PARAM_REQUEST_READ':
            raise ValueError('prohibited message in read-only diagnosis: '+kind)


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=DEFAULT_CONFIG)
    p.add_argument('--output',type=Path,default=Path('reports/live_diagnosis.json'))
    a=p.parse_args(argv);cfg=load_config(a.config,'pwm-test')
    backend=load_communication_backend(cfg['legacy_directory']);old=load_connection_config(cfg['legacy_config'])
    a.output.parent.mkdir(parents=True,exist_ok=True)
    logger=backend.SessionLogger(a.output.parent/'readonly_logs',action='direct6_read_only_diagnosis')
    sent=[]
    class ReadOnlyController(controller_class(backend)):
        def _send_heartbeat(self):pass  # Do not maintain anyone else's GCS failsafe.
        def service_io(self,*args,**kwargs):
            if self.master is not None and not getattr(self,'guard_installed',False):
                original=self.master.mav.send
                def guarded(msg,*args,**kwargs):
                    validate_readonly_message(msg,(self.target_system,self.target_component))
                    original(msg,*args,**kwargs)
                    sent.append({'host_mono':time.monotonic(),'message':msg.to_dict()})
                self.master.mav.send=guarded;self.guard_installed=True
            return super().service_io(*args,**kwargs)
    c=ReadOnlyController(old['connection'],old['baud'],old['source_system'],cfg['source_component'],
                        old.get('target_system'),old.get('target_component'),logger,5.,2.,3.)
    report={'created_utc':datetime.now(timezone.utc).isoformat(),'read_only':True,'connected':False,
            'configured_endpoint':old['connection'],'source':[old['source_system'],cfg['source_component']],
            'query_protocol':{'carrier':'DATA64','type':214,'length':34},'dvl_required':False}
    try:
        require_free_udp_endpoint(old['connection'])
        c.connect();mav=c.master.mav
        report.update(connected=True,target=[c.target_system,c.target_component],armed=c.vehicle.armed,mode=c.vehicle.mode,
                      observed_udp_peers=[list(peer) for peer in getattr(c.master,'clients',[])])
        mav.command_long_send(c.target_system,c.target_component,520,0,1,0,0,0,0,0,0)
        parameter_names=list(cfg['required_parameters'])+['SYSID_MYGCS','FS_PILOT_INPUT','FS_PILOT_TIMEOUT']
        for name in dict.fromkeys(parameter_names):
            mav.param_request_read_send(c.target_system,c.target_component,name.encode(),-1)
        end=time.monotonic()+3;next_query=0.
        while time.monotonic()<end:
            if time.monotonic()>=next_query:
                mav.data64_send(0xD6,34,list(encode_frame(QUERY,(c.target_system,c.target_component))))
                next_query=time.monotonic()+1.
            c.service_io();time.sleep(.005)
        v=c.vehicle.autopilot_version;report['autopilot_version']=v
        if v.get('flight_sw_version') is not None:
            n=int(v['flight_sw_version']);report['version']=[(n>>24)&255,(n>>16)&255,(n>>8)&255]
        report['parameters']=c.vehicle.parameters
        report['direct6_status']=c.direct_status
        report['direct6_evidence']='reply_received' if c.direct_status else 'no_reply_observed; firmware support and DATA64 routing not individually resolved'
        report['received_message_types']=sorted(c.state_messages)
        if 'SERVO_OUTPUT_RAW' in c.state_messages:
            msg=c.state_messages['SERVO_OUTPUT_RAW'][1]
            report['observed_pwm_us']=[getattr(msg,f'servo{i}_raw') for i in range(1,7)]
        report['board_model']='undetermined; board_version/USB IDs alone do not certify a build target'
    except Exception as exc:report['error']=f'{type(exc).__name__}: {exc}'
    finally:c.close();logger.close()
    report['sent_requests']=sent
    report['sent_message_counts']=dict(Counter(r['message']['mavpackettype'] for r in sent))
    report['actuator_commands_sent']=False  # Enforced by guard, including before target discovery.
    report['parameter_writes']=0
    a.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False,indent=2))
    return 0 if report['connected'] and 'error' not in report else 1

if __name__=='__main__':raise SystemExit(main())
