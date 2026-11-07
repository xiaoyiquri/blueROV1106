"""Explicit plan/PWM execution entry, using the original LBTS feedback executor."""
from pathlib import Path
import argparse
import hashlib
import json
import signal
import time
import numpy as np
from .thruster_output import ThrusterOutput, controller_class, vector6
from .communication import load_communication_backend, load_connection_config

ROOT=Path(__file__).resolve().parents[1]
DEFAULT_CONFIG=Path(__file__).with_name('data')/'force_config.json'
STATE_ORDER=['N','E','D','roll','pitch','yaw','u','v','w','p','q','r']


def jsonable(value):
    if isinstance(value,np.ndarray):return jsonable(value.tolist())
    if isinstance(value,np.generic):return jsonable(value.item())
    if isinstance(value,dict):return {k:jsonable(v) for k,v in value.items()}
    if isinstance(value,(tuple,list)):return [jsonable(v) for v in value]
    if isinstance(value,float) and not np.isfinite(value):return None
    if hasattr(value,'__dict__'):return jsonable(vars(value))
    return value


def model_signature(model):
    fields={k:np.asarray(getattr(model,k)).tolist() for k in ('thruster_position','thruster_direction','thrust_min','thrust_max')}
    return hashlib.sha256(json.dumps(fields,sort_keys=True).encode()).hexdigest()


def save_force_plan(path,result,origin_file):
    """Attach coordinate/unit metadata to an existing result, without changing its arrays or model.

    Call only if the planner used this fixed DVL task basis, translated by this origin.
    This function does not rotate/translate an incompatible existing plan.
    """
    result=dict(result)
    result['execution_reference']={'frame':'local_at_dvl_reset','attitude_reference':'fixed_dvl_reset_world',
        'origin_sha256':hashlib.sha256(Path(origin_file).read_bytes()).hexdigest(),
        'model_signature':model_signature(result['model']), 'state_order':STATE_ORDER,
        'units':['m','rad','m/s','rad/s','N','s']}
    from .io import save_result
    save_result(path,result)


def load_config(path, mode='plan'):
    path=Path(path).resolve();cfg=json.loads(path.read_text())
    paths=['legacy_directory','legacy_config','log_directory']
    if mode in ('plan','thrust-test'):paths.append('curve_file')
    for name in paths:
        p=Path(cfg[name]);cfg[name]=str(p if p.is_absolute() else path.parent/p)
    output=cfg['output']
    period=output['refresh_period_s'];lease=output['lease_ms']
    if not np.isfinite(period) or period<=0 or not np.isfinite(lease) or not 50<=lease<=200 or period*1000>=lease/2:
        raise ValueError('refresh period must be positive and less than half the 50..200 ms lease')
    if mode in ('pwm-test','thrust-test'):
        if mode == 'thrust-test':
            if not np.isfinite(output['voltage_max_age_s']) or output['voltage_max_age_s']<=0:
                raise ValueError('invalid output.voltage_max_age_s')
            for name in ('test_voltage_v','hardware_clip_tolerance_n'):
                value=output[name]
                if name == 'test_voltage_v' and value is None:continue
                if not np.isfinite(value) or (name == 'test_voltage_v' and value<=0) or (name == 'hardware_clip_tolerance_n' and not 0<=value<=.1):
                    raise ValueError('invalid output.'+name)
        return cfg
    from .state_adapter import proper_rotation
    for name in ('voltage_max_age_s',):
        if not np.isfinite(output[name]) or output[name]<=0:raise ValueError(name)
    if not np.isfinite(output['hardware_clip_tolerance_n']) or not 0<=output['hardware_clip_tolerance_n']<=.1:
        raise ValueError('hardware clipping tolerance must be 0..0.1 N')
    for name in ('max_age_s','max_receipt_skew_s','max_position_std_m','position_jump_margin_m','attitude_jump_margin_rad','max_plausible_speed_m_s','max_plausible_rate_rad_s'):
        if not np.isfinite(cfg['state'][name]) or cfg['state'][name]<=0:raise ValueError('state.'+name)
    for name in ('world_from_dvl_world','body_from_dvl_sensor'):proper_rotation(cfg['state'][name])
    return cfg


def validate_plan(result,cfg,origin_file,mapper,voltage=None):
    from .execution import execute_lbts_plan
    result['model'].validate()
    if model_signature(result['model'])!=cfg['model_signature']:raise ValueError('plan model geometry/thrust limits do not match the verified channel mapping configuration')
    ref=result.get('execution_reference',{})
    expected={'frame':'local_at_dvl_reset','attitude_reference':'fixed_dvl_reset_world',
        'origin_sha256':hashlib.sha256(Path(origin_file).read_bytes()).hexdigest(),
        'model_signature':model_signature(result['model']),'state_order':STATE_ORDER,
        'units':['m','rad','m/s','rad/s','N','s']}
    if ref!=expected:raise ValueError('plan reference/units/model metadata missing or mismatched; use save_force_plan with the original planning reference')
    # Reuse the executor's plan/limit validation before opening any link.
    x=np.asarray(result['nominalTrajectory'],float);counter=iter([x[:,0],*x.T,x[:,-1]])
    checked=execute_lbts_plan(result,{'read_state':lambda:{'state':next(counter),'age':0.}},
                      {**cfg['executor'],'enable_actuation':False,'real_time':False})
    if checked['status']!='completed':raise ValueError('plan violates original executor guard: '+checked['abort_reason'])
    u=np.asarray(result['controlTrajectory'])
    if np.any(u<result['model'].thrust_min[:,None]) or np.any(u>result['model'].thrust_max[:,None]):
        raise ValueError('nominal plan exceeds its original algorithm force limits')
    if voltage is not None:
        lo,hi=mapper.limits(voltage)
        excess=np.maximum(lo[:,None]-u,u-hi[:,None])
        if np.max(excess)>cfg['output']['hardware_clip_tolerance_n']:
            raise ValueError(f'nominal plan exceeds hardware capability at {voltage:.3f} V by {np.max(excess):.3f} N')
    return {'intervals':u.shape[1],'duration_s':float(result['time'][-1]),'model_signature':expected['model_signature']}


def write_plots(report,path):
    if not report.get('steps'):return
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    steps=report.get('steps',[])
    if steps:
        t=[s['actual_time'] for s in steps]
        fig,ax=plt.subplots(2,1,sharex=True)
        error=np.array([s['tracking_error'] for s in steps])
        ax[0].plot(t,error[:,:3]);ax[0].set_ylabel('Position error (m)');ax[0].legend(['N/local X','E/local Y','D'],fontsize=8)
        ax[1].plot(t,error[:,3:6]);ax[1].set_ylabel('Attitude error (rad)');ax[1].set_xlabel('Time (s)');ax[1].legend(['roll','pitch','yaw'],fontsize=8)
        fig.savefig(path.with_suffix('.error.png'));plt.close(fig)
        fig,ax=plt.subplots(2,1,sharex=True)
        force=np.array([s['algorithm_force_n'] for s in steps]);pwm=np.array([s['output']['pwm_us'] for s in steps])
        ends=report.get('experiment',{}).get('actual_time',[])
        if len(ends) and ends[-1] is not None and np.isfinite(ends[-1]) and ends[-1]>t[-1]:
            t=[*t,ends[-1]];force=np.vstack([force,force[-1]]);pwm=np.vstack([pwm,pwm[-1]])
        ax[0].step(t,force,where='post');ax[0].set_ylabel('Command force (N)');ax[0].legend([f'model {i}' for i in range(6)],ncol=3,fontsize=7)
        # Forces are in model order; PWM columns are physical channel order.
        by_channel={m['output_channel']:m for m in report.get('mapping',[])}
        for channel in range(1,7):
            m=by_channel.get(channel)
            label=f'MAIN{channel}' if m is None else f'MAIN{channel} (model {m["model_index"]}, sign {m["model_to_curve_sign"]:+g})'
            ax[1].step(t,pwm[:,channel-1],where='post',label=label,
                       color=f'C{m["model_index"] if m else channel-1}')
        ax[1].set_ylabel('Sent PWM (us)');ax[1].set_xlabel('Time (s)');ax[1].legend(ncol=2,fontsize=7)
        fig.savefig(path.with_suffix('.output.png'));plt.close(fig)


def wait_initial_state(read, timeout=3., poll_period=.01):
    deadline=time.monotonic()+timeout
    while True:
        sample=read()
        if sample['valid']:return sample
        if sample.get('status') != 'waiting':
            raise RuntimeError(sample.get('reason','invalid initial state'))
        if time.monotonic()>=deadline:
            raise RuntimeError('initial state wait timed out: '+sample.get('reason','missing state'))
        time.sleep(min(poll_period,max(0.,deadline-time.monotonic())))


def run(args,cfg):
    is_plan=args.command=='plan'
    is_thrust=args.command=='thrust-test'
    report={'status':'checked','actuation_requested':args.execute,
            'mode':'direct6_force' if is_plan else ('direct6_thrust_test' if is_thrust else 'direct6_pwm_test'),
            'steps':[],'tracking_success':None,'sequence_completion_is_tracking_success':False}
    mapper=None;result=None
    if not is_plan:
        if is_thrust:
            from .t200 import T200
            mapper=T200(cfg['mapping'],cfg['curve_file'])
            report['mapping']=mapper.mapping
            forces=vector6(args.forces_n,'forces_n')
            if not np.isfinite(args.duration) or not 0<args.duration<=5:
                raise ValueError('thrust test duration must be in (0,5] s')
            report['plan']={'model_index_force_n':forces.tolist(),'duration_s':args.duration}
            voltage=cfg['output']['test_voltage_v']
            if voltage is not None and not args.execute:
                mapped=mapper.map_forces(forces,voltage,cfg['output']['hardware_clip_tolerance_n'])
                report['plan']['preview']={**jsonable(mapped),'voltage_source':'configured_test_voltage'}
                report['plan']['voltage_source']='configured_test_voltage'
            if not args.execute:return report
        else:
            pwm=vector6(args.pwm,'pwm_us')
            if np.any((pwm<1100)|(pwm>1900)) or np.any(pwm!=np.rint(pwm)) or not np.isfinite(args.duration) or not 0<args.duration<=5:
                raise ValueError('PWM test requires six integer 1100..1900 values and duration in (0,5] s')
            report['plan']={'physical_channel_pwm_us':pwm.tolist(),'duration_s':args.duration}
            if not args.execute:return report
        backend=load_communication_backend(cfg['legacy_directory'])
        legacy=load_connection_config(cfg['legacy_config'])
    else:
        # Planning, DVL, frames, model and curves belong exclusively to this branch.
        from .execution import execute_lbts_plan
        from .host_tracking import load_backend, make_feed
        from .io import load_result
        from .state_adapter import StateAdapter, proper_rotation
        from .t200 import T200
        backend=load_backend(cfg['legacy_directory'])
        legacy=backend.load_config(Path(cfg['legacy_config']))
        base=Path(cfg['legacy_config']).parent
        origin_path=Path(legacy['origin_file'])
        origin_path=origin_path if origin_path.is_absolute() else base/origin_path
        origin=backend.load_origin_state(origin_path,legacy)
        if origin is None:raise ValueError('existing fixed task_origin.json is required; no automatic reset/origin creation')
        if not np.allclose(proper_rotation(legacy['dvl_to_task_ned_rotation']),proper_rotation(cfg['state']['world_from_dvl_world'])):
            raise ValueError('world rotation must match the existing saved task basis')
        mapper=T200(cfg['mapping'],cfg['curve_file'])
        report['mapping']=mapper.mapping
        result=load_result(args.plan)
        report['plan']=validate_plan(result,cfg,origin_path,mapper)
        if not args.execute:return report
        if not cfg['state']['frame_verified']:
            raise ValueError('12-state installation, reference, and timestamp semantics not verified')
        if not cfg['model_geometry_compatible_verified']:
            raise ValueError('model geometry and physical channel/force correspondence not yet verified; algorithm unchanged')
        if not cfg['output']['firmware_verified']:
            raise ValueError('trajectory output installation/physical verification record is incomplete')
    logdir=Path(cfg['log_directory']);logdir.mkdir(parents=True,exist_ok=True)
    logger=backend.SessionLogger(logdir,action='direct6')
    def log(kind,**data):logger.write_log(kind,**jsonable(data))
    if mapper is not None:
        log('force_channel_mapping',mapping=mapper.mapping,force_order='model_index 0..5',
            pwm_order='physical MAIN1..6',configured_test_voltage_v=cfg['output']['test_voltage_v'])
    ctrl=controller_class(backend)(legacy['connection'],legacy['baud'],legacy['source_system'],cfg['source_component'],
        legacy.get('target_system'),legacy.get('target_component'),logger,
        legacy.get('connect_timeout',15.),legacy.get('confirm_timeout',5.),legacy.get('heartbeat_timeout',3.))
    output=ThrusterOutput(ctrl,mapper,cfg['output'],log)
    feed=None
    def interrupted(signum,_frame):ctrl.stop_requested=True;ctrl.termination_reason=f'signal_{signum}'
    handlers={s:signal.signal(s,interrupted) for s in (signal.SIGINT,signal.SIGTERM)}
    try:
        ctrl.connect()
        if ctrl.vehicle.vehicle_type!=12 or ctrl.vehicle.autopilot!=3:raise ValueError('selected target is not an ArduSub submarine')
        if ctrl.vehicle.armed is not False:raise ValueError('refusing an already armed/unknown vehicle')
        # Version and required parameters are requested read-only. Never set a parameter.
        mav=ctrl.master.mav;mav.command_long_send(ctrl.target_system,ctrl.target_component,520,0,1,0,0,0,0,0,0)
        for name in cfg['required_parameters']:
            mav.param_request_read_send(ctrl.target_system,ctrl.target_component,name.encode(),-1)
        end=time.monotonic()+3
        while time.monotonic()<end:
            ctrl.service_io();time.sleep(.005)
        v=ctrl.vehicle.autopilot_version.get('flight_sw_version')
        if v is None or ((int(v)>>24)&255,(int(v)>>16)&255,(int(v)>>8)&255)!=(3,5,4):
            raise ValueError('live firmware is not verified ArduSub 3.5.4; do not apply this patch to another version')
        report['firmware']=ctrl.vehicle.autopilot_version
        report['parameters']=ctrl.vehicle.parameters
        for name,value in cfg['required_parameters'].items():
            if ctrl.vehicle.parameters.get(name)!=value:raise ValueError(f'output parameter not verified: {name} expected {value}, got {ctrl.vehicle.parameters.get(name)}')
        if result is not None:
            feed=make_feed(backend,legacy,bounded_parse=True);feed.connect()
            adapter=StateAdapter(feed,ctrl,origin,origin_path,cfg['state'])
            wait_initial_state(adapter.read_state)
            voltage,source=output.voltage();report['preflight_voltage']={'voltage_v':voltage,'source':source}
            validate_plan(result,cfg,origin_path,mapper,voltage)
            def poll():
                sample=adapter.read_state()
                if not sample['valid']:raise RuntimeError(sample.get('reason','invalid_state_estimate'))
                output.poll(service_io=False)
            def on_step(entry):
                report['steps'].append(jsonable(entry));log('control_step',**entry)
            experiment=execute_lbts_plan(result,{
                'read_state':adapter.read_state,'send_thruster_forces':output.send_thruster_forces,
                'on_start':lambda *_:output.prepare(),'stop':output.stop,'poll':poll,
                'set_interval_deadline':output.set_interval_deadline,'abort_requested':lambda:ctrl.stop_requested,
                'on_step':on_step,'require_stop_confirmation':True},
                {**cfg['executor'],'enable_actuation':True,'confirmation':'ENABLE_BLUEROV2_THRUST','real_time':True})
            report['experiment']=jsonable(experiment)
            report['status']='sequence_completed' if experiment['status']=='completed' else experiment['status']
            report['reason']=experiment['abort_reason']
        else:
            # Physical channel test is how mapping is verified, so it does not require force-sign approval.
            if mapper is not None:
                if not cfg['output']['channel_mapping_verified']:
                    raise ValueError('model-to-motor/channel mapping requires a one-time physical verification record')
                voltage,source=output.voltage()
                report['plan']['voltage_v']=voltage;report['plan']['voltage_source']=source
                report['plan']['preview']={**jsonable(mapper.map_forces(forces,voltage,cfg['output']['hardware_clip_tolerance_n'])),
                                          'voltage_source':source}
            output.prepare(require_mapping=mapper is not None)
            deadline=time.monotonic()+args.duration;output.set_interval_deadline(deadline)
            if mapper is None:output.send_thruster_pwm(pwm)
            else:report['output']=jsonable(output.send_thruster_forces(forces))
            while time.monotonic()<deadline:
                output.poll();time.sleep(min(.005,max(0,deadline-time.monotonic())))
            report['status']='sequence_completed'
    except Exception as exc:
        report['status']='aborted';report['reason']=f'{type(exc).__name__}: {exc}'
    finally:
        stop=output.stop();report['stop']=stop
        if output.acquire_attempted and not stop['confirmed']:report['status']='stop_unconfirmed'
        if feed is not None:feed.close()
        ctrl.close();logger.close()
        for s,h in handlers.items():signal.signal(s,h)
    return report


def main(argv=None):
    parser=argparse.ArgumentParser(description='六推进器轨迹推力执行；默认离线检查，不驱动实机')
    parser.add_argument('--config',type=Path,default=DEFAULT_CONFIG)
    parser.add_argument('--output',type=Path,default=Path('force_report.json'))
    sub=parser.add_subparsers(dest='command',required=True)
    p=sub.add_parser('plan');p.add_argument('plan',type=Path);p.add_argument('--execute',action='store_true')
    p=sub.add_parser('pwm-test');p.add_argument('--pwm',type=float,nargs=6,required=True);p.add_argument('--duration',type=float,default=.5);p.add_argument('--execute',action='store_true')
    p=sub.add_parser('thrust-test');p.add_argument('--forces-n',type=float,nargs=6,required=True,metavar=('F0','F1','F2','F3','F4','F5'));p.add_argument('--duration',type=float,default=.5);p.add_argument('--execute',action='store_true')
    args=parser.parse_args(argv)
    try:report=run(args,load_config(args.config,args.command))
    except Exception as exc:report={'status':'rejected','reason':f'{type(exc).__name__}: {exc}','actuation_requested':args.execute}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(jsonable(report),ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    write_plots(report,args.output)
    print(json.dumps({'status':report['status'],'reason':report.get('reason'),'report':str(args.output)},ensure_ascii=False))
    return 0 if report['status'] in ('checked','sequence_completed') else 1

if __name__=='__main__':raise SystemExit(main())
