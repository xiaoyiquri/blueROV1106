function experiment=executeLBTSPlanHardware(result,io,userOptions)
%EXECUTELBTSPLANHARDWARE 在真实 BlueROV2 上闭环执行 LBTS 规划结果。
%
%   experiment=executeLBTSPlanHardware(result,io,options)
%
% 本函数不模拟机器人，也不人为叠加过程噪声或测量噪声。真实水动力误差、
% 水流和传感器噪声会自然出现在 io.readState 返回的状态估计中。控制律为
%
%   f_k = saturation(ubar_k-K_k*(xhat_k-xbar_k)),
%
% 其中姿态误差包络到 [-pi,pi]，K_k 直接取自 LBTS 最终路径。每个 f_k
% 在 [time(k),time(k+1)) 内保持；单位是 N，推进器顺序必须与规划模型一致。
%
% io 回调接口：
%   io.initialize()                 可选，建立通信但不要自动解锁推进器
%   sample=io.readState()           必需，读取真实系统的 12 维状态估计
%   io.sendThrusterForces(f)        实机输出时必需，f 为 8x1 推力 [N]
%   io.abortRequested()             可选，返回急停/人工终止布尔值
%   io.stop()                       可选，发送零推力、上锁并关闭通信
%   io.onStart(result,options)      可选，正式计时前调用
%   io.onFinish(experiment)         可选，正常或安全中止后调用
%
% readState 可以直接返回 12x1 数值，也可以返回结构体：
%   sample.state      必需，状态顺序 [N,E,D,roll,pitch,yaw,u,v,w,p,q,r]'
%   sample.valid      可选，默认 true
%   sample.age        可选，数据年龄 [s]；实机输出默认要求提供
%   sample.timestamp  可选，设备时间戳，仅用于日志
%
% 为防止误启动，默认 options.enableActuation=false。真实输出还必须设置：
%   options.enableActuation=true;
%   options.confirmation='ENABLE_BLUEROV2_THRUST';
%
% 重要：readState 应返回已经融合好的状态估计 xhat，而不是原始 IMU/DVL/
% 深度计数据。若机载系统已经运行 EKF，不应再次应用 result.kalmanGains，
% 否则会对同一测量重复滤波。

if nargin<3, userOptions=struct(); end
options=completeHardwareOptions(userOptions);
validatePlanAndIO(result,io,options);

X=result.nominalTrajectory;
U=result.controlTrajectory;
time=reshape(result.time,1,[]);
K=result.feedbackGains;
steps=size(U,2);

if isfield(io,'initialize') && ~isempty(io.initialize), feval(io.initialize); end
cleanup=onCleanup(@()safeStop(io,options.enableActuation));

preflight=readStateSample(io);
assertStateSample(preflight,options,true);
[preflightOK,preflightReason]=checkStartState(preflight.state,X(:,1),result.world,options);
assert(preflightOK,'LBTSHardware:Preflight','实机预检失败：%s',preflightReason);
if options.enableActuation && options.requireStateAge
    assert(isfinite(preflight.age),'LBTSHardware:StateAge','实机输出要求 readState 返回 sample.age。');
end
if isfield(io,'onStart') && ~isempty(io.onStart), feval(io.onStart,result,options); end

estimatedState=nan(12,steps+1);
trackingError=nan(12,steps+1);
feedforward=U;
feedback=nan(8,steps);
unconstrainedCommand=nan(8,steps);
command=nan(8,steps);
scheduledTime=time;
actualTime=nan(1,steps+1);
stateAge=nan(1,steps+1);
deviceTimestamp=cell(1,steps+1);
saturated=false(8,steps);
status='completed'; abortReason=''; executedSteps=0;
runClock=tic;

for k=1:steps
    if options.realTime
        if waitUntil(runClock,time(k),io,options), status='aborted'; abortReason='operator_abort'; break; end
        timingOverrun=toc(runClock)-time(k);
        if timingOverrun>options.maxTimingOverrun
            status='aborted'; abortReason=sprintf('timing_overrun_%.3f_s',timingOverrun); break
        end
    end
    if abortIsRequested(io), status='aborted'; abortReason='operator_abort'; break; end

    sample=readStateSample(io);
    actualTime(k)=elapsedOrScheduled(runClock,time(k),options.realTime);
    estimatedState(:,k)=sample.state;
    stateAge(k)=sample.age;
    deviceTimestamp{k}=sample.timestamp;
    [sampleOK,sampleReason]=checkRuntimeState(sample,X(:,k),result.world,options);
    if ~sampleOK, status='aborted'; abortReason=sampleReason; break; end

    error=stateDifference(sample.state,X(:,k));
    trackingError(:,k)=error;
    feedback(:,k)=-K{k}*error;
    unconstrainedCommand(:,k)=U(:,k)+feedback(:,k);
    command(:,k)=min(result.model.thrustMax,max(result.model.thrustMin,unconstrainedCommand(:,k)));
    saturated(:,k)=abs(command(:,k)-unconstrainedCommand(:,k))>options.saturationTolerance;
    if options.enableActuation, feval(io.sendThrusterForces,command(:,k)); end
    executedSteps=k;

    if options.verbose
        fprintf('Hardware step %d/%d, t=%.3f s, |ep|=%.3f m, |ea|=%.2f deg, saturated=%d.\n',k,steps,actualTime(k),norm(error(1:3)),rad2deg(norm(error(4:6))),nnz(saturated(:,k)));
    end
end

if strcmp(status,'completed')
    if options.realTime && waitUntil(runClock,time(end),io,options), status='aborted'; abortReason='operator_abort'; end
    if strcmp(status,'completed')
        finalSample=readStateSample(io);
        actualTime(end)=elapsedOrScheduled(runClock,time(end),options.realTime);
        estimatedState(:,end)=finalSample.state;
        stateAge(end)=finalSample.age;
        deviceTimestamp{end}=finalSample.timestamp;
        [finalOK,finalReason]=checkRuntimeState(finalSample,X(:,end),result.world,options);
        if ~finalOK, status='aborted'; abortReason=finalReason; end
        trackingError(:,end)=stateDifference(finalSample.state,X(:,end));
    end
end
sendZeroThrust(io,options.enableActuation);

experiment=struct();
experiment.status=status;
experiment.abortReason=abortReason;
experiment.actuationEnabled=options.enableActuation;
experiment.executedSteps=executedSteps;
experiment.referenceState=X;
experiment.referenceControl=U;
experiment.scheduledTime=scheduledTime;
experiment.actualTime=actualTime;
experiment.estimatedState=estimatedState;
experiment.trackingError=trackingError;
experiment.feedforwardControl=feedforward;
experiment.feedbackControl=feedback;
experiment.unconstrainedCommand=unconstrainedCommand;
experiment.commandedThrusterForce=command;
experiment.saturated=saturated;
experiment.stateAge=stateAge;
experiment.deviceTimestamp=deviceTimestamp;
experiment.options=options;
experiment.createdAt=char(datetime('now','Format','yyyy-MM-dd HH:mm:ss.SSS Z'));
experiment.finalPositionError=finiteNorm(trackingError(1:3,end));
experiment.finalAttitudeError=finiteNorm(trackingError(4:6,end));

if options.verbose
    fprintf('Hardware execution %s: executed %d/%d intervals',status,executedSteps,steps);
    if ~isempty(abortReason), fprintf(', reason=%s',abortReason); end
    fprintf('.\n');
end
if ~isempty(options.logFile)
    logFolder=fileparts(options.logFile);
    assert(isempty(logFolder) || isfolder(logFolder),'LBTSHardware:LogFolder','日志目录不存在：%s',logFolder);
    save(options.logFile,'experiment','-v7.3');
end
if isfield(io,'onFinish') && ~isempty(io.onFinish), feval(io.onFinish,experiment); end
end

function options=completeHardwareOptions(user)
options=struct('enableActuation',false,'confirmation','','realTime',true,'waitPollPeriod',0.01,'maxTimingOverrun',0.25,'requireStateAge',true,'maxStateAge',0.25,'maxStartPositionError',0.25,'maxStartAttitudeError',deg2rad(15),'maxStartVelocityError',0.25,'maxStartAngularRateError',deg2rad(20),'maxPositionTrackingError',0.75,'maxAttitudeTrackingError',deg2rad(35),'maxLinearSpeed',1.20,'maxAngularSpeed',deg2rad(100),'abortOutsideFreeSpace',true,'saturationTolerance',1e-10,'logFile','','verbose',true);
names=fieldnames(user);
for k=1:numel(names), options.(names{k})=user.(names{k}); end
assert(islogical(options.enableActuation) && isscalar(options.enableActuation),'LBTSHardware:Options','enableActuation 必须是逻辑标量。');
assert(options.waitPollPeriod>0 && options.maxTimingOverrun>=0 && options.maxStateAge>0,'LBTSHardware:Options','时间参数必须为正数。');
if options.enableActuation, assert(strcmp(options.confirmation,'ENABLE_BLUEROV2_THRUST'),'LBTSHardware:Locked','若要输出真实推力，必须设置 confirmation=''ENABLE_BLUEROV2_THRUST''。'); end
end

function validatePlanAndIO(result,io,options)
assert(isstruct(result) && isfield(result,'success') && result.success,'LBTSHardware:Plan','需要成功的 LBTS result。');
required={'nominalTrajectory','controlTrajectory','time','feedbackGains','model','world'};
for k=1:numel(required), assert(isfield(result,required{k}),'LBTSHardware:Plan','result 缺少字段 %s。',required{k}); end
X=result.nominalTrajectory; U=result.controlTrajectory; time=reshape(result.time,1,[]); steps=size(U,2);
assert(size(X,1)==12 && size(U,1)==8 && size(X,2)==steps+1 && numel(time)==steps+1 && abs(time(1))<1e-12 && all(diff(time)>0),'LBTSHardware:Plan','名义状态、控制或时间格式错误。');
assert(numel(result.feedbackGains)==steps && all(cellfun(@(gain) isequal(size(gain),[8 12]),result.feedbackGains)),'LBTSHardware:Plan','feedbackGains 必须为每个控制区间提供一个 8x12 增益。');
assert(numel(result.model.thrustMin)==8 && numel(result.model.thrustMax)==8,'LBTSHardware:Plan','模型必须提供 8 个推进器的推力上下限。');
assert(isstruct(io) && isfield(io,'readState') && isa(io.readState,'function_handle'),'LBTSHardware:IO','io.readState 必须是函数句柄。');
if options.enableActuation, assert(isfield(io,'sendThrusterForces') && isa(io.sendThrusterForces,'function_handle'),'LBTSHardware:IO','实机输出需要 io.sendThrusterForces 函数句柄。'); end
end

function sample=readStateSample(io)
raw=feval(io.readState);
if isnumeric(raw)
    sample=struct('state',raw(:),'valid',true,'age',nan,'timestamp',[]);
else
    assert(isstruct(raw) && isfield(raw,'state'),'LBTSHardware:StateSample','readState 必须返回 12x1 数值或含 state 的结构体。');
    sample=raw;
    if ~isfield(sample,'valid') || isempty(sample.valid), sample.valid=true; end
    if ~isfield(sample,'age') || isempty(sample.age), sample.age=nan; end
    if ~isfield(sample,'timestamp'), sample.timestamp=[]; end
    sample.state=sample.state(:);
end
end

function assertStateSample(sample,options,isPreflight)
assert(isscalar(sample.valid) && sample.valid && numel(sample.state)==12 && all(isfinite(sample.state)),'LBTSHardware:StateSample','状态估计无效、非有限或不是 12 维。');
if isfinite(sample.age), assert(sample.age>=0,'LBTSHardware:StateAge','sample.age 不能为负数。'); end
if isPreflight && options.enableActuation && options.requireStateAge, assert(isfinite(sample.age),'LBTSHardware:StateAge','实机输出时 sample.age 不能为空。'); end
end

function [ok,reason]=checkStartState(state,reference,world,options)
error=stateDifference(state,reference);
ok=false; reason='';
if norm(error(1:3))>options.maxStartPositionError, reason='start_position_error'; return; end
if norm(error(4:6))>options.maxStartAttitudeError, reason='start_attitude_error'; return; end
if norm(error(7:9))>options.maxStartVelocityError, reason='start_velocity_error'; return; end
if norm(error(10:12))>options.maxStartAngularRateError, reason='start_angular_rate_error'; return; end
if options.abortOutsideFreeSpace && ~isPositionCollisionFreeLBTS(state(1:3),world), reason='start_not_in_free_space'; return; end
ok=true;
end

function [ok,reason]=checkRuntimeState(sample,reference,world,options)
ok=false; reason='';
try
    assertStateSample(sample,options,false);
catch
    reason='invalid_state_estimate'; return
end
if options.enableActuation && options.requireStateAge && (~isfinite(sample.age) || sample.age>options.maxStateAge), reason='stale_state_estimate'; return; end
error=stateDifference(sample.state,reference);
if norm(error(1:3))>options.maxPositionTrackingError, reason='position_tracking_error'; return; end
if norm(error(4:6))>options.maxAttitudeTrackingError, reason='attitude_tracking_error'; return; end
if norm(sample.state(7:9))>options.maxLinearSpeed, reason='linear_speed_limit'; return; end
if norm(sample.state(10:12))>options.maxAngularSpeed, reason='angular_speed_limit'; return; end
if options.abortOutsideFreeSpace && ~isPositionCollisionFreeLBTS(sample.state(1:3),world), reason='estimated_collision_or_out_of_bounds'; return; end
ok=true;
end

function error=stateDifference(state,reference)
error=state(:)-reference(:);
error(4:6)=atan2(sin(error(4:6)),cos(error(4:6)));
end

function requested=waitUntil(runClock,targetTime,io,options)
requested=false;
while toc(runClock)<targetTime
    if abortIsRequested(io), requested=true; return; end
    pause(min(options.waitPollPeriod,max(targetTime-toc(runClock),0)));
end
end

function requested=abortIsRequested(io)
requested=isfield(io,'abortRequested') && ~isempty(io.abortRequested) && logical(feval(io.abortRequested));
end

function value=elapsedOrScheduled(runClock,scheduled,realTime)
if realTime, value=toc(runClock); else, value=scheduled; end
end

function sendZeroThrust(io,actuationEnabled)
if actuationEnabled && isfield(io,'sendThrusterForces') && ~isempty(io.sendThrusterForces)
    try
        feval(io.sendThrusterForces,zeros(8,1));
    catch exception
        warning('LBTSHardware:StopCommand','发送零推力失败：%s',exception.message);
    end
end
end

function value=finiteNorm(vector)
if all(isfinite(vector)), value=norm(vector); else, value=nan; end
end

function safeStop(io,actuationEnabled)
sendZeroThrust(io,actuationEnabled);
if isfield(io,'stop') && ~isempty(io.stop)
    try
        feval(io.stop);
    catch exception
        warning('LBTSHardware:StopCallback','停止/断开回调失败：%s',exception.message);
    end
end
end
