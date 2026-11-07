function [simulation, controller] = MC_path(result, userOptions)
%MC_PATH 对 LBTS 最终路径执行 BlueROV2 非线性闭环 Monte Carlo 仿真。
%
% 用法：
%   result = LBTS(graphFile,xStart,xGoal,options);
%   simulation = MC_path(result);
%   [simulation,controller] = MC_path(result,mcOptions);
%
% 与 double-integrator 参考代码相比，本函数不会在 Monte Carlo 循环中
% 重复调用 meanControl、动力学线性化、Riccati 递推或 Kalman 协方差
% 递推。LBTS 已经为最终 belief 路径的每一条边计算了 A_k、B_k、LQR
% 增益 K_k 和 Kalman 增益 L_k；assembleLBTSResult 将这些逐边序列按
% 路径顺序拼接。本函数在进入随机仿真前只做一次维度检查和噪声平方根
% 分解，随后所有 Monte Carlo 样本共享同一控制器时序。
%
% 重要：K_k 仍是“每条图边独立反向 Riccati，再沿路径拼接”的结果，
% 与规划时 propagateBelief 使用的控制器完全一致，并不是对整条路径
% 重新求一个全局 TVLQR。
%
% 非线性闭环仿真：
%   u_k = ubar_k - K_k wrap(xhat_k-xbar_k)
%   x_{k+1} = F_RK4(x_k,u_k,dt_k) + w_k
%   xhat^-_{k+1} = F_RK4(xhat_k,u_k,dt_k)
%   xhat_{k+1} = xhat^-_{k+1}
%       + L_k(z_{k+1}-C_k*xhat^-_{k+1})
% 其中 w_k~N(0,Qc*dt_k)，测量噪声服从 N(0,R_k)。真实状态和估计
% 状态均用完整 6-DOF BlueROV2 RK4 模型推进，A/B 只用于生成规划器的
% 固定时变增益，不在 Monte Carlo 内层重新计算。

if nargin<2, userOptions=struct(); end
settings=completeMCOptions(userOptions);
assert(isstruct(result) && isfield(result,'success') && result.success, ...
    'MC_path:NoPlan','必须提供 result.success=true 的 LBTS 规划结果。');

ensureKPRMPath(result);
[xbar,ubar,time]=validateNominalResult(result);
model=result.model;
world=result.world;
controller=prepareController(result,size(ubar,2));

nx=12; nu=8; steps=size(ubar,2); runs=settings.numRuns;
dt=diff(time);

% 所有矩阵平方根只计算一次。规划模型使用离散过程噪声 Qk=Qc*dt。
Qsqrt=cell(1,steps);
Rsqrt=cell(1,steps);
for k=1:steps
    Qsqrt{k}=covarianceSquareRoot(result.options.processNoiseQc*dt(k));
    Rsqrt{k}=covarianceSquareRoot(controller.R{k});
end
P0=result.covarianceSequence{1};
Ptilde0=result.estimationCovarianceSequence{1};
Phat0=nearestPSD(P0-Ptilde0,0);
hatRoot=covarianceSquareRoot(Phat0);
errorRoot=covarianceSquareRoot(Ptilde0);

% 独立 RandStream 不修改 MATLAB 全局随机状态。所有标准正态随机数一次
% 生成，避免在时间/样本双循环中反复调用 randn。
stream=RandStream('mt19937ar','Seed',settings.rngSeed);
initialHatNoise=randn(stream,nx,runs);
initialErrorNoise=randn(stream,nx,runs);
processNoise=randn(stream,nx,steps,runs);
measurementDimension=size(controller.C{1},1);
measurementNoise=randn(stream,measurementDimension,steps,runs);

trueState=zeros(nx,steps+1,runs);
estimatedState=zeros(nx,steps+1,runs);
control=zeros(nu,steps,runs);
estimated0=xbar(:,1)+hatRoot*initialHatNoise;
true0=estimated0+errorRoot*initialErrorNoise;
trueState(:,1,:)=reshape(true0,nx,1,runs);
estimatedState(:,1,:)=reshape(estimated0,nx,1,runs);
saturatedElementCount=0;

for k=1:steps
    xhat=reshape(estimatedState(:,k,:),nx,runs);
    trackingError=stateDifferenceMatrix(xhat,xbar(:,k));
    unconstrainedControl=ubar(:,k)-controller.K{k}*trackingError;
    commandedControl=unconstrainedControl;
    if settings.enforceThrustLimits
        commandedControl=min(model.thrustMax,max(model.thrustMin, commandedControl));
    end
    saturatedElementCount=saturatedElementCount+nnz(abs(commandedControl-unconstrainedControl)>1e-12);
    control(:,k,:)=reshape(commandedControl,nu,1,runs);

    trueNext=zeros(nx,runs);
    estimatePrior=zeros(nx,runs);
    for sample=1:runs
        uk=commandedControl(:,sample);
        % 完整非线性真实系统和非线性 EKF 预测各执行一次 RK4。不同样本
        % 状态不同，无法复用数值积分；但昂贵的 Jacobian 已完全移出循环。
        trueNext(:,sample)=rk4StepLBTS(trueState(:,k,sample),uk,dt(k),model) + Qsqrt{k}*processNoise(:,k,sample);
        estimatePrior(:,sample)=rk4StepLBTS(estimatedState(:,k,sample),uk,dt(k),model);
    end

    C=controller.C{k};
    measurement=C*trueNext + Rsqrt{k}*reshape(measurementNoise(:,k,:),measurementDimension,runs);
    innovation=measurement-C*estimatePrior;
    % 观测矩阵当前是状态分量选择矩阵。若测量含 roll/pitch/yaw，创新量
    % 必须按角度包络，否则跨 +/-pi 时会产生接近 2*pi 的虚假残差。
    angleRows=any(abs(C(:,4:6))>0,2);
    innovation(angleRows,:)=wrapToPiLocal(innovation(angleRows,:));
    estimateNext=estimatePrior+controller.L{k}*innovation;

    trueState(:,k+1,:)=reshape(trueNext,nx,1,runs);
    estimatedState(:,k+1,:)=reshape(estimateNext,nx,1,runs);
end

collisionMask=false(1,runs);
for sample=1:runs
    collisionMask(sample)=~isMeanTrajectoryCollisionFreeLBTS(trueState(:,:,sample),world);
end

% 统计相对名义轨迹的状态误差。姿态误差使用圆周差，避免角度分支影响
% 均值、RMSE 和经验协方差。
meanError=zeros(nx,steps+1);
empiricalCovariance=cell(1,steps+1);
positionRMSE=zeros(1,steps+1);
for k=1:steps+1
    errors=stateDifferenceMatrix(reshape(trueState(:,k,:),nx,runs),xbar(:,k));
    meanError(:,k)=mean(errors,2);
    positionRMSE(k)=sqrt(mean(sum(errors(1:3,:).^2,1)));
    if runs>1
        empiricalCovariance{k}=cov(errors',1);
    else
        empiricalCovariance{k}=zeros(nx);
    end
end

simulation=struct();
simulation.trueTrajectories=trueState;
simulation.estimatedTrajectories=estimatedState;
simulation.controlTrajectories=control;
simulation.nominalTrajectory=xbar;
simulation.nominalControl=ubar;
simulation.time=time;
simulation.collisionMask=collisionMask;
simulation.collisionRate=mean(collisionMask);
simulation.meanStateError=meanError;
simulation.positionRMSE=positionRMSE;
simulation.empiricalCovariance=empiricalCovariance;
simulation.saturatedElementCount=saturatedElementCount;
simulation.controlSaturationRate=saturatedElementCount/numel(control);
simulation.settings=settings;
simulation.figure=[];

if settings.plot
    simulation.figure=plotMonteCarloResult( ...
        simulation,result,settings);
end
if settings.verbose
    finalPositionRMSE=positionRMSE(end);
    fprintf(['BlueROV2 MC: runs=%d, collision=%d (%.2f%%), ' ...
        'final position RMSE=%.4g m, saturated controls=%.3f%%.\n'], ...
        runs,nnz(collisionMask),100*simulation.collisionRate, ...
        finalPositionRMSE,100*simulation.controlSaturationRate);
end
end

function settings=completeMCOptions(user)
defaults=struct('numRuns',40,'rngSeed',0,'plot',true,'plotEstimatedMean',true,'plotCovariance',true,'covarianceSigma',3,'maxCovarianceEllipsoids',20,'maxTrajectoriesToPlot',inf,'enforceThrustLimits',true,'figureHandle',[],'verbose',true);
settings=defaults;
names=fieldnames(user);
for k=1:numel(names), settings.(names{k})=user.(names{k}); end
assert(settings.numRuns>=1 && settings.numRuns==round(settings.numRuns),'MC_path:Options','numRuns 必须为正整数。');
assert(settings.covarianceSigma>0 && settings.maxCovarianceEllipsoids>=1 && settings.maxTrajectoriesToPlot>=1,'MC_path:Options','绘图数量和 covarianceSigma 必须为正数。');
end

function ensureKPRMPath(result)
if exist('bluerov2Dynamics','file')==2, return; end
if isfolder(result.options.kprmPath), addpath(char(result.options.kprmPath),'-begin'); end
assert(exist('bluerov2Dynamics','file')==2,'MC_path:KPRMPath','找不到 bluerov2Dynamics.m，请设置 result.options.kprmPath。');
end

function [xbar,ubar,time]=validateNominalResult(result)
xbar=result.nominalTrajectory;
ubar=result.controlTrajectory;
time=reshape(result.time,1,[]);
steps=size(ubar,2);
assert(size(xbar,1)==12 && size(ubar,1)==8 && size(xbar,2)==steps+1 && numel(time)==steps+1 && all(diff(time)>0),'MC_path:NominalData','规划结果的状态、控制或时间维度不一致。');
assert(numel(result.covarianceSequence)==steps+1 && numel(result.estimationCovarianceSequence)==steps+1,'MC_path:CovarianceData','规划结果缺少完整协方差序列。');
end

function controller=prepareController(result,steps)
assert(numel(result.feedbackGains)==steps && numel(result.kalmanGains)==steps && numel(result.linearizationA)==steps && numel(result.linearizationB)==steps && numel(result.measurementMatrices)==steps && numel(result.measurementCovariances)==steps,'MC_path:ControllerData','result 中的控制器序列不完整，请重新运行 LBTS。');
controller=struct('K',{result.feedbackGains},'L',{result.kalmanGains},'A',{result.linearizationA},'B',{result.linearizationB},'C',{result.measurementMatrices},'R',{result.measurementCovariances},'edgeStepCounts',result.edgeStepCounts,'gainSource','LBTS final-path cache');
end

function difference=stateDifferenceMatrix(states,reference)
difference=states-reference(:);
difference(4:6,:)=wrapToPiLocal(difference(4:6,:));
end

function angle=wrapToPiLocal(angle)
angle=atan2(sin(angle),cos(angle));
end

function root=covarianceSquareRoot(P)
P=nearestPSD(P,0);
[root,flag]=chol(P,'lower');
if flag~=0
    [V,D]=eig(P);
    root=V*diag(sqrt(max(real(diag(D)),0)));
end
end

function fig=plotMonteCarloResult(simulation,result,settings)
if ~isempty(settings.figureHandle) && ...
        isgraphics(settings.figureHandle,'figure')
    fig=settings.figureHandle; clf(fig);
else
    fig=figure('Color','w','Name','BlueROV2 LBTS Monte Carlo');
end
ax=axes('Parent',fig); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
view(ax,3); set(ax,'ZDir','reverse');
xlabel(ax,'North [m]'); ylabel(ax,'East [m]'); zlabel(ax,'Depth [m]');
drawWorld(ax,result.world);

X=simulation.nominalTrajectory;
plot3(ax,X(1,:),X(2,:),X(3,:),'b-','LineWidth',2.4, ...
    'DisplayName','nominal LBTS trajectory');
runs=size(simulation.trueTrajectories,3);
plotCount=min(runs,floor(settings.maxTrajectoriesToPlot));
for sample=1:plotCount
    path=simulation.trueTrajectories(1:3,:,sample);
    if simulation.collisionMask(sample)
        color=[.90 .12 .10]; width=1.0;
    else
        color=[.55 .55 .55]; width=.65;
    end
    plot3(ax,path(1,:),path(2,:),path(3,:),'-', ...
        'Color',color,'LineWidth',width,'HandleVisibility','off');
end
plot3(ax,nan,nan,nan,'-','Color',[.55 .55 .55], ...
    'DisplayName','collision-free MC trajectory');
plot3(ax,nan,nan,nan,'-','Color',[.90 .12 .10], ...
    'DisplayName','colliding MC trajectory');

if settings.plotEstimatedMean
    estimateMean=mean(simulation.estimatedTrajectories,3);
    plot3(ax,estimateMean(1,:),estimateMean(2,:),estimateMean(3,:), ...
        'm--','LineWidth',1.4,'DisplayName','mean state estimate');
end
if settings.plotCovariance
    count=numel(result.covarianceSequence);
    stride=max(1,ceil(count/settings.maxCovarianceEllipsoids));
    for k=1:stride:count
        drawCovarianceEllipsoid(ax,X(1:3,k), ...
            result.covarianceSequence{k}(1:3,1:3), ...
            settings.covarianceSigma);
    end
end
scatter3(ax,X(1,1),X(2,1),X(3,1),70,[.05 .65 .15], ...
    'filled','MarkerEdgeColor','k','DisplayName','start');
scatter3(ax,X(1,end),X(2,end),X(3,end),90,[.75 .08 .75], ...
    'p','filled','MarkerEdgeColor','k','DisplayName','goal');
b=result.world.positionBounds;
xlim(ax,b(1,:)); ylim(ax,b(2,:)); zlim(ax,b(3,:));
title(ax,sprintf(['BlueROV2 nonlinear Monte Carlo: %d runs, ' ...
    'collision %.1f%%'],runs,100*simulation.collisionRate));
legend(ax,'Location','northeastoutside');
end

function drawWorld(ax,world)
for k=1:numel(world.obstacles)
    obstacle=world.obstacles(k);
    switch lower(obstacle.type)
        case 'sphere'
            [x,y,z]=sphere(22);
            surf(ax,obstacle.center(1)+obstacle.radius*x, ...
                obstacle.center(2)+obstacle.radius*y, ...
                obstacle.center(3)+obstacle.radius*z, ...
                'FaceColor',[.35 .35 .35],'FaceAlpha',.45, ...
                'EdgeColor','none','HandleVisibility','off');
        case 'box'
            c=obstacle.center(:); h=obstacle.size(:)/2;
            vertices=[-1 -1 -1;1 -1 -1;1 1 -1;-1 1 -1; ...
                -1 -1 1;1 -1 1;1 1 1;-1 1 1].*h'+c';
            faces=[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6; ...
                3 4 8 7;4 1 5 8];
            patch(ax,'Vertices',vertices,'Faces',faces, ...
                'FaceColor',[.35 .35 .35],'FaceAlpha',.45, ...
                'EdgeColor',[.2 .2 .2],'HandleVisibility','off');
        otherwise
            error('MC_path:Obstacle','未知障碍物类型：%s',obstacle.type);
    end
end
plot3(ax,nan,nan,nan,'s','MarkerFaceColor',[.35 .35 .35], ...
    'MarkerEdgeColor',[.2 .2 .2],'DisplayName','obstacle');
end

function drawCovarianceEllipsoid(ax,center,P,sigma)
P=nearestPSD(P,0);
[V,D]=eig(P);
radii=sigma*sqrt(max(real(diag(D)),0));
[x,y,z]=sphere(10);
points=V*diag(radii)*[x(:)';y(:)';z(:)']+center(:);
x=reshape(points(1,:),size(x));
y=reshape(points(2,:),size(y));
z=reshape(points(3,:),size(z));
surf(ax,x,y,z,'FaceColor',[.10 .65 .85],'FaceAlpha',.08, ...
    'EdgeColor','none','HandleVisibility','off');
end
