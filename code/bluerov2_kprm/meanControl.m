function [xbar, Mcost, N, ubar, info] = meanControl(xbar0, xbarf, param)
%MEANCONTROL 可复用的 BlueROV2 Heavy 两点时间/控制最优连接器。
%
% 调用接口与原 double-integrator 版本保持一致：
%   [xbar,Mcost,N] = meanControl(x0,xf,param)
%   [xbar,Mcost,N,ubar,info] = meanControl(x0,xf,param)
%
% 非线性规划使用 CasADi Opti + IPOPT 和 RK4 直接多重打靶。与逐边重新
% 创建符号图不同，本实现把首末状态声明为 Opti parameter，并把完整
% 求解器保存在 persistent 缓存中。只要 model/ocp 参数未变，后续边只
% 更新参数值和数值初值，不再重复展开 80 次动力学符号表达式。
%
% 决策变量：X(12,N+1)、U(8,N)、自由终止时间 T；dt=T/N。
% 目标函数：
%   J = wTime*T + dt*sum(U_k'*R*U_k)
%       + wDeltaU*sum(||U_{k+1}-U_k||^2).
% 约束包括严格首末状态、RK4 动力学、推力/速度/pitch 上下界。

import casadi.*

callClock = tic;
xbar0 = xbar0(:);
xbarf = xbarf(:);
assert(numel(xbar0)==12 && numel(xbarf)==12, ...
    'BlueROV2 state must have 12 elements.');
param = completeParameters(param);

% 欧拉角终点选择与起点最近的 2*pi 等价分支，避免跨包络边界绕行。
target = xbarf;
target(4:6) = xbar0(4:6) + wrapToPiLocal(xbarf(4:6)-xbar0(4:6));
N = param.ocp.N;

persistent cache
solverReused = ~isempty(cache) && isequaln(cache.model,param.model) && ...
    isequaln(cache.ocp,param.ocp);
if ~solverReused
    setupClock = tic;
    cache = createReusableSolver(param);
    setupSeconds = toc(setupClock);
else
    setupSeconds = 0;
end

opti = cache.opti;
opti.set_value(cache.initialState,xbar0);
opti.set_value(cache.targetState,target);

% 每次查询均给出与当前端点匹配的稳定初值。persistent 缓存的目的主要
% 是复用符号图；不盲目沿用上一条可能相距很远的边的数值解。
alpha = linspace(0,1,N+1);
Xguess = xbar0 + (target-xbar0)*alpha;
distance = norm(target(1:3)-xbar0(1:3));
Tguess = min(param.ocp.Tmax,max(param.ocp.Tmin, ...
    distance/max(param.ocp.vNominal,1.0e-3)));
hoverTau = fullRestoringAtState(Xguess(:,1),param.model);
hoverU = param.model.allocationPseudoInverse*hoverTau;
hoverU = min(param.model.thrustMax,max(param.model.thrustMin,hoverU));
opti.set_initial(cache.X,Xguess);
opti.set_initial(cache.U,repmat(hoverU,1,N));
opti.set_initial(cache.Tfinal,Tguess);

xbar=[]; ubar=[]; Mcost=inf;
info=struct('success',false,'returnStatus','','T',nan,'time',[], ...
    'cost',inf,'message','','solverReused',solverReused, ...
    'setupSeconds',setupSeconds,'solveSeconds',nan,'totalSeconds',nan);
try
    solveClock=tic;
    sol=opti.solve();
    info.solveSeconds=toc(solveClock);
    xbar=full(sol.value(cache.X));
    ubar=full(sol.value(cache.U));
    Mcost=full(sol.value(cache.objective));
    info.success=true;
    info.T=full(sol.value(cache.Tfinal));
    info.time=linspace(0,info.T,N+1);
    info.cost=Mcost;
    stats=sol.stats();
    if isfield(stats,'return_status')
        info.returnStatus=stats.return_status;
    else
        info.returnStatus='Solve_Succeeded';
    end
catch ME
    info.message=ME.message;
    try
        stats=opti.stats();
        if isfield(stats,'return_status')
            info.returnStatus=stats.return_status;
        end
    catch
        info.returnStatus='exception_before_solver_status';
    end
end
info.totalSeconds=toc(callClock);
end

function cache=createReusableSolver(param)
%CREATEREUSABLESOLVER 仅在参数变化时构造一次完整 CasADi NLP。
import casadi.*
N=param.ocp.N; nx=12; nu=8;
opti=Opti();
X=opti.variable(nx,N+1);
U=opti.variable(nu,N);
Tfinal=opti.variable();
initialState=opti.parameter(nx,1);
targetState=opti.parameter(nx,1);
dt=Tfinal/N;

opti.subject_to(X(:,1)==initialState);
opti.subject_to(X(:,end)==targetState);
opti.subject_to(Tfinal>=param.ocp.Tmin);
opti.subject_to(Tfinal<=param.ocp.Tmax);
for k=1:N
    xk=X(:,k); uk=U(:,k);
    k1=bluerov2Dynamics(xk,           uk,param.model);
    k2=bluerov2Dynamics(xk+dt*k1/2,uk,param.model);
    k3=bluerov2Dynamics(xk+dt*k2/2,uk,param.model);
    k4=bluerov2Dynamics(xk+dt*k3,  uk,param.model);
    xnext=xk+dt*(k1+2*k2+2*k3+k4)/6;
    opti.subject_to(X(:,k+1)==xnext);
end

thrustLower=repmat(param.model.thrustMin,1,N);
thrustUpper=repmat(param.model.thrustMax,1,N);
opti.subject_to(U(:)>=thrustLower(:));
opti.subject_to(U(:)<=thrustUpper(:));
linearLimit=repmat(param.ocp.maxLinearVelocity,1,N+1);
angularLimit=repmat(param.ocp.maxAngularVelocity,1,N+1);
linearState=X(7:9,:); angularState=X(10:12,:);
opti.subject_to(linearState(:)>=-linearLimit(:));
opti.subject_to(linearState(:)<= linearLimit(:));
opti.subject_to(angularState(:)>=-angularLimit(:));
opti.subject_to(angularState(:)<= angularLimit(:));
pitchTrajectory=X(5,:)';
opti.subject_to(pitchTrajectory>=-param.ocp.maxPitch);
opti.subject_to(pitchTrajectory<= param.ocp.maxPitch);

effort=0;
for k=1:N
    effort=effort+U(:,k)'*param.ocp.R*U(:,k);
end
slew=0;
for k=1:N-1
    du=U(:,k+1)-U(:,k);
    slew=slew+du'*du;
end
objective=param.ocp.wTime*Tfinal+dt*effort+param.ocp.wDeltaU*slew;
opti.minimize(objective);

printLevel=0;
if param.ocp.verbose, printLevel=5; end
opts.expand=true;
opts.print_time=param.ocp.verbose;
opts.ipopt.print_level=printLevel;
opts.ipopt.max_iter=param.ocp.ipoptMaxIter;
opts.ipopt.tol=param.ocp.acceptableTol;
opts.ipopt.acceptable_tol=10*param.ocp.acceptableTol;
opts.ipopt.sb='yes';
opti.solver('ipopt',opts);
cache=struct('opti',opti,'X',X,'U',U,'Tfinal',Tfinal, ...
    'initialState',initialState,'targetState',targetState, ...
    'objective',objective,'model',param.model,'ocp',param.ocp);
end

function param=completeParameters(param)
% 旧 trajectoryGraph 的参数结构可能不含新预计算量；在内存中补齐。
if ~isfield(param,'model') || ~isfield(param,'ocp')
    defaults=bluerov2Parameters();
    if ~isfield(param,'model'), param.model=defaults.model; end
    if ~isfield(param,'ocp'), param.ocp=defaults.ocp; end
end
if ~isfield(param.model,'massMatrix') || ...
        ~isfield(param.model,'massMatrixInverse') || ...
        ~isfield(param.model,'allocationPseudoInverse')
    Srg=skew3(param.model.rg);
    Io=param.model.Ig-param.model.m*Srg*Srg;
    Mrb=[param.model.m*eye(3),-param.model.m*Srg; ...
         param.model.m*Srg,Io];
    param.model.massMatrix=Mrb+diag(param.model.addedMass);
    param.model.massMatrixInverse=param.model.massMatrix\eye(6);
    param.model.allocationPseudoInverse=pinv(param.model.allocation);
end
end

function a=wrapToPiLocal(a)
a=atan2(sin(a),cos(a));
end

function g=fullRestoringAtState(x,model)
% 在 nu=0、f=0 时 nu_dot=-M^{-1}g，反推出悬停恢复广义力。
xz=x; xz(7:12)=0;
xdot=bluerov2Dynamics(xz,zeros(8,1),model);
g=-model.massMatrix*xdot(7:12);
end

function S=skew3(a)
S=[0,-a(3),a(2);a(3),0,-a(1);-a(2),a(1),0];
end
