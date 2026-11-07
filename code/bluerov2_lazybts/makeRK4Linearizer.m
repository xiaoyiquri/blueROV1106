function linearizer = makeRK4Linearizer(model, settings)
%MAKERK4LINEARIZER 构造离散 BlueROV2 动力学及其 Jacobian 计算器。
%
% 对每个名义点 (xbar_k,ubar_k) 需要
%   A_k = dF_RK4/dx,  B_k = dF_RK4/du,
% 其中 F_RK4 使用与 meanControl 相同的步长 dt。优先由 CasADi 精确自动
% 微分；若 CasADi 不可用且明确允许，则退化为中心有限差分。

import casadi.*

linearizer = struct('mode', '', 'F', [], 'model', model, ...
    'finiteDifferenceStep', settings.finiteDifferenceStep);
casadiError = '';
if settings.preferCasADi
    try
        x = SX.sym('x_lbts', 12, 1);
        u = SX.sym('u_lbts', 8, 1);
        dt = SX.sym('dt_lbts', 1, 1);
        xnext = rk4StepLBTS(x, u, dt, model);
        A = jacobian(xnext, x);
        B = jacobian(xnext, u);
        linearizer.F = Function('bluerov2_lbts_rk4_jacobian', ...
            {x,u,dt}, {xnext,A,B});
        linearizer.mode = 'casadi';
        return
    catch ME
        casadiError = ME.message;
    end
end

if settings.allowFiniteDifference
    linearizer.mode = 'finite-difference';
    warning('LBTS:FiniteDifferenceFallback', ...
        ['CasADi 自动微分线性化不可用，正在使用明显较慢的中心有限差分。' ...
         ' 原因：%s'],casadiError);
else
    error('LBTS:CasADiUnavailable', ...
        '无法建立 CasADi 离散线性化，且有限差分已禁用：%s', casadiError);
end
end
