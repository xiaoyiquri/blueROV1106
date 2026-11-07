function [xnext, A, B] = evaluateRK4Linearization(linearizer, x, u, dt)
%EVALUATERK4LINEARIZATION 在一个名义离散点求 F、A、B。

x = x(:); u = u(:);
if strcmp(linearizer.mode, 'casadi')
    [xnext, A, B] = linearizer.F(x, u, dt);
    xnext = full(xnext); A = full(A); B = full(B);
    return
end

% 中心差分是无 CasADi 环境下的备用路径。步长按变量量级缩放，避免
% 对大位置坐标使用过小扰动，也避免在接近零的速度上发生消减误差。
model = linearizer.model;
h0 = linearizer.finiteDifferenceStep;
xnext = rk4StepLBTS(x, u, dt, model);
A = zeros(12, 12);
B = zeros(12, 8);
for i = 1:12
    h = h0 * max(1, abs(x(i)));
    dx = zeros(12,1); dx(i) = h;
    fp = rk4StepLBTS(x+dx, u, dt, model);
    fm = rk4StepLBTS(x-dx, u, dt, model);
    A(:,i) = (fp-fm)/(2*h);
end
for i = 1:8
    h = h0 * max(1, abs(u(i)));
    du = zeros(8,1); du(i) = h;
    fp = rk4StepLBTS(x, u+du, dt, model);
    fm = rk4StepLBTS(x, u-du, dt, model);
    B(:,i) = (fp-fm)/(2*h);
end
end
