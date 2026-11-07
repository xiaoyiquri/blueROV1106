function xnext = rk4StepLBTS(x, u, dt, model)
%RK4STEPLBTS 与 KPRM meanControl 完全一致的零阶保持 RK4 离散化。
% x、u 可以是 double，也可以是 CasADi SX/MX；这使数值仿真与自动
% 微分共用同一条离散动力学，避免线性化模型和名义轨迹积分不一致。

k1 = bluerov2Dynamics(x,             u, model);
k2 = bluerov2Dynamics(x + dt*k1/2,  u, model);
k3 = bluerov2Dynamics(x + dt*k2/2,  u, model);
k4 = bluerov2Dynamics(x + dt*k3,    u, model);
xnext = x + dt * (k1 + 2*k2 + 2*k3 + k4) / 6;
end
