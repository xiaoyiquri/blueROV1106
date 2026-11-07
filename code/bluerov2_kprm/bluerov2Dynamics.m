function xdot = bluerov2Dynamics(x, f, model)
%BLUEROV2DYNAMICS Full nonlinear 6-DOF BlueROV2 Heavy dynamics.
% Accepts numeric arrays or CasADi SX/MX symbols.
%
% 输入：
%   x     12x1 状态 [eta;nu]；
%   f      8x1 各推进器实际推力，单位 N；
%   model  bluerov2Parameters().model。
% 输出：
%   xdot  12x1 连续时间状态导数。
%
% eta=[N,E,D,phi,theta,psi] 使用 NED 惯性系和 ZYX 欧拉角；
% nu =[u,v,w,p,q,r] 使用机体系。动力学采用 Fossen/SNAME 形式：
%   eta_dot = J(eta)nu
%   M nu_dot + C(nu)nu + D(nu)nu + g(eta) = tau
%   tau = allocation*f
% 函数没有使用 if/abs 等不可微分分支，因此同一份代码既能数值仿真，
% 也能被 CasADi 自动微分并嵌入非线性规划。

eta = x(1:6);
nu  = x(7:12);
phi = eta(4);
theta = eta(5);
psi = eta(6);

R = rotationZYX(phi, theta, psi);
T = eulerRateMap(phi, theta);
% R 把机体系线速度转换到 NED；T 把机体系角速度转换为 ZYX 欧拉角率。
etaDot = [R, zerosLike(phi, 3, 3); zerosLike(phi, 3, 3), T] * nu;

% 刚体质量矩阵的参考点是机体系原点。Ig 给在重心处，Io 通过平行轴
% 定理转换到机体系原点；非零 rg 会同时引入平移/转动耦合块。
if isfield(model,'massMatrix') && isfield(model,'massMatrixInverse')
    M = model.massMatrix;
    Minv = model.massMatrixInverse;
else
    % 兼容旧 MAT 文件中的 model；新参数由 bluerov2Parameters 预计算。
    Srg = skew3(model.rg);
    Io = model.Ig - model.m * Srg * Srg;
    Mrb = [model.m * eye(3), -model.m * Srg; ...
           model.m * Srg,     Io];
    M = Mrb + diag(model.addedMass);
    Minv = M \ eye(6);
end

% Energy-consistent Coriolis/centripetal matrix for a constant partitioned
% mass matrix. Using total M includes rigid-body and added-mass coupling.
C = coriolisFromMass(M, nu);

% sqrt(nu^2+eps^2) 是 |nu| 的光滑近似。直接使用 abs 在零速度处不可微，
% 容易影响 IPOPT 的梯度/Hessian 计算。
absNu = sqrt(nu .* nu + model.smoothAbsEps^2);
damping = (model.linearDamping + model.quadraticDamping .* absNu) .* nu;
restoring = restoringForces(phi, theta, model);
tau = model.allocation * f;
% M 在当前模型中是常矩阵，所以逆矩阵只涉及普通 double，不进入 CasADi
% 符号求导图；实际辨识若得到随姿态变化的 M，应改回符号线性求解。
nuDot = Minv * (tau - C * nu - damping - restoring);

xdot = [etaDot; nuDot];
end

function R = rotationZYX(phi, theta, psi)
% 机体系到 NED 惯性系的 ZYX 旋转矩阵 Rz(psi)Ry(theta)Rx(phi)。
cphi = cos(phi); sphi = sin(phi);
cth = cos(theta); sth = sin(theta);
cpsi = cos(psi); spsi = sin(psi);
R = [cth*cpsi, sphi*sth*cpsi-cphi*spsi, cphi*sth*cpsi+sphi*spsi; ...
     cth*spsi, sphi*sth*spsi+cphi*cpsi, cphi*sth*spsi-sphi*cpsi; ...
     -sth,     sphi*cth,                   cphi*cth];
end

function T = eulerRateMap(phi, theta)
% [phi_dot theta_dot psi_dot]' = T(phi,theta)[p q r]'。
% 当 cos(theta)=0 时欧拉角参数化奇异；meanControl 将 pitch 限制在
% +/-maxPitch 内，默认 75 度，以避免数值接近该奇异点。
cphi = cos(phi); sphi = sin(phi);
cth = cos(theta); tth = tan(theta);
T = [1, sphi*tth, cphi*tth; ...
     0, cphi,     -sphi; ...
     0, sphi/cth, cphi/cth];
end

function C = coriolisFromMass(M, nu)
% 对称常质量矩阵的分块构造。该形式保证 C'=-C，因此理想情况下
% nu'*C*nu=0：科氏/向心项只重新分配动能，不凭空耗散或增加能量。
v = nu(1:3);
w = nu(4:6);
p = M(1:3, 1:3) * v + M(1:3, 4:6) * w;
h = M(4:6, 1:3) * v + M(4:6, 4:6) * w;
Z = zerosLike(nu(1), 3, 3);
C = [Z,        -skew3(p); ...
     -skew3(p), -skew3(h)];
end

function g = restoringForces(phi, theta, model)
% 重力与浮力的静力恢复项。位置和 yaw 不影响静水恢复力；roll/pitch
% 以及 CG/CB 的相对位置决定扶正力矩。符号与主方程左端的 +g 一致。
cphi = cos(phi); sphi = sin(phi);
cth = cos(theta); sth = sin(theta);
W = model.W; B = model.B;
xg = model.rg(1); yg = model.rg(2); zg = model.rg(3);
xb = model.rb(1); yb = model.rb(2); zb = model.rb(3);
g = [ ...
    (W-B)*sth; ...
    -(W-B)*cth*sphi; ...
    -(W-B)*cth*cphi; ...
    -(yg*W-yb*B)*cth*cphi + (zg*W-zb*B)*cth*sphi; ...
    (zg*W-zb*B)*sth + (xg*W-xb*B)*cth*cphi; ...
    -(xg*W-xb*B)*cth*sphi - (yg*W-yb*B)*sth];
end

function S = skew3(a)
% 反对称矩阵满足 skew3(a)*b = cross(a,b)。
S = [0, -a(3), a(2); a(3), 0, -a(1); -a(2), a(1), 0];
end

function Z = zerosLike(a, m, n)
% repmat preserves a CasADi scalar type and is numeric for doubles.
Z = repmat(0 * a, m, n);
end
