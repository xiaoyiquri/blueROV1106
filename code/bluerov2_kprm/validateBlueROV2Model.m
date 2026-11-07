function report = validateBlueROV2Model(param)
%VALIDATEBLUEROV2MODEL Fast numeric consistency checks (CasADi not needed).
%
% 该函数只检查模型本身的代数性质，不求解 OCP：
%   allocationRank=6      表示 8 路推进器可瞬时生成任意 6 维广义力；
%   M=M'>0                是物理质量矩阵的基本要求；
%   方向向量必须为单位向量；
%   伪逆计算的悬停推力应抵消恢复力且位于推力范围内。
% 这些检查通过不代表水动力参数已针对实机辨识，但能发现常见的矩阵
% 维度、推进器符号和坐标系实现错误。

if nargin < 1, param = bluerov2Parameters(); end
m = param.model;
Srg = skew3(m.rg);
Io = m.Ig - m.m*Srg*Srg;
M = [m.m*eye(3), -m.m*Srg; m.m*Srg, Io] + diag(m.addedMass);

report.allocationRank = rank(m.allocation);
report.massSymmetryError = norm(M-M', 'fro');
report.minimumMassEigenvalue = min(eig(M));
report.thrusterDirectionNormError = max(abs( ...
    vecnorm(m.thrusterDirection, 2, 1)-1));

x = zeros(12, 1);
x(3) = 2;
% 平姿态、零速度、零控制下的加速度只由恢复力产生。用质量矩阵反推
% 恢复力，再经分配矩阵伪逆得到最小二范数悬停推力。
xdotZero = bluerov2Dynamics(x, zeros(8, 1), m);
restoring = -M * xdotZero(7:12);
hover = pinv(m.allocation) * restoring;
xdot = bluerov2Dynamics(x, hover, m);
report.hoverAccelerationNorm = norm(xdot(7:12));
report.hoverWithinThrustLimits = all(hover >= m.thrustMin) && ...
    all(hover <= m.thrustMax);

assert(report.allocationRank == 6, 'Thruster allocation must have rank 6.');
assert(report.massSymmetryError < 1e-10, 'Mass matrix is not symmetric.');
assert(report.minimumMassEigenvalue > 0, 'Mass matrix is not positive definite.');
assert(report.thrusterDirectionNormError < 1e-12, ...
    'A thruster direction is not a unit vector.');
assert(report.hoverAccelerationNorm < 1e-10, ...
    'Computed hover force does not produce equilibrium.');
disp(report)
end

function S = skew3(a)
S = [0, -a(3), a(2); a(3), 0, -a(1); -a(2), a(1), 0];
end
