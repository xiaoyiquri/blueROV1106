function [Vertices, positions] = sampleBlueROV2States(param)
%SAMPLEBLUEROV2STATES Poisson-like 3-D pose/velocity sampling for the PRM.
%
% 输出：
%   positions : nPosition x 3 的无碰撞空间位置；
%   Vertices  : nState x 12 的完整动力学状态。每个位置的状态数等于
%               connectionRadius 内其他位置的数量，因此 nState 不固定。
%
% 与直接在 12 维空间做均匀随机采样相比，本函数先保证位置覆盖，再为
% 每个位置附加若干姿态/速度。这样能控制空间分辨率，也能避免绝大多数
% 高维样本之间因距离过远而无法被局部最优控制器连接。

rng(param.prm.rngSeed);
bounds = param.world.positionBounds;
nWanted = param.prm.numPositions;
eta = param.world.robotRadius;

% 离线路网必须与具体障碍物布置无关。采样时只保留工作空间边界和
% 机器人包围球半径，显式清空障碍物；同一份状态样本因而可用于不同
% 在线场景。障碍物碰撞统一在 queryKinodynamicPRM 中检查。
samplingWorld = param.world;
samplingWorld.obstacles = struct([]);

positions = zeros(nWanted, 3);
n = 0;
attempts = 0;
while n < nWanted && attempts < param.prm.maxSampleAttempts
    attempts = attempts + 1;
    q = bounds(:, 1)'+ eta + rand(1, 3) .* (bounds(:, 2)-bounds(:, 1) - eta)';
    % 离线阶段只检查工作空间边界，不检查任何障碍物。
    if ~isPositionCollisionFree(q', samplingWorld)
        continue
    end
    if n > 0
        % 使用平方距离可避免不必要的 sqrt；这是近似 Poisson-disk 采样。
        d2 = sum((positions(1:n, :) - q).^2, 2);
        if min(d2) < param.prm.minPositionSeparation^2
            continue
        end
    end
    n = n + 1;
    positions(n, :) = q;
end
positions = positions(1:n, :);
if n < nWanted
    warning('sampleBlueROV2States:Shortfall', ...
        'Accepted %d of %d requested positions.', n, nWanted);
end
assert(n >= 2, 'At least two collision-free positions are required.');

% 每个位置产生的状态数量由半径邻域内的邻点数量决定。最坏情况下，
% 每两个不同位置都互为邻居，因此最多生成 n*(n-1) 个有向状态。
% 先按该上界预分配，完成采样后再裁去未使用的行。
Vertices = zeros(n*(n-1), 12);
row = 0;
eps = 0.01;
for i = 1:n
    % 计算当前位置到所有位置样本的欧氏距离。
    delta = positions - positions(i, :);
    distance = sqrt(sum(delta.^2, 2));

    % 与原 SampleMultipleVel.m 的思想一致：选择全部距离小于阈值的
    % 邻点，不再排序后只取固定数量。distance>eps 排除当前位置自身。
    nearIdx = find(distance < param.prm.connectionRadius & distance > eps);

    % 每个邻近位置对应一个完整状态，线速度从当前位置指向该邻点。
    % 不再为每个位置额外插入零线速度、零角速度状态。
    for k = 1:numel(nearIdx)
        j = nearIdx(k);
        row = row + 1;
        dirNed = delta(j, :)' / distance(j);
        % 令机头大致指向邻近样本：yaw 对准水平投影，pitch 对准垂向
        % 分量。NED 的 z 轴向下，因此朝深处运动对应负 pitch。
        yaw = atan2(dirNed(2), dirNed(1));
        pitch = -atan2(dirNed(3), hypot(dirNed(1), dirNed(2)));
        pitch = min(param.prm.pitchRange(2), max(param.prm.pitchRange(1), pitch));
        roll = randomInRange(param.prm.rollRange);
        R = rotationZYXNumeric(roll, pitch, yaw);
        % 采样速度先在 NED 中沿 dirNed 定义，再转换成动力学使用的
        % 机体系线速度。角速度仅作小范围随机扰动。
        velBody = R' * (param.prm.sampleSpeed * dirNed);
        omega = param.prm.sampleAngularRate * (2*rand(3, 1)-1);
        Vertices(row, :) = [positions(i, :), roll, pitch, yaw, velBody', omega'];
    end
end

% 删除预分配但未使用的行。由于每个位置的邻点数量不同，最终节点数
% 一般不等于位置数的固定倍数。
Vertices = Vertices(1:row, :);
if isempty(Vertices)
    error('sampleBlueROV2States:NoVelocityStates', ...
        ['No position pairs are closer than prm.connectionRadius. ', ...
         'Increase connectionRadius or reduce minPositionSeparation.']);
end
end

function value = randomInRange(range)
% 在闭区间两端之间作均匀采样；端点被抽中的概率在浮点意义下可忽略。
value = range(1) + rand * (range(2)-range(1));
end

function R = rotationZYXNumeric(phi, theta, psi)
% 与 bluerov2Dynamics 中的旋转矩阵保持完全相同的坐标约定。
cphi = cos(phi); sphi = sin(phi);
cth = cos(theta); sth = sin(theta);
cpsi = cos(psi); spsi = sin(psi);
R = [cth*cpsi, sphi*sth*cpsi-cphi*spsi, cphi*sth*cpsi+sphi*spsi; ...
     cth*spsi, sphi*sth*spsi+cphi*cpsi, cphi*sth*spsi-sphi*cpsi; ...
     -sth, sphi*cth, cphi*cth];
end
