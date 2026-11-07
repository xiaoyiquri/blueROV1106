function valid = isPositionCollisionFree(position, world)
%ISPOSITIONCOLLISIONFREE Spherical-robot test against bounds and obstacles.
%
% ROV 被半径 world.robotRadius 的球体包络，因此检测只依赖位置，不依赖
% 姿态。这比点机器人安全，也比每次做网格碰撞快；代价是狭窄通道中
% 可能拒绝实际可通过的姿态。返回 true 表示球体完全处于自由空间。

p = position(:);
r = world.robotRadius;
b = world.positionBounds;
% 将工作空间边界向内收缩 r，等价于用机器人球体检测原始边界。
valid = all(p >= b(:, 1)+r) && all(p <= b(:, 2)-r);
if ~valid
    return
end

for k = 1:numel(world.obstacles)
    obstacle = world.obstacles(k);
    switch lower(obstacle.type)
        case 'sphere'
            % 两球不相交要求球心距离大于半径之和。
            valid = norm(p-obstacle.center(:)) > obstacle.radius + r;
        case 'box'
            % 将轴对齐盒体沿三个方向膨胀 r，再把机器人视为质点。
            % 若至少一个坐标轴落在膨胀盒外，则该点不在盒内。
            halfSize = obstacle.size(:)/2 + r;
            valid = any(abs(p-obstacle.center(:)) > halfSize);
        otherwise
            error('Unknown obstacle type: %s', obstacle.type);
    end
    if ~valid
        return
    end
end
end
