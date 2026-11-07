function valid = isPositionCollisionFreeLBTS(position, world)
%ISPOSITIONCOLLISIONFREELBTS 用包围球检测三维位置是否处于自由空间。
% 与 bluerov2_kprm/isPositionCollisionFree.m 的几何约定相同，但保留
% 本地副本使 LBTS 的风险检查不依赖 MATLAB 路径上的同名函数。

p = position(:);
r = world.robotRadius;
b = world.positionBounds;
valid = all(p >= b(:,1)+r) && all(p <= b(:,2)-r);
if ~valid, return; end

for k = 1:numel(world.obstacles)
    obstacle = world.obstacles(k);
    switch lower(obstacle.type)
        case 'sphere'
            valid = norm(p-obstacle.center(:)) > obstacle.radius + r;
        case 'box'
            halfSize = obstacle.size(:)/2 + r;
            valid = any(abs(p-obstacle.center(:)) > halfSize);
        otherwise
            error('LBTS:UnknownObstacle', '未知障碍物类型：%s', obstacle.type);
    end
    if ~valid, return; end
end
end
