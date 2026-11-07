function valid = isTrajectoryCollisionFree(xbar, world)
%ISTRAJECTORYCOLLISIONFREE Densified position-only collision check.
% The vehicle is conservatively enclosed by world.robotRadius.
%
% OCP 只在 N+1 个打靶节点给出状态。如果只检查这些节点，两个节点间的
% 曲线可能穿过很薄的障碍物。这里在每段位置弦线上按 collisionStep
% 加密。由于局部边较短且 RK4 节点较密，线性弦段是实用的近似；若
% 障碍物很小，应进一步减小 collisionStep 或在 OCP 内增加路径约束。

if isempty(xbar) || size(xbar, 1) < 3
    valid = false;
    return
end
valid = true;
for k = 1:size(xbar, 2)-1
    a = xbar(1:3, k);
    b = xbar(1:3, k+1);
    n = max(1, ceil(norm(b-a)/world.collisionStep));
    % j=0:n 同时包含两端点。重复检查相邻段公共端点开销很小，并能
    % 保持代码直接、无遗漏。
    for j = 0:n
        p = a + (j/n)*(b-a);
        if ~isPositionCollisionFree(p, world)
            valid = false;
            return
        end
    end
end
end
