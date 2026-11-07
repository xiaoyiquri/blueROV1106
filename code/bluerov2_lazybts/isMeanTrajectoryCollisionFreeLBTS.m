function valid = isMeanTrajectoryCollisionFreeLBTS(xbar, world)
%ISMEANTRAJECTORYCOLLISIONFREELBTS 惰性检查整条名义轨迹。
% 不只检查优化节点，还按 world.collisionStep 对相邻位置线性插值，避免
% 轨迹在两个 RK4 节点之间穿过薄障碍物。

valid = true;
for k = 1:size(xbar,2)-1
    a = xbar(1:3,k); b = xbar(1:3,k+1);
    count = max(1, ceil(norm(b-a)/world.collisionStep));
    for q = 0:count
        p = a + (q/count)*(b-a);
        if ~isPositionCollisionFreeLBTS(p, world)
            valid = false;
            return
        end
    end
end
if size(xbar,2) == 1
    valid = isPositionCollisionFreeLBTS(xbar(1:3,1), world);
end
end
