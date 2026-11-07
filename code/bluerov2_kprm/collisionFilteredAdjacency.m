function [filteredEdges, filteredCosts, blockedEdgeCount] = ...
        collisionFilteredAdjacency(roadmap, world)
%COLLISIONFILTEREDADJACENCY Build a query-specific collision-free graph.
%
% 离线 roadmap 中的边只保证动力学可行，不包含任何具体障碍物信息。
% 本函数在在线查询时逐条检查已保存的状态轨迹，并返回仅包含当前地图
% 下无碰撞边的临时邻接表。roadmap 本身不会被修改，因此更换 world
% 后可以再次调用本函数，而不必重新求解离线局部最优控制问题。
%
% 输入：
%   roadmap  含 Edges、EdgesCost 和 EdgeData 的动力学路网；
%   world    当前查询地图，含边界、robotRadius、collisionStep、obstacles。
%
% 输出：
%   filteredEdges  与 roadmap.Edges 同形的无碰撞邻接表；
%   filteredCosts  与 filteredEdges 一一对应的边代价；
%   blockedEdgeCount 被当前边界或障碍物阻断的有向边数量。

filteredEdges = cell(size(roadmap.Edges));
filteredCosts = cell(size(roadmap.EdgesCost));
blockedEdgeCount = 0;

for i = 1:numel(roadmap.Edges)
    children = roadmap.Edges{i};
    costs = roadmap.EdgesCost{i};
    assert(numel(children) == numel(costs), ...
        'Edges{%d} and EdgesCost{%d} have inconsistent lengths.', i, i);

    keep = false(size(children));
    for k = 1:numel(children)
        edge = roadmap.EdgeData{i}{k};

        % 缺少轨迹数据的邻接记录无法执行碰撞检查，保守地视为阻断。
        if isempty(edge) || isempty(edge{1}) || ...
                ~isTrajectoryCollisionFree(edge{1}, world)
            blockedEdgeCount = blockedEdgeCount + 1;
            continue
        end
        keep(k) = true;
    end

    filteredEdges{i} = children(keep);
    filteredCosts{i} = costs(keep);
end
end
