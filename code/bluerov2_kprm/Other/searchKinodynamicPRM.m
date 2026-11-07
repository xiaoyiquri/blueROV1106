function result = searchKinodynamicPRM(roadmap, queryInfo, currentWorld)
%SEARCHKINODYNAMICPRM Filter obstacles, search the graph, and join trajectories.
%
% 这是在线查询的第二步。输入必须是 attachQueryEndpoints 返回的临时
% roadmap 和 queryInfo。本函数不会再次添加端点或求解端点局部边，而是：
%   1) 按 currentWorld 过滤全部预计算边；
%   2) 在临时无碰撞邻接表上运行有向 Dijkstra；
%   3) 拼接最短路径上各条边的状态、控制和累计时间。

if nargin < 2 || isempty(queryInfo)
    assert(isfield(roadmap, 'queryInfo'), ...
        'queryInfo is required. Run attachQueryEndpoints first.');
    queryInfo = roadmap.queryInfo;
end
if nargin < 3 || isempty(currentWorld)
    if isfield(queryInfo, 'world') && ~isempty(queryInfo.world)
        currentWorld = queryInfo.world;
    else
        currentWorld = roadmap.param.world;
    end
end
roadmap.param.world = currentWorld;

startIdx = queryInfo.startIdx;
goalIdx = queryInfo.goalIdx;
assert(startIdx <= size(roadmap.Vertices, 1) && ...
       goalIdx <= size(roadmap.Vertices, 1), ...
    'queryInfo endpoint indices do not match this roadmap.');

[searchEdges, searchCosts, blockedEdgeCount] = ...
    collisionFilteredAdjacency(roadmap, currentWorld);
[path, cost, distance] = dijkstraDirected( ...
    searchEdges, searchCosts, startIdx, goalIdx);

result = struct('success', ~isempty(path), 'path', path, 'cost', cost, ...
    'distance', distance, 'states', [], 'controls', [], 'time', [], ...
    'blockedEdgeCount', blockedEdgeCount, 'queryInfo', queryInfo, ...
    'roadmap', roadmap);
if isempty(path)
    warning('searchKinodynamicPRM:NoPath', ...
        'No collision-free directed kinodynamic path connects start to goal.');
    return
end

timeOffset = 0;
for k = 1:numel(path)-1
    edge = getKinodynamicEdge(roadmap,path(k),path(k+1));
    x = edge{1};
    u = edge{3};
    info = edge{4};
    if k > 1
        % 相邻边共享端点，删除后续边的第一列，避免状态和时间重复。
        x = x(:, 2:end);
        edgeTime = info.time(2:end);
    else
        edgeTime = info.time;
    end
    result.states = [result.states, x];
    result.controls = [result.controls, u];
    result.time = [result.time, timeOffset+edgeTime];
    timeOffset = timeOffset + info.T;
end
end
