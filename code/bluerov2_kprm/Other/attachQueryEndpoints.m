function [roadmap, queryInfo] = ...
        attachQueryEndpoints(roadmapOrFile, startState, goalState, currentWorld)
%ATTACHQUERYENDPOINTS Add query endpoints and their local edges to a roadmap.
%
% 这是在线查询的第一步，只完成以下工作：
%   1) 把 startState 和 goalState 添加为两个临时图节点；
%   2) 在 query.connectionRadius 内选取候选路网节点；
%   3) 调用 meanControl 求解端点与候选节点之间的局部有向边；
%   4) 仅接受在 currentWorld 中无碰撞的端点连接；
%   5) 返回端点编号和原始离线节点数，供绘图与第二步图搜索使用。
%
% 本函数不执行 Dijkstra，也不对全部离线边做碰撞过滤。因此调用后
% 可以先用 plotKinodynamicPRM(roadmap, [], queryInfo) 检查端点连接。

if ischar(roadmapOrFile) || isstring(roadmapOrFile)
    roadmap = loadKinodynamicRoadmap(roadmapOrFile);
else
    roadmap = normalizeKinodynamicRoadmap(roadmapOrFile);
end

if nargin < 4 || isempty(currentWorld)
    currentWorld = roadmap.param.world;
end
roadmap.param.world = currentWorld;

startState = startState(:)';
goalState = goalState(:)';
assert(numel(startState) == 12 && numel(goalState) == 12, ...
    'Start and goal must each contain 12 state elements.');
assert(isPositionCollisionFree(startState(1:3)', currentWorld), ...
    'Start state is in collision or outside the workspace.');
assert(isPositionCollisionFree(goalState(1:3)', currentWorld), ...
    'Goal state is in collision or outside the workspace.');

baseVertexCount = size(roadmap.Vertices, 1);
edgeCountBefore = roadmap.edge_count;
[roadmap, startIdx] = appendVertex(roadmap, startState);
[roadmap, goalIdx] = appendVertex(roadmap, goalState);

baseCandidates = 1:baseVertexCount;
% 单向查询只会使用 start->graph 和 graph->goal；不求解永远不会进入
% start 的入边或离开 goal 的出边，可把端点局部 NLP 尝试数约减半。
roadmap = connectEndpoint(roadmap, startIdx, baseCandidates, true, false);
roadmap = connectEndpoint(roadmap, goalIdx, baseCandidates, false, true);

% 起终点足够近时额外尝试直接连接。该边也属于第一步新增的查询边。
if norm(startState(1:3)-goalState(1:3)) <= ...
        roadmap.param.query.connectionRadius
    roadmap = tryInsert(roadmap, startIdx, goalIdx);
end

queryInfo = struct( ...
    'baseVertexCount', baseVertexCount, ...
    'startIdx', startIdx, ...
    'goalIdx', goalIdx, ...
    'startState', startState, ...
    'goalState', goalState, ...
    'addedEdgeCount', roadmap.edge_count-edgeCountBefore, ...
    'world', currentWorld);

% 把元数据同时保存在临时路网中，使绘图函数在省略第 3 个参数时也能
% 自动识别查询端点。它不改变离线 MAT 文件中的原始 roadmap。
roadmap.queryInfo = queryInfo;
end

function [roadmap, idx] = appendVertex(roadmap, state)
% 同时扩充所有图容器，保持节点数、邻接表和边数据矩阵尺寸一致。
idx = size(roadmap.Vertices, 1) + 1;
roadmap.Vertices(idx, :) = state;
roadmap.Edges{idx, 1} = [];
roadmap.Parents{idx, 1} = [];
roadmap.EdgesCost{idx, 1} = [];
roadmap.EdgeData{idx,1} = cell(1,0);
end

function roadmap = connectEndpoint(roadmap, endpoint, candidates, outgoing, incoming)
% 先按位置半径筛选，再按完整状态距离选择有限数量的在线候选节点。
% 固定 k 只用于控制在线查询的额外 NLP 次数，不影响离线全半径建图。
positionDistance = vecnorm( ...
    roadmap.Vertices(candidates, 1:3)-roadmap.Vertices(endpoint, 1:3), 2, 2);
keep = positionDistance <= roadmap.param.query.connectionRadius;

% 位于当前障碍物内部的节点不可能形成有效端点连接，提前排除可避免
% 对注定失败的候选调用 meanControl。
for k = 1:numel(candidates)
    if keep(k)
        keep(k) = isPositionCollisionFree( ...
            roadmap.Vertices(candidates(k), 1:3)', roadmap.param.world);
    end
end
candidates = candidates(keep);
if isempty(candidates)
    return
end

d = weightedStateDistance(roadmap.Vertices(endpoint, :), ...
    roadmap.Vertices(candidates, :), roadmap.param.metric.weights);
[~, order] = sort(d, 'ascend');
candidates = candidates(order(1:min( ...
    roadmap.param.query.kNeighbors, numel(order))));

for j = candidates(:)'
    if outgoing
        roadmap = tryInsert(roadmap, endpoint, j);
    end
    if incoming
        roadmap = tryInsert(roadmap, j, endpoint);
    end
end
end

function roadmap = tryInsert(roadmap, i, j)
% 求解并原子式添加一条在线有向边；求解失败或碰撞时路网保持不变。
if any(roadmap.Edges{i}==j)
    return
end
[x, c, N, u, info] = meanControl( ...
    roadmap.Vertices(i, :)', roadmap.Vertices(j, :)', roadmap.param);
if ~info.success || ~isfinite(c) || ...
        ~isTrajectoryCollisionFree(x, roadmap.param.world)
    return
end
roadmap.Edges{i}(end+1) = j;
roadmap.Parents{j}(end+1) = i;
roadmap.EdgesCost{i}(end+1) = c;
roadmap.EdgeData{i}{end+1} = {x, N, u, info};
roadmap.edge_count = roadmap.edge_count + 1;
end
