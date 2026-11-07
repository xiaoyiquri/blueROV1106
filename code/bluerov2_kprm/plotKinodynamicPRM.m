function plotKinodynamicPRM(roadmap, path, queryInfo)
%PLOTKINODYNAMICPRM Plot roadmap, query endpoints/connections, and a path.
%
% NED 坐标的 depth 向下为正，因此设置 ZDir='reverse'，视觉上深水区域
% 位于图的下方。默认颜色约定：
%   灰色点、浅绿色细线：原始离线路网节点和边；
%   绿色五角星：查询起点；紫色五角星：查询终点；
%   蓝色线：查询起终点与路网之间新求解的局部边；
%   红色粗线：第二步图搜索选中的最短动力学路径。

if nargin < 2, path = []; end
if nargin < 3 || isempty(queryInfo)
    if isfield(roadmap, 'queryInfo')
        queryInfo = roadmap.queryInfo;
    else
        queryInfo = [];
    end
end

figure; hold on; grid on; axis equal; view(3);
xlabel('North [m]'); ylabel('East [m]'); zlabel('Depth [m]');
set(gca, 'ZDir', 'reverse');
plotObstacles(roadmap.param.world);

if isempty(queryInfo)
    baseVertexCount = size(roadmap.Vertices, 1);
else
    baseVertexCount = queryInfo.baseVertexCount;
end
baseVertexCount = min(baseVertexCount, size(roadmap.Vertices, 1));
hRoadmapNodes = scatter3(roadmap.Vertices(1:baseVertexCount, 1), ...
    roadmap.Vertices(1:baseVertexCount, 2), ...
    roadmap.Vertices(1:baseVertexCount, 3), ...
    12, [0.2 0.2 0.2], 'filled', 'DisplayName', 'Roadmap nodes');

roadmapEdgeColor = [0.70 0.84 0.72];
queryEdgeColor = [0.00 0.45 0.85];
pathColor = [0.85 0.10 0.10];

for i = 1:numel(roadmap.Edges)
    for edgeIndex = 1:numel(roadmap.Edges{i})
        j = roadmap.Edges{i}(edgeIndex);
        edge = roadmap.EdgeData{i}{edgeIndex};
        if isempty(edge), continue; end
        x = edge{1};

        % 查询新边至少有一个端点是 startIdx 或 goalIdx。它们使用蓝色，
        % 从而与原始离线路网边清晰区分。
        isQueryEdge = ~isempty(queryInfo) && ( ...
            i == queryInfo.startIdx || i == queryInfo.goalIdx || ...
            j == queryInfo.startIdx || j == queryInfo.goalIdx);
        if isQueryEdge
            color = queryEdgeColor;
            width = 1.4;
        else
            color = roadmapEdgeColor;
            width = 0.5;
        end
        plot3(x(1, :), x(2, :), x(3, :), ...
            'Color', color, 'LineWidth', width, 'HandleVisibility', 'off');
    end
end

% 用单独的虚拟线对象提供稳定图例，避免每条边都出现在图例中。
hRoadmapEdges = plot3(nan, nan, nan, '-', 'Color', roadmapEdgeColor, ...
    'LineWidth', 0.8, 'DisplayName', 'Offline roadmap edges');
legendHandles = [hRoadmapNodes, hRoadmapEdges];

if ~isempty(queryInfo)
    hQueryEdges = plot3(nan, nan, nan, '-', 'Color', queryEdgeColor, ...
        'LineWidth', 1.4, 'DisplayName', 'Query connection edges');
    startPosition = roadmap.Vertices(queryInfo.startIdx, 1:3);
    goalPosition = roadmap.Vertices(queryInfo.goalIdx, 1:3);
    hStart = scatter3(startPosition(1), startPosition(2), startPosition(3), ...
        100, [0.00 0.65 0.20], 'p', 'filled', ...
        'MarkerEdgeColor', 'k', 'DisplayName', 'Query start');
    hGoal = scatter3(goalPosition(1), goalPosition(2), goalPosition(3), ...
        100, [0.65 0.10 0.80], 'p', 'filled', ...
        'MarkerEdgeColor', 'k', 'DisplayName', 'Query goal');
    legendHandles = [legendHandles, hQueryEdges, hStart, hGoal];
end

for k = 1:numel(path)-1
    edge = getKinodynamicEdge(roadmap,path(k),path(k+1));
    x = edge{1};
    plot3(x(1, :), x(2, :), x(3, :), '-', ...
        'Color', pathColor, 'LineWidth', 2.2, 'HandleVisibility', 'off');
end
if numel(path) >= 2
    hPath = plot3(nan, nan, nan, '-', 'Color', pathColor, ...
        'LineWidth', 2.2, 'DisplayName', 'Selected path');
    legendHandles = [legendHandles, hPath];
end
legend(legendHandles, 'Location', 'best');
end

function plotObstacles(world)
% 障碍物仅用于规划可视化；膨胀后的机器人安全边界不单独绘制。
for k = 1:numel(world.obstacles)
    o = world.obstacles(k);
    switch lower(o.type)
        case 'sphere'
            [x, y, z] = sphere(18);
            surf(o.center(1)+o.radius*x, o.center(2)+o.radius*y, ...
                o.center(3)+o.radius*z, 'FaceAlpha', 0.25, ...
                'EdgeColor', 'none', 'FaceColor', [0.2 0.4 0.8], ...
                'HandleVisibility', 'off');
        case 'box'
            % patch 需要 8 个顶点和 6 个四边形面；盒体与坐标轴对齐。
            lo = o.center(:)-o.size(:)/2;
            hi = o.center(:)+o.size(:)/2;
            vertices = [lo(1) lo(2) lo(3); hi(1) lo(2) lo(3); ...
                hi(1) hi(2) lo(3); lo(1) hi(2) lo(3); ...
                lo(1) lo(2) hi(3); hi(1) lo(2) hi(3); ...
                hi(1) hi(2) hi(3); lo(1) hi(2) hi(3)];
            faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; ...
                2 3 7 6; 3 4 8 7; 4 1 5 8];
            patch('Vertices', vertices, 'Faces', faces, ...
                'FaceAlpha', 0.25, 'FaceColor', [0.2 0.4 0.8], ...
                'EdgeColor', [0.2 0.3 0.5], 'HandleVisibility', 'off');
    end
end
end
