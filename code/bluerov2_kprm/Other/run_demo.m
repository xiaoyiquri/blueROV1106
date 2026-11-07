%% RUN_DEMO 加载 graph_heuristic_bluerov2 生成的 trajectoryGraph
clear; clc;

% Add CasADi before running if it is not installed globally, for example:
addpath('C:\Users\dlzhe\Documents\MATLAB\casadi-3.7.2-windows64-matlab2018b')
assert(exist('casadi.Opti', 'class') == 8, ...
    'CasADi is not on the MATLAB path. Add the CasADi MATLAB folder first.');

thisFolder = fileparts(mfilename('fullpath'));
graphFile = fullfile(thisFolder, 'trajectoryGraph.mat');
assert(isfile(graphFile), ...
    ['找不到 trajectoryGraph.mat。请先运行 graph_heuristic_bluerov2.m ' ...
     '生成并校验名义轨迹图。']);
savedVariables = who('-file',graphFile);
if ismember('trajectoryGraph',savedVariables)
    savedGraph = load(graphFile,'trajectoryGraph');
    roadmap = savedGraph.trajectoryGraph;
elseif ismember('roadmap',savedVariables)
    savedGraph = load(graphFile,'roadmap');
    roadmap = savedGraph.roadmap;
else
    error('MAT 文件中没有 trajectoryGraph 或 roadmap 结构体。');
end
validateTrajectoryGraphInterface(roadmap);
param = roadmap.param;

% 障碍物只在下面的在线查询阶段生效。可在不重建 roadmap 的情况下
% 修改 param.world.obstacles，并把新的 world 作为第 4 个参数传入。

% [N E depth roll pitch yaw u v w p q r]
% 起终点均静止，但目标 yaw=pi/2，因此规划器还必须安排姿态变化。
start = [ -2,  -1, 3.5, 0, 0, pi/2, zeros(1, 6)];
goal  = [3, 1, 1.5, 0, 0, 0, zeros(1, 6)];

%% Query step 1: attach start/goal and inspect their local connections
[queryRoadmap, queryInfo] = attachQueryEndpoints( ...
    roadmap, start, goal, param.world);
fprintf('Attached query endpoints: start=%d, goal=%d, new edges=%d\n', ...
    queryInfo.startIdx, queryInfo.goalIdx, queryInfo.addedEdgeCount);
plotKinodynamicPRM(queryRoadmap, [], queryInfo);
title('Step 1: roadmap with query endpoints and connection edges');

%% Query step 2: filter obstacles and search the directed graph
result = searchKinodynamicPRM(queryRoadmap, queryInfo, param.world);

fprintf('Query success: %d, cost: %.6g, blocked edges: %d\n', ...
    result.success, result.cost, result.blockedEdgeCount);
plotKinodynamicPRM(result.roadmap, result.path, queryInfo);
title('Step 2: collision-free shortest kinodynamic path');

if result.success
    % 状态时间轴有“总控制区间数+1”个点；controls 每列在对应区间恒定。
    figure;
    tiledlayout(2, 1);
    nexttile;
    plot(result.time, result.states(1:3, :)', 'LineWidth', 1.2);
    grid on; ylabel('position [m]'); legend('N', 'E', 'D');
    nexttile;
    stairs(result.controls', 'LineWidth', 1.0);
    grid on; xlabel('control interval'); ylabel('thruster force [N]');
end
