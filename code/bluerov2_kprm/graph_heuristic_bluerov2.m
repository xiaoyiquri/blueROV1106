%GRAPH_HEURISTIC_BLUEROV2 Script-style counterpart of graph_heuristic.m.
clear; clc;

% Add CasADi first if it is not installed globally:
addpath('C:\Users\dlzhe\Documents\MATLAB\casadi-3.7.2-windows64-matlab2018b')
param = bluerov2Parameters();
thisFolder = fileparts(mfilename('fullpath'));
graphFile = fullfile(thisFolder, 'trajectoryGraph.mat');

% 本脚本在末尾统一保存，以便保存前先执行 LazyBTS 接口校验。关闭
% buildKinodynamicPRM 的内部自动保存可避免同一个大 MAT 文件写两次。
param.prm.saveFile = '';

roadmap = buildKinodynamicPRM(param);

% 把实际输出文件名写回参数并同步到结构体。后续加载 MAT 后即可知道
% 图的来源；这一赋值不改变任何已经求出的顶点、边或最优轨迹。
param.prm.saveFile = graphFile;
roadmap.param = param;

% trajectoryGraph 是后续 LazyBTS 的统一变量名。只保存一份结构体，
% 其中 EdgeData{i}{k} 与 Edges{i}(k) 对齐，避免 n-by-n 空 cell 和顶层
% 兼容变量造成的重复存储；加载器仍能读取历史 Edges_data 文件。
trajectoryGraph = normalizeKinodynamicRoadmap(roadmap);
interfaceReport = validateTrajectoryGraphInterface(trajectoryGraph);
fprintf(['trajectoryGraph interface passed: %d vertices, %d directed ' ...
    'edges, %d checked edge records.\n'], interfaceReport.vertexCount, ...
    interfaceReport.edgeCount, interfaceReport.checkedEdgeDataCount);

plotKinodynamicPRM(roadmap);
title(sprintf('BlueROV2 Heavy kinodynamic PRM: %d directed edges', ...
    trajectoryGraph.edge_count));

fprintf('Saving LazyBTS-compatible trajectoryGraph to:\n  %s\n', graphFile);
saveTrajectoryGraph(graphFile,trajectoryGraph);
