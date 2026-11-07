function report = saveTrajectoryGraph(fileName, roadmap)
%SAVETRAJECTORYGRAPH 校验并仅保存一份规范 trajectoryGraph 结构体。
%
% 不再同时保存结构体和展开的顶层副本，避免轨迹数组在 MAT 文件中重复。
trajectoryGraph=normalizeKinodynamicRoadmap(roadmap);
report=validateTrajectoryGraphInterface(trajectoryGraph);
save(fileName,'trajectoryGraph','-v7.3');
end
