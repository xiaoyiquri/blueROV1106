%% BlueROV2 Lazy Belief Tree Search 示例
% 运行前：
%   1) 把 graphFile 改为已经生成的 trajectoryGraph.mat；
%   2) 确认 sibling 文件夹 ../bluerov2_kprm 存在；
%   3) 将 CasADi MATLAB 接口加入 MATLAB path。
clear; clc;

addpath('C:\Users\dlzhe\Documents\MATLAB\casadi-3.7.2-windows64-matlab2018b')

thisFolder=fileparts(mfilename('fullpath'));
addpath(thisFolder);
% 默认直接读取 sibling KPRM 示例生成的文件；若图位于其他位置，只需
% 修改这一行，不需要复制 MAT 文件到 LazyBTS 目录。
graphFile=fullfile(thisFolder,'..','bluerov2_kprm','trajectoryGraph.mat');

% 状态顺序：[N,E,D,roll,pitch,yaw,u,v,w,p,q,r]，角度单位 rad。
xStart = [ -2,  -1, 3.5, 0, 0, pi/2, zeros(1, 6)];
xGoal  = [3, 1, 1.5, 0, 0, 0, zeros(1, 6)];

options=defaultLBTSOptions();
options.search.verbose=true;
options.chanceConstraint=0.20;
options.chance.numSamples=100;
% 调试机会约束时取消下一行注释。animate 每次只显示当前时刻；改为
% accumulate 可把该轨迹所有时刻的协方差和样本叠加在同一张图中。
options.chance.visualize=false;
options.chance.plotMode='animate';

options.attachQuery = 0;

% 示例：声学基站覆盖的高质量区域。位置进入该盒体后，全部状态测量的
% 标准差减半。实际项目应根据定位系统标定结果设置各分量，而非照抄。
goodStd=0.5*options.sensor.defaultStd;
options.sensor.regions=struct('min',[-1.5;-1.5;1], ...
    'max',[1.5;1.5;4],'std',goodStd,'R',[]);


if options.attachQuery
    result=LBTS(graphFile,xStart,xGoal,options);
else
    result=LBTS(graphFile,97,117, options);
end
if result.success
    fprintf('Solution cost: %.6g, graph edges in path: %d\n', ...
        result.cost,size(result.edgePath,1));
    plotLBTSResult(result);

    %% 使用规划阶段同一套逐边 LQG 控制器做非线性 Monte Carlo 验证
    % MC_path 直接复用 result 中缓存的 A/B/K/L，不会为每个随机样本
    % 重新求 meanControl、线性化或 Riccati 方程。
    mcOptions=struct('numRuns',40,'rngSeed',0,'plot',true,'verbose',true);
    simulation=MC_path(result,mcOptions);
else
    fprintf('No solution. Termination reason: %s\n',result.statistics.termination);
end
