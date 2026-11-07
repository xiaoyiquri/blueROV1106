function options = defaultLBTSOptions()
%DEFAULTLBTSOPTIONS BlueROV2 Lazy Belief Tree Search 的集中参数表。
%
% 所有协方差均按 BlueROV2 的 12 维状态顺序定义：
%   x = [N,E,D,roll,pitch,yaw,u,v,w,p,q,r]'.
% 位置采用 m，姿态采用 rad，线速度采用 m/s，角速度采用 rad/s。
%
% 本函数只提供可运行的基准值。实际传感器、推进器扰动和定位系统经过
% 标定后，应优先替换 initialCovariance、processNoiseQc 和 sensor 中的值。

%% 图与局部连接器
thisFolder = fileparts(mfilename('fullpath'));
options.kprmPath = fullfile(thisFolder, '..', 'bluerov2_kprm');
options.attachQuery = true;              % 在内存中把给定起终状态连接到图
options.query.connectionRadius = 2.5;    % 起终点候选的三维位置半径 [m]
options.query.kNeighbors = 14;           % 起点出边/终点入边各尝试的最大候选数
options.query.tryDirectConnection = true;
options.query.verbose = true;

%% 初始高斯信念 N(x0,P0) 及其估计误差分解
sigma0 = [0.08 0.08 0.08, deg2rad([2 2 3]), ...
          0.03 0.03 0.03, deg2rad([1 1 1])]';
options.initialCovariance = diag(sigma0.^2);
% Ptilde 是 Kalman 滤波器的状态估计误差协方差；
% Phat=P-Ptilde 是估计均值在不同观测历史之间的离散程度。
options.initialEstimationCovariance = 0.60 * options.initialCovariance;

%% 连续时间过程噪声谱密度 Qc；离散时使用 Qk=Qc*dt
processStd = [0.002 0.002 0.002, deg2rad([0.05 0.05 0.08]), ...
              0.035 0.035 0.035, deg2rad([0.8 0.8 1.0])]';
options.processNoiseQc = diag(processStd.^2);

%% 状态相关观测模型 z=Cx+v
% 默认假设定位/姿态/速度估计器给出全部 12 个状态。若只测量部分状态，
% 将 measurementIndices 改为相应下标；R 的维度会自动随之改变。
options.sensor.measurementIndices = 1:12;
sensorStd = [0.06 0.06 0.08, deg2rad([1.2 1.2 2.0]), ...
             0.025 0.025 0.035, deg2rad([0.7 0.7 1.0])]';
options.sensor.defaultStd = sensorStd;
% 可选的高/低质量传感区域。每个元素需要 min、max (3x1)，并可给出
% std（12 维或 measurementIndices 长度）或 R。位置落入区域后覆盖默认 R。
% 示例见 run_demo.m。
options.sensor.regions = struct('min', {}, 'max', {}, 'std', {}, 'R', {});

%% 沿名义轨迹的时变 LQR 反馈 u=ubar-K(xhat-xbar)
options.lqr.Q = diag([8 8 8, 4 4 4, 2 2 2, 1 1 1]);
options.lqr.R = 0.05 * eye(8);
options.lqr.Qf = 8 * options.lqr.Q;

%% 机会约束的确定性抽样近似
options.chanceConstraint = 0.10;         % 每个离散时刻允许的最大碰撞概率
options.chance.numSamples = 200;
options.chance.rngSeed = 11;
options.chance.includeNominalPoint = true;
% 可视化是调试功能。开启后，每次 belief 边传播都会显示该边逐时刻的
% 位置协方差和高斯碰撞样本，因此大型搜索中建议仅临时使用。
options.chance.visualize = false;
options.chance.plotMode = 'animate';     % 'animate' 或 'accumulate'
options.chance.pauseSeconds = 0.05;
options.chance.covarianceSigma = 3.0;
options.chance.sampleMarkerSize = 16;
options.chance.reuseFigure = true;
options.chance.figureHandle = [];
options.chance.showInflatedObstacles = true;
% 样本仅作用于位置边缘分布 N(mu_pos,P_pos)。机器人仍由 world 中的
% robotRadius 包围球表示；因此概率检查不需要对 12 维状态全部采样。

%% 代价、支配关系与终止条件
% 名义边代价来自 meanControl。协方差项非负，因此只用名义代价计算的
% 反向 Dijkstra 值仍是可采纳的 cost-to-go 下界。
options.cost.covarianceWeight = 0.0;     % 与原 LazyBTS 默认行为一致
options.cost.covarianceMatrix = diag([1 1 1, .2 .2 .2, .1 .1 .1, .05 .05 .05]);
options.dominance.costTolerance = 1e-10;
options.dominance.psdTolerance = 1e-9;
options.dominance.indices = 1:12;        % 可只比较安全相关的状态子空间
options.search.stopAtFirstSolution = false;
options.search.maxExpansions = 20000;
options.search.maxPoppedEdges = 100000;  % 含碰撞/过期候选的总弹出上限
options.search.maxWallTime = inf;        % 主搜索秒数；Inf 表示不设时间上限
options.search.maxBeliefsPerVertex = 200;
options.search.boundTolerance = 1e-10;
options.search.verbose = true;

%% 输出体积
% 默认结果只保留最终路径及统计，不复制完整图和全部搜索 belief。调试搜索
% 树时可把相应开关设为 true；plotLBTSResult 可直接使用 result.world。
options.output.includeBeliefs = false;
options.output.includeTrajectoryGraph = false;

%% 离散化和数值稳定设置
options.linearization.preferCasADi = true;
options.linearization.allowFiniteDifference = true;
options.linearization.finiteDifferenceStep = 1e-6;
options.numerics.minEigenvalue = 0;
options.numerics.psdTolerance = 1e-9;
end
