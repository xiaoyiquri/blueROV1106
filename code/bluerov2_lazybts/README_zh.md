# BlueROV2 Lazy Belief Tree Search

本文件夹在已有 BlueROV2 kinodynamic PRM（下文统一称 `trajectoryGraph`）上执行 Lazy Belief Tree Search（LBTS）。代码是独立工程，不复制或修改 `bluerov2_kprm`；它只通过 MATLAB path 读取后者的 `bluerov2Dynamics.m`、`bluerov2Parameters.m` 和 `meanControl.m`。

配套 KPRM 工程中的 `graph_heuristic_bluerov2.m` 会直接生成本工程可读的
`trajectoryGraph.mat`；保存前的 `validateTrajectoryGraphInterface.m` 会
核对每条边的 12 维状态、8 维控制、时间、代价及父子索引。

## 1. 快速开始

```matlab
addpath('C:/path/to/bluerov2_lazybts');

x0 = [-4;-4;1.2; 0;0;0; 0;0;0; 0;0;0];
xg = [ 4; 4;3.5; 0;0;pi/2; 0;0;0; 0;0;0];

opt = defaultLBTSOptions();
opt.kprmPath = 'C:/path/to/bluerov2_kprm';
result = LBTS('C:/data/trajectoryGraph.mat', x0, xg, opt);

if result.success
    plotLBTSResult(result);
    mc = MC_path(result);  % 非线性闭环 Monte Carlo 轨迹
end
```

如果起点、终点已经是图顶点，可跳过昂贵的 `meanControl` 查询连接：

```matlab
opt = defaultLBTSOptions();
opt.attachQuery = false;
result = LBTS('trajectoryGraph.mat', startVertexIndex, goalVertexIndex, opt);
```

## 2. trajectoryGraph 接口

`loadTrajectoryGraph` 只读取 MAT 文件中的 `trajectoryGraph` 结构体，不再兼容 `roadmap`、顶层分散变量或旧版稠密边表。`trajectoryGraph` 必须包含 `Vertices`、`Edges`、`Parents`、`EdgesCost`、`EdgeData` 和 `param`。

每条边必须保存：

```matlab
EdgeData{i}{k} = {xbar, N, ubar, info}; % 对应 Edges{i}(k)
```

其中 `xbar` 为 `12 x (N+1)` 状态，`ubar` 为 `8 x N` 推进器推力，`info.time` 为长度 `N+1` 的时间序列。`EdgeData{i}` 只保存与 `Edges{i}` 同序的真实出边数据。

## 3. 搜索方法

### 3.1 名义图和启发式

图的每条有向边来自确定性最优控制问题。`backwardDijkstra` 从终点沿反向边计算名义 cost-to-go：

\[
h(v)=\min_{(v,w)\in E}\{c_{vw}+h(w)\},\qquad h(v_g)=0.
\]

协方差代价非负，机会约束只会删除边，因此忽略它们的 `h` 是 belief-space 代价的下界。候选边用 `g+c+h` 排序。

### 3.2 惰性 belief 扩展

队列中的边尚未传播不确定度。只有候选边成为当前最小下界时，算法才依次执行：

1. 名义轨迹的连续碰撞检测，并缓存结果；
2. 沿名义轨迹离散线性化完整 6-DOF BlueROV2 动力学；
3. 时变 LQR、Kalman 协方差传播；
4. 高斯位置抽样的机会约束检查；
5. 同一图顶点上的 belief 支配判断。

这避免对大量永远不会成为候选最优路径的图边执行昂贵的信念传播。

### 3.3 分离式协方差传播

离散线性化为

\[
\delta x_{k+1}=A_k\delta x_k+B_k\delta u_k+w_k,
\qquad \delta u_k=-K_k\delta\hat x_k.
\]

`A_k,B_k` 默认由 CasADi 对 RK4 映射自动微分。令总协方差为 `P`，滤波估计误差为 `Ptilde`，`Phat=P-Ptilde`，则

\[
P_{k+1}=A_k\tilde P_kA_k^\top+
(A_k-B_kK_k)\hat P_k(A_k-B_kK_k)^\top+Q_k.
\]

预测估计误差为

\[
\tilde P_{k+1}^{-}=A_k\tilde P_kA_k^\top+Q_k,
\]

然后针对状态相关的 `C,R(x)` 做 Kalman 更新。实现采用 Joseph 形式以保持数值半正定。传感器区域在 `options.sensor.regions` 中定义，可表达水声基站、视觉标志或低能见度区域。

### 3.4 机会约束

对每个离散时刻，从位置边缘分布

\[
p_k\sim\mathcal N(\bar p_k,P_k(1\!:\!3,1\!:\!3))
\]

使用固定的白化高斯样本，估计包围球与边界/障碍物碰撞的比例。只有每个检查时刻的碰撞比例均不超过 `options.chanceConstraint` 时才接受该 belief 边。固定样本（common random numbers）使不同分支间的风险比较可复现。

边界、球体和轴对齐盒体的样本碰撞检测均按样本列向量化；正常搜索时
不再计算只供椭球绘图使用的 Mahalanobis 覆盖率。某一步风险已经超过
机会约束时，会提前终止该候选边后续时刻的检测；开启绘图或请求第三
输出时仍完整计算所有时刻。

注意：这里约束的是离散时刻的最大边缘碰撞概率，不是整条轨迹“至少碰撞一次”的联合概率。提高 `meanControl` 的离散节点数和 `chance.numSamples` 会增加精度，也增加计算量。

机会约束可视化默认关闭。调试单条候选边时可启用：

```matlab
opt.chance.visualize = true;
opt.chance.plotMode = 'animate';     % 逐时刻刷新
opt.chance.pauseSeconds = 0.10;
opt.chance.covarianceSigma = 3.0;    % 绘制 3-sigma 位置协方差椭球
```

`animate` 模式保留 world 和完整名义轨迹，每个时刻刷新协方差及样本；
`accumulate` 模式将所有时刻的椭球和样本叠加在同一张图中。蓝点表示
自由样本，红点表示碰撞样本，灰色为物理障碍物，红色透明表面表示考虑
`robotRadius` 后机器人中心不能进入的膨胀区域。工作空间外框和收缩后的
安全中心边界也会同时绘制。

该功能显示的是完整 12 维状态协方差的 `1:3,1:3` 位置子块，因为只有
N/E/D 三个位置分量能直接画在三维工作空间坐标中。函数新增的可选第三
输出 `sampleHistory` 还会保存每个时刻的样本坐标与碰撞布尔标记。

这里的 `sigma` 是三维 Mahalanobis 半径，不是三个坐标分别落在一维
`±sigma` 区间内。三维 `2-sigma` 椭球的理论覆盖率约为 73.85%，因此
约 26.15% 样本落在椭球外是正常现象；三维 `3-sigma` 椭球覆盖率约为
97.07%。若希望三维联合覆盖率达到一维“3-sigma”常说的 99.73%，应
设置 `opt.chance.covarianceSigma=3.7625`。绘图标题会同时显示样本实际
覆盖率和由 chi-square 分布计算的理论覆盖率。

### 3.5 支配剪枝与终止

在同一图顶点，若已有 belief 同时满足更低代价、更小总协方差和更小估计误差协方差，则新 belief 被支配并丢弃。矩阵大小采用 Loewner 半正定序判断。

参考 `LazyBTS` 脚本在首次到达目标后停止。本实现默认保存当前最优目标（incumbent），继续处理所有 `g+c+h` 仍可能改善它的分支；当队列最小下界不小于 incumbent 时才结束。若需要复现实验中的首次解行为，可设置：

```matlab
opt.search.stopAtFirstSolution = true;
```

### 3.6 最终路径 Monte Carlo 仿真

`MC_path.m` 使用最终规划结果中的名义状态、推进器推力以及规划阶段
已经计算的逐边 TVLQR/Kalman 增益：

```matlab
mcOpt.numRuns = 100;
mcOpt.rngSeed = 0;
mcOpt.plot = true;
[simulation, controller] = MC_path(result, mcOpt);
```

每条边仍独立以 `Qf` 做 Riccati 反向递推，然后沿最终路径拼接，因此
`controller.K` 与 LBTS 机会约束传播时使用的控制器完全一致。`result`
同时导出：

- `feedbackGains / kalmanGains`：每个控制区间的 (K_k,L_k)；
- `linearizationA / linearizationB`：每个区间的离散 Jacobian；
- `measurementMatrices / measurementCovariances`：(C_k,R_k)；
- `edgeStepCounts`：逐边控制区间数，用于保留边界。

Monte Carlo 循环不会调用 `meanControl`，也不会重新计算线性化、
Riccati 或 Kalman 增益。进入循环前只计算一次 (Q_k) 和 (R_k) 的
平方根，并一次生成全部随机数。所有样本在完整非线性 BlueROV2 RK4
模型上运行：

[
u_k=\bar u_k-K_k(\hat x_k-\bar x_k),\qquad
x_{k+1}=F_{RK4}(x_k,u_k)+w_k.
]

估计器使用同一控制输入做非线性预测，再用规划阶段的时变 (L_k)
更新。图中蓝线是名义路径，灰线是无碰撞样本轨迹，红线是碰撞轨迹，
青色椭球是规划协方差；同时返回碰撞率、位置 RMSE、经验协方差和推力饱和比例。`MC_path` 直接使用当前 `LBTS` 输出的 A/B/K/L 序列；若这些字段不完整，会提示重新运行 `LBTS`。

## 4. 主要文件

- `LBTS.m`：主搜索、惰性队列、incumbent 和统计信息；
- `attachQueryStates.m`：在内存中连接起终状态；
- `propagateBelief.m`：时变 LQR、Kalman 和协方差代价；
- `makeRK4Linearizer.m`：CasADi RK4 自动微分；
- `chanceCollisionProbability.m`：确定性高斯样本机会约束；
- `isBeliefDominated.m`：belief 偏序剪枝；
- `assembleLBTSResult.m`：父指针回溯和整段轨迹拼接；
- `plotLBTSResult.m`：三维路径和位置协方差椭球；
- `MC_path.m`：复用规划增益的非线性闭环 Monte Carlo 仿真；
- `executeLBTSPlanHardware.m`：不添加人工噪声的实机闭环 plan 执行、安全中止和日志；
- `HARDWARE_EXPERIMENT_zh.md`：实物实验接口、准备、分阶段流程和指标；
- `selftest_lbts.m`：不调用 IPOPT 的轻量测试；
- `run_demo.m`：完整调用示例。

## 5. 输出与调参

`result.statistics` 给出名义碰撞、机会约束、支配和数值失败的拒绝数量。没有路径时，应先看：

- `nominal_graph_disconnected`：起终点没有通过名义有向图连通；增大 query 半径/近邻数或扩充 PRM；
- `chanceRejected` 很多：检查噪声标定、传感区域和风险阈值；
- `vertexCapacityRejected` 很多：增大 `maxBeliefsPerVertex`；
- `max_expansions`：增大扩展上限，或减小图规模/每顶点 belief 容量。

`elapsedSeconds` 只统计主搜索循环，`totalElapsedSeconds` 从图加载
开始统计整个 LBTS 调用（包含验证、启发式和查询端点连接）。还可设置
`search.maxPoppedEdges` 与 `search.maxWallTime`，防止大量碰撞候选
不计入 `maxExpansions` 时运行失控。

为避免返回值再次持有完整图和全部 belief，默认：

```matlab
opt.output.includeBeliefs = false;
opt.output.includeTrajectoryGraph = false;
```

最终路径绘图使用 `result.world`，不依赖完整图。调试搜索树时可显式
打开上述开关。反向 Dijkstra 已使用二叉最小堆；LBTS 的主候选队列和
belief 支配策略按本次要求保持原实现不变。

默认 `cost.covarianceWeight=0` 与给定 LazyBTS 代码一致。若希望偏好低不确定度路径，可设为正数；这时反向 Dijkstra 仍因忽略非负协方差项而保持下界性质。

## 6. 依赖与模型边界

- MATLAB；
- CasADi（推荐并与 KPRM 局部优化器一致）；
- `bluerov2_kprm` 文件夹；
- 已生成且含控制和时间信息的 `trajectoryGraph.mat`。

若 CasADi 不可用，默认允许中心有限差分线性化，但查询起终点的 `meanControl` 仍需要 CasADi。可以在 `attachQuery=false` 且起终点已在图内时使用有限差分路径。

该实现采用一阶高斯/EKF 近似和球形机器人几何。强非线性、大姿态不确定度、多峰定位误差或非高斯海流需要更高阶滤波、采样传播或更完整的几何模型；不能仅靠增加蒙特卡洛样本数消除这些模型误差。

## 7. 理论来源

整体结构遵循 IBBT 的思想：先构建确定性名义轨迹图，计算名义图上的 informed cost-to-go，再以该启发式有序生长 belief tree。论文为 D. Zheng and P. Tsiotras, “[IBBT: Informed Batch Belief Trees for Motion Planning Under Uncertainty](https://arxiv.org/abs/2304.10984),” ICRA 2024 / arXiv:2304.10984。
