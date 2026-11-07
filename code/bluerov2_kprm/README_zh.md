# BlueROV2 Heavy 六自由度 kinodynamic PRM（MATLAB + CasADi）

这套代码把原 double-integrator 示例替换为 BlueROV2 Heavy 的 12 状态、8 推进器非线性模型，并保留原图数据结构的基本语义。

> 重要：标准 BlueROV2 只有 6 个推进器，官方标注为 5 个可控自由度；真正全自由度驱动的是带 8 个推进器的 **BlueROV2 Heavy**。所以本实现默认 Heavy。若必须使用标准版，把推进器几何改成 6 路后，局部 OCP 将是欠驱动问题，不能保证任意两点 12 维边界状态都可连接。

## 1. 状态、控制和动力学

采用 SNAME/Fossen 的 NED 约定：

```text
x = [north east depth roll pitch yaw u v w p q r]'
f = [f1 ... f8]'  (每路推进器推力，单位 N)
```

动力学为：

```text
eta_dot = J(eta) nu
M nu_dot + C(nu)nu + D(nu)nu + g(eta) = B_T f
```

`bluerov2Dynamics.m` 同时接受 MATLAB double 和 CasADi MX/SX。`bluerov2Parameters.m` 集中保存刚体、水动力、恢复力、推进器几何、OCP、采样和地图参数。换载荷或浮力材料时，应重新辨识/填写这个文件，而不是改动力学函数。

默认水动力参数使用文献中常见的一组 BlueROV2 Heavy 低速对角参数：附加质量 `[5.50 12.70 14.60 0.12 0.12 0.12]`，线性阻尼 `[4.03 6.22 5.18 0.07 0.07 0.07]`，二次阻尼 `[18.18 21.66 36.99 1.55 1.55 1.55]`。这是一组建模起点，不应被理解为所有 BlueROV2 的厂家标定真值。

## 2. `meanControl` 局部最优连接器

兼容原调用：

```matlab
[xbar, Mcost, N] = meanControl(x0, xf, param);
```

完整调用：

```matlab
[xbar, Mcost, N, ubar, info] = meanControl(x0, xf, param);
```

它用 CasADi `Opti` + IPOPT 做 RK4 直接多重打靶。最终时间 `T` 是优化变量，目标为：

```text
J = wTime*T + integral(f' R f) + wDeltaU*sum(||f(k+1)-f(k)||^2)
```

12 维首末状态（角度按最邻近的 `2*pi` 等价表示）严格约束；同时约束推进器推力、线速度、角速度和 pitch。失败时返回空轨迹、`Inf` 代价以及 `info.success=false`，建图器据此丢弃该有向边。

首末状态在 NLP 中使用 CasADi parameter。第一次调用会构建并缓存完整
符号图；当 `param.model` 和 `param.ocp` 不变时，后续边只更新端点及
数值初值。`info.solverReused/setupSeconds/solveSeconds/totalSeconds`
可用于性能诊断。若需要清空缓存，可执行 `clear meanControl`。质量
矩阵、其逆矩阵及推进器分配伪逆也只预计算一次；旧参数会自动补齐。

## 3. 建图和查询

把本目录及 CasADi 加入路径后：

```matlab
addpath('此目录');
addpath('你的 casadi MATLAB 目录');

param = bluerov2Parameters();
param.prm.saveFile = fullfile(pwd, 'trajectoryGraph.mat');
roadmap = buildKinodynamicPRM(param);

start = [-4 -4 1, 0 0 0, zeros(1,6)];
goal  = [ 4  4 4, 0 0 pi/2, zeros(1,6)];

% 第一步：添加查询端点和局部连接，然后在搜索前检查扩展路网。
[queryRoadmap, queryInfo] = attachQueryEndpoints( ...
    roadmap, start, goal, param.world);
plotKinodynamicPRM(queryRoadmap, [], queryInfo);

% 第二步：按当前障碍物过滤边、搜索并拼接最优轨迹。
result = searchKinodynamicPRM(queryRoadmap, queryInfo, param.world);
plotKinodynamicPRM(result.roadmap, result.path, queryInfo);
```

若目标是继续运行 BlueROV2 LazyBTS，推荐直接执行
`graph_heuristic_bluerov2.m`。该脚本会在本代码目录生成
`trajectoryGraph.mat`，其中只保存一份规范的 `trajectoryGraph`
结构体。保存前会调用 `validateTrajectoryGraphInterface`，逐边核对
12 维状态、8 维控制、时间、代价和父子索引，随后可直接交给 LazyBTS。

先运行快速数值检查（不要求 CasADi）：

```matlab
validateBlueROV2Model();
```

然后可直接运行 `run_demo.m`。状态采样对每个位置选择 `connectionRadius` 内的全部其他位置，并分别生成一个朝向该位置的非零速度状态；不按距离排序、不固定选择前若干个状态，也不额外生成零速度状态。

离线建图同样检查半径内的全部节点，并在 NED 坐标系中进行三维速度方向筛选。起点速度必须基本指向目标位置，终点速度不能朝来向倒退；通过筛选后才调用 `meanControl`。每个有向边独立求解，反向边不会被强制添加。离线位置采样和局部边验收都不读取障碍物，因此生成的图只描述 BlueROV2 动力学可达性，可在不同环境中复用。

障碍物只在在线查询时使用。第一步由 `attachQueryEndpoints` 检查起终点并生成端点局部连接；此时可以先绘制扩展后的 roadmap，起点、终点和新增边分别使用独立颜色。第二步由 `searchKinodynamicPRM` 检查所有预计算轨迹、构造临时无碰撞邻接表并运行 Dijkstra；原始离线图不会被删除或改变。当前地图可以在两个步骤中显式传入：

```matlab
currentWorld = param.world;
currentWorld.obstacles = myObstacles;
[queryRoadmap, queryInfo] = attachQueryEndpoints( ...
    roadmap, start, goal, currentWorld);
result = searchKinodynamicPRM(queryRoadmap, queryInfo, currentWorld);
```

原来的单函数接口仍然可用，并在内部依次调用上述两个步骤：

```matlab
result = queryKinodynamicPRM(roadmap, start, goal, currentWorld);
```

由于完整 PRM 可能求解很多次非线性规划，应先用 demo 的 30 个位置调通，再逐步增大 `numPositions` 或 `connectionRadius`。

图变量为：

- `Vertices`: `n x 12` 状态；
- `Edges{i}`: 从 `i` 出发的子节点；
- `Parents{j}`: 指向 `j` 的父节点；
- `EdgesCost{i}`: 与 `Edges{i}` 一一对应的代价；
- `EdgeData{i}{k} = {xbar, N, ubar, info}`，与
  `Edges{i}(k)`、`EdgesCost{i}(k)` 严格同序。

该布局只为真实边分配 cell，空间复杂度为 `O(V+E)`。加载器仍可读取
旧的 `Edges_data{i,j}` 并自动转换。已有文件可一次性迁移：

```matlab
migrateTrajectoryGraphFile('trajectoryGraph.mat');
```

在线查询只求解 `start -> graph`、`graph -> goal` 及可选直接边，不再
求解不会被单向查询使用的 `graph -> start` 和 `goal -> graph`。

## 4. 障碍物和碰撞模型

示例支持球体与轴对齐盒体。ROV 用半径 `world.robotRadius` 的包围球保守近似，轨迹按 `world.collisionStep` 加密后检查。若需要精确机体碰撞，应把 `isPositionCollisionFree.m` 替换为姿态相关的 OBB/网格检测；其余代码无需修改。

## 5. 文件说明

- `bluerov2Parameters.m`：模型、推进器和规划参数；
- `bluerov2Dynamics.m`：完整六自由度动力学；
- `meanControl.m`：CasADi 时间/控制最优局部连接；
- `sampleBlueROV2States.m`：三维位置、姿态和速度采样；
- `buildKinodynamicPRM.m`：有向 PRM 构建和保存；
- `graph_heuristic_bluerov2.m`：与原 `graph_heuristic.m` 对应的脚本入口；
- `validateTrajectoryGraphInterface.m`：保存前检查 LazyBTS 数据接口；
- `attachQueryEndpoints.m`：第一步，添加查询端点及其在线局部边；
- `searchKinodynamicPRM.m`：第二步，过滤碰撞边、Dijkstra、拼接轨迹；
- `queryKinodynamicPRM.m`：保留原单次调用方式的兼容包装器；
- `dijkstraDirected.m`：二叉最小堆实现的非负权有向图最短路；
- `normalizeKinodynamicRoadmap.m`：新旧边数据布局规范化；
- `loadKinodynamicRoadmap.m`：兼容加载 `trajectoryGraph/roadmap`；
- `saveTrajectoryGraph.m`：校验并保存单份规范图；
- `migrateTrajectoryGraphFile.m`：旧 MAT 文件就地迁移；
- `isPositionCollisionFree.m`、`isTrajectoryCollisionFree.m`：碰撞检查；
- `collisionFilteredAdjacency.m`：在线过滤碰撞边并生成临时邻接表；
- `plotKinodynamicPRM.m`：三维可视化；
- `validateBlueROV2Model.m`：质量矩阵、分配矩阵和悬停检查；
- `run_demo.m`：最小使用示例。

## 6. 参数来源与适用范围

- Blue Robotics 官方技术页：标准版 5-DOF，Heavy 8 推进器/6-DOF，以及尺寸、速度与推力信息：<https://bluerobotics.com/learn/technical-reference/>
- Blue Robotics 官方产品页：当前 BlueROV2 规格：<https://bluerobotics.com/store/rov/bluerov2/>
- BlueROV2 Heavy 开源基准模型与低速 6-DOF 方程：<https://doi.org/10.3390/jmse10121898>
- 推进器位置和更新的水动力辨识汇总（Patrick Ng, 2024）：<https://bluerobotics.com/wp-content/uploads/2025/01/ng_patrick_ms_thesis_2024.pdf>

不同文献参数差异明显，尤其是惯量、heave 阻尼、净浮力和推力上限。用于实机前至少需要用实际载荷配置检查 `m/Ig/W/B/rg/rb`、8 路位置与正方向、T200 在实际电压下的正反向推力，以及阻尼系数。
