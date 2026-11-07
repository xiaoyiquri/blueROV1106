# BlueROV2 LazyBTS 实物实验流程

## 1. 是否直接执行 plan

`result.controlTrajectory` 是模型上的名义最优推力，不能把它当作完全开环的“录像”直接播放。即使假设规划模型与实物动力学完全一致，初始对位误差、水流、线缆拖曳、推进器死区以及传感器误差仍会让实物偏离名义轨迹。

实机应执行规划阶段使用的同一时变反馈律：

\[
f_k=\operatorname{sat}\!\left(\bar f_k-K_k(\hat x_k-\bar x_k)\right),
\qquad t\in[t_k,t_{k+1}),
\]

其中 `result.nominalTrajectory(:,k)` 是 \(\bar x_k\)，`result.controlTrajectory(:,k)` 是 \(\bar f_k\)，`result.feedbackGains{k}` 是该 PRM 边独立计算后按最终路径拼接的 \(K_k\)，\(\hat x_k\) 是真实传感器和状态估计器给出的状态估计。姿态误差必须包络到 \([-\pi,\pi]\)。

因此，“执行 plan”应理解为执行名义轨迹、前馈推力和规划时同一组反馈增益，而不是只执行前馈推力。

## 2. 实机中如何处理噪声

实机实验不调用 `MC_path`，也不应额外生成 `randn` 过程噪声或测量噪声：

- 实际水动力误差、水流、推进器波动和线缆作用就是系统真实受到的过程扰动；
- IMU、深度计、DVL、声学定位等设备自身的误差就是实际测量噪声；
- 规划中的 `processNoiseQc` 和传感器协方差 `R` 用于预测 belief、评估碰撞概率并选择路径，不是实机执行时要再次施加的信号；
- 多次独立实物试验本身就是对真实噪声分布的采样。

如果 BlueROV2 上已有 EKF 或其他状态融合器，`readState` 应读取它输出的 12 维状态估计，并直接用于 LQR 反馈。此时不要再应用 `result.kalmanGains`，否则会对同一批传感器信息重复滤波。只有当程序直接接收与规划观测模型一致的原始测量时，才应单独实现规划中的预测—Kalman 更新；当前实机执行文件没有假定这种硬件接口。

## 3. 坐标、状态和推力接口

状态顺序固定为：

```text
x = [north, east, depth, roll, pitch, yaw, u, v, w, p, q, r]'
```

- 惯性坐标采用 NED，深度向下为正；
- 位置单位为 m，姿态单位为 rad；
- `u,v,w` 是机体系线速度，单位 m/s；
- `p,q,r` 是机体系角速度，单位 rad/s；
- 控制是 8 个 T200 推进器产生的力，单位 N；
- 推进器编号、正方向必须与 `result.model.thrusterPosition` 和 `thrusterDirection` 的列顺序一致。

`io.sendThrusterForces` 必须由实验平台实现从推力 N 到 PWM、归一化指令或底层电调命令的标定映射。不能直接把 N 数值当成 PWM。建议为每个推进器使用水池标定得到的正、反向查表，并在适配器内处理死区、供电电压和最终硬限幅。

## 4. 使用代码

新文件 `executeLBTSPlanHardware.m` 是硬件无关的闭环执行器。通信细节通过 `io` 回调提供：

```matlab
load('planned_result.mat','result');
addpath('C:/path/to/bluerov2_lazybts');

io.initialize=@initializeVehicleConnection;
io.readState=@readFusedBlueROVState;
io.sendThrusterForces=@sendCalibratedThrusterForces;
io.abortRequested=@readEmergencyStop;
io.stop=@stopDisarmAndDisconnect;

% 第一次只做全流程读数和时序检查，不发送推力。
opt=struct();
opt.enableActuation=false;
opt.realTime=true;
opt.logFile='logs/dry_run_001.mat';
dryLog=executeLBTSPlanHardware(result,io,opt);
```

`readFusedBlueROVState` 推荐返回：

```matlab
sample.state=xhat;          % 12x1，顺序和单位见上一节
sample.valid=estimatorOK;   % 定位丢失、EKF 发散时为 false
sample.age=dataAgeSeconds;  % 当前时刻减去状态时间戳
sample.timestamp=deviceTime;
```

完成台架检查和无推力 dry run 后，实机输出需要两个显式开关：

```matlab
opt.enableActuation=true;
opt.confirmation='ENABLE_BLUEROV2_THRUST';
opt.logFile='logs/pool_run_001.mat';
experiment=executeLBTSPlanHardware(result,io,opt);
```

默认安全条件包括：起始位置、姿态、速度必须接近 plan 起点；状态数据不能超时；跟踪误差、线速度和角速度不能超过阈值；估计位置必须仍处于规划 world 的自由空间；控制始终限制在模型的 8 个推进器推力上下限内。任一条件失败或人工急停后，函数停止后续区间，并通过清理回调发送零推力及调用 `io.stop`。

所有阈值都在 `executeLBTSPlanHardware.m` 的 `completeHardwareOptions` 中集中定义。正式试验前应根据水池尺寸、定位更新率、制动距离和安全员要求确定，不能直接把默认值视为认证后的安全参数。

## 5. 实验前准备

### 5.1 模型和执行器标定

1. 称量整机质量，核对浮力、重心和浮心；每次更换电池、配重、机械臂或浮力材料后重新核对。
2. 在水中逐个测试 8 个推进器的编号和正方向。一次只允许低推力点动，并设置物理急停。
3. 标定正反向“命令—推力”曲线。记录电池电压，因为 T200 推力与电压有关。
4. 检查规划推力范围是否落在标定范围内；底层适配器和 MATLAB 两端都应限幅。
5. 核对 NED/ENU、深度符号、机体系轴方向和 Euler 角定义。坐标错误通常会使反馈立即向错误方向放大。

### 5.2 状态估计和时间

1. 静止记录至少 2–5 分钟传感器数据，确认偏置、方差和漂移量级与规划参数一致。
2. 检查状态估计器能持续输出 12 个状态；若某些速度由差分得到，确认滤波延迟。
3. `sample.age` 应在 MATLAB 收到数据时计算，正式输出默认要求小于 0.25 s。
4. 检查 MATLAB 计算机、飞控和外部定位设备的时间同步；日志中同时保留 MATLAB 相对时间和设备时间戳。
5. 如果实物估计器的延迟不可忽略，应在控制器中补偿或降低速度，不能把过期状态直接用于高增益反馈。

### 5.3 场地和安全

1. 将水池坐标系与 `result.world` 配准，实测墙面和障碍物位置。
2. 从 world 障碍物尺寸中额外预留定位误差、机器人外形误差、线缆和制动距离。
3. 设置独立于 MATLAB 的物理急停、飞控失联保护、深度限制和推力限制。
4. 安排一名只负责急停的安全员；测试区域内不得有人靠近推进器。
5. 首次试验移除障碍物并缩短路径。不要直接从完整障碍环境开始。

## 6. 推荐实验顺序

1. **离线检查**：确认 `result.success=true`，绘制名义轨迹、协方差、推力和反馈增益；检查所有名义推力都在实机标定范围内。
2. **软件 dry run**：`enableActuation=false`，让全部通信、状态读取、时序、安全判断和日志运行一遍。
3. **系留低推力测试**：使用很短、无障碍、低速路径，临时降低底层最大推力，验证反馈方向。
4. **单自由度测试**：依次测试深度、surge、sway 和 yaw，确认坐标、推进器分配和姿态包络正确。
5. **短三维路径**：执行离障碍物较远的短轨迹，逐步提高到规划速度。
6. **完整 plan**：只有前述阶段的跟踪误差、时延和推力饱和均合格后，才加入障碍物并执行完整路径。
7. **重复试验**：相同 plan 至少重复多次，不人为添加噪声。使用各次真实日志统计成功率、最大/均方跟踪误差、最终误差、饱和率和最小障碍距离。

每次改变模型、配重、传感器、控制频率、定位系统或推进器标定后，都应返回较低一级重新验证。

## 7. 日志和结果分析

代码保存：

- `referenceState/referenceControl`：规划名义轨迹和前馈推力；
- `estimatedState/trackingError`：真实状态估计及其相对名义轨迹的误差；
- `feedbackControl`：LQR 修正量；
- `unconstrainedCommand/commandedThrusterForce`：限幅前后推力；
- `saturated`：每个推进器、每个时刻是否限幅；
- `actualTime/scheduledTime/stateAge/deviceTimestamp`：控制时序和传感器新鲜度；
- `status/abortReason/executedSteps`：完成或安全中止原因。

建议报告以下指标：

\[
e_{p,\mathrm{RMS}}=\sqrt{\frac{1}{N}\sum_k\|p_k-\bar p_k\|^2},
\qquad
e_{p,\max}=\max_k\|p_k-\bar p_k\|,
\]

以及最终位置/姿态误差、控制饱和比例、状态最大延迟、每次试验最小障碍间距和任务成功率。若有独立光学定位或声学真值系统，应同时记录真值；控制器仍使用在线估计，真值只用于事后评估。

规划协方差与实测误差的比较应跨多次独立试验进行。例如在每个名义时刻对所有重复试验的误差计算经验协方差，再与 `result.covarianceSequence{k}` 比较。一次轨迹不能可靠验证概率模型。

## 8. 代码边界

该文件提供计划调度、LQR 反馈、安全检查和日志框架，但不会猜测具体飞控接口，也不会自动完成以下工作：

- T200 推力到 PWM/归一化命令的标定映射；
- ROS、MAVLink、串口或厂商 SDK 的连接；
- 机载 EKF、DVL/USBL/视觉定位的数据融合；
- 物理急停、飞控 failsafe 和监管认证。

这些功能必须在 `io` 适配器和飞控端按实际实验平台实现，并在无推力和低推力阶段逐项验证。
