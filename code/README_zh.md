本次新增独立轨迹推力执行入口、官方T200曲线和ArduSub3.5.4补丁，详见 [README_force_zh.md](README_force_zh.md)。当前硬件输出未完成核验；算法对齐发现一处机会约束采样差异，报告与操作限制以新说明为准。

# BlueROV2 Python：标准六推进器 KPRM + LazyBTS

本目录现在包含一个可运行的 Python 移植。默认配置是原厂标准 BlueROV2：四个 45° 水平矢量推进器和两个垂直推进器，控制向量始终为

```text
f = [f1, f2, f3, f4, f5, f6]  # N
x = [N,E,D,roll,pitch,yaw,u,v,w,p,q,r]  # 12 维
```

世界坐标是 NED，机体坐标是 `x` 向前、`y` 向右、`z` 向下。状态、控制、轨迹数组的形状在 Python 接口中明确规定为：状态 `(12,)`，控制 `(6,)`，状态轨迹 `(12,N+1)`，控制轨迹 `(6,N)`，时间 `(N+1,)`，离散 `A=(12,12)`、`B=(12,6)`、反馈增益 `K=(6,12)`。

## 安装和运行

在本目录执行：

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python -m pip install -e .
python -m pytest -q
python -m bluerov2.demo --output demo_result.pkl
python -m bluerov2.demo --output demo_result.pkl --mc
python -m bluerov2.demo --output demo_result.pkl --mc --plot
```

演示使用无障碍小图和较小的打靶区间，单独覆盖局部 CasADi 连接、图搜索、LazyBTS 协方差传播、结果保存、六路 mock 执行器；正式默认参数仍在 `bluerov2.model.parameters()` 中保持原 MATLAB 数值。

`save_trajectory_graph` / `load_trajectory_graph` 保存带嵌套边记录的 Python pickle；`save_result` 同时写入 `.json` 元数据，包含模型版本、推进器顺序、坐标系、单位和关键维度。`.mat` 旧图不会被执行器静默读取，加载时会明确拒绝；八推进器结果也会被拒绝。

## 标准六推进器适配

官方资料确认标准版采用 1–6 号电机，1–4 号为水平 45° 矢量推进器，5、6 号为垂直推进器；Heavy Retrofit 才增加 7、8 号垂直推进器。官方标准安装图中，俯视时 1 在前右（starboard）、2 在前左（port）、3 在后右、4 在后左；正视图中 5 在左、6 在右。安装说明还明确 45° 是默认水平安装，5/6 的 nose cone 朝下。来源：

- [Blue Robotics 软件设置：标准/Heavy 配置和电机方向图](https://bluerobotics.com/learn/bluerov2-software-setup/)
- [Blue Robotics R4 组装说明：电机标号、CW/CCW 和 45° 安装](https://bluerobotics.com/learn/bluerov2-assembly/)
- [ArduSub frame 配置：Vectored 为 6 路、5 个可控轴，Vectored_6DOF 为 8 路](https://ardupilot.ardupilot.org/sub/docs/sub-frames.html)
- [ArduPilot `AP_Motors6DOF.cpp`：标准 VECTORED 的 1–6 号电机方向/轴向符号](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Motors/AP_Motors6DOF.cpp#L1370-L1384)
- [Blue Robotics Technical Reference：标准版 4 horizontal + 2 vertical，5 DOF](https://docs.bluerobotics.com/downloads/)

资料中的“6 个推进器”和“5 个可控轴”是一致的；ArduSub 的 `Vectored` 名称指标准 6 路，而 `Vectored_6DOF` 指 Heavy 的 8 路。较早的 BlueROV/BlueROV1 页面会出现不同的 6 路布局名称，不能把它们的电机编号直接套到 BlueROV2 R4。

图示中的蓝色箭头是推进器排出的流体方向，模型 `direction_i` 表示作用在机体上的力方向，所以按照牛顿第三定律取相反方向；在 NED 约定下垂直推进器的正机体推力是 `+z`（向下）。模型输入顺序明确写成 `[1,2,3,4,5,6]`，不把任意绘图数组顺序冒充 ArduSub 输出顺序。

ArduPilot 源码中的 Vectored 系数只用于确认前后排的方向符号；它是无量纲混控，不能当作牛顿/牛顿米分配矩阵。Python 仍按实际 45° 单位方向和 `cross(position,direction)` 构造物理 allocation。官方网页和图纸给出了编号、朝向和配置，但没有给出相对于模型参考点的完整三维测量表。本项目因此将水平臂长沿用原 MATLAB Heavy 源文件的 `.156/.111/.085 m`，将标准 5/6 的 `y=±.218 m, x=z=0` 作为显式的未标定估计；这些数值在 `ModelParameters.geometry_status` 中标记，不能当成某一台实机的标定值。换载荷时还必须确认 `m, Ig, W, B, rg, rb`、阻尼、实际 T200 力限和推进器坐标。

当前默认分配矩阵由 `allocation[:, i] = [direction_i; cross(position_i, direction_i)]` 直接构造。对上述几何，数值秩为 **5**，奇异值约为 `[1.4196, 1.4193, 1.4142, 0.3071, 0.06364, 1.39e-17]`。不可独立产生的广义力方向主要沿 pitch 力矩的组合（数值左零空间约 `[0.0847, 0, 0, 0, -0.9964, 0]`，会随实测坐标改变）。因此保留 12 维状态和完整 OCP 约束；不可行边由 CasADi 返回失败，不通过放宽终端约束或降维掩盖欠驱动。

## MATLAB → Python 对应表

| MATLAB | Python |
|---|---|
| `bluerov2Parameters.m` | `bluerov2/model.py:parameters`, `ModelParameters` |
| `bluerov2Dynamics.m` | `bluerov2/dynamics.py:bluerov2_dynamics` |
| `meanControl.m` | `bluerov2/kprm.py:mean_control` |
| `sampleBlueROV2States.m` | `sample_blue_rov2_states` |
| `buildKinodynamicPRM.m` | `build_kinodynamic_prm` |
| `getKinodynamicEdge.m` | `get_kinodynamic_edge` |
| `weightedStateDistance.m` | `weighted_state_distance` |
| `collisionFilteredAdjacency.m` | `collision_filtered_adjacency` |
| `attachQueryEndpoints.m` / `queryKinodynamicPRM.m` | `attach_query_endpoints` / `query_kinodynamic_prm` |
| `dijkstraDirected.m` | `dijkstra_directed` |
| `saveTrajectoryGraph.m` / `loadKinodynamicRoadmap.m` | `save_trajectory_graph` / `load_trajectory_graph` |
| `rk4StepLBTS.m` | `dynamics.rk4_step` |
| `makeRK4Linearizer.m` / `evaluateRK4Linearization.m` | `make_rk4_linearizer` / `evaluate_rk4_linearization` |
| `measurementModelAtState.m` | `measurement_model_at_state` |
| `propagateBelief.m` | `propagate_belief` |
| `chanceCollisionProbability.m` | `chance_collision_probability` |
| `isBeliefDominated.m` / `backwardDijkstra.m` | `belief_dominated` / `backward_dijkstra` |
| `LBTS.m` / `assembleLBTSResult.m` | `lbts` and its result assembly |
| `MC_path.m` | `monte_carlo_path` |
| `executeLBTSPlanHardware.m` | `execute_lbts_plan` |
| `plot*.m` | `bluerov2/plotting.py` |

`Other/` 中的查询、端点连接、图加载、验证、Dijkstra 和碰撞辅助逻辑均已纳入 Python；MAT 文件缓存本身不作为可执行输入。第三方 CasADi 库和无用缓存没有翻译。

## 修改清单和边界

纯语言/数据结构转换包括 MATLAB 1-based 索引到 Python 0-based、cell/struct 到 dataclass/dict、列优先 CasADi `vec`、反斜杠到 `solve`、PSD 投影、稀疏邻接表、图和结果 pickle 保存。

六推进器必需修改包括标准 `[1..6]` 几何、`(6,6)` allocation、6 维推力上下限和 `R`、局部 OCP 控制变量、动力学输入雅可比 `(12,6)`、Riccati/LQR `K=(6,12)`、Monte Carlo、协方差和执行器回调；停止输出是六维零推力。

没有擅自改变控制器、目标、RK4、机会约束、信念协方差递推或搜索终止规则。仍待确认的物理量是标准机体的实测推进器坐标、实际方向符号、质量/浮力/惯量/阻尼和电压相关 T200 推力曲线；在确认前程序只代表“算法移植 + 显式几何估计”，不声称是实机标定模型。

原 MATLAB 中需要特别留意的项目问题是：`bluerov2Parameters.m` 的说明和几何是 Heavy 八推进器，而 `bluerov2_lazybts/executeLBTSPlanHardware.m`、`MC_path.m`、`validateLBTSSetup.m` 以及 OCP/LQR 代价都直接固定了 8 维。这不是可安全删除两列的语言差异，已在 Python 入口统一检查并改为 6 维。标准版本身只有 5 个可控广义力方向，所以任意 pitch 终端约束可能导致 `mean_control` 不可行；这是实机欠驱动能力的真实结果，本移植没有为让示例成功而放宽约束。质量、浮力、惯量、阻尼和臂长来自原 Heavy/近似模型或未测量估计，属于待确认物理参数，不是已验证的标准机标定值。

## 验证结果和限制

已实际运行 `python3 -m pytest -q` 和 `python3 -m bluerov2.demo --output /tmp/demo.pkl`。测试检查动力学输出、RK4、CasADi 离散 Jacobian、六路维度、秩/奇异值、旧 MAT 拒绝、局部连接、LBTS belief 传播、六路 mock 回调和结果图保存。演示成功得到局部边、固定小图搜索和 mock 执行；mock 状态保持起点，因此报告的是执行状态和跟踪误差，不把“执行完时间序列”称为成功跟踪。

当前环境没有运行 MATLAB，因此没有声称 MATLAB 数值逐项一致。需要 MATLAB 对照时，可在 MATLAB 中使用 `tools/export_matlab_reference.m` 导出固定输入；相同整数随机种子不保证 MATLAB/NumPy 生成相同样本。

`execute_lbts_plan` 仍接受 `initialize/read_state/send_thruster_forces/abort_requested/stop` 这组硬件无关回调；新增 `bluerov2.hardware_check` 用于 MAVLink/A50 被动遥测诊断。它不是推力到 PWM 或 MANUAL_CONTROL 的实机输出适配器。

## 真机连接与坐标核验（2026-11-03）

参考上级目录的 `rov_control.py`、`rov_host_goto.py`、`dvl_mavlink_bridge.py` 和 `host_goto_config.json`，新增只读检查入口：

    python3 -m pip install -e '.[hardware]'
    python3 -m bluerov2.hardware_check --config ../host_goto_config.json --duration 8 --output hardware_check_report.json

命令并行接收飞控 MAVLink 与 A50 TCP JSON，输出消息计数、真实飞控来源、解锁状态、状态新鲜度、DVL 速度/波束有效性与独立的位姿数据。它不发送心跳、消息请求、解锁、模式、执行器或参数命令；没有已配置好的遥测路由时，被动监听可能收不到数据。串口连接的打开超时由底层串口实现决定，采样时长不包含其打开时间。

坐标处理遵守以下约定：

- 飞控 `LOCAL_POSITION_NED` 的 `vx/vy/vz` 属于世界 NED，规划器的 `u/v/w` 属于机体 FRD，转换使用 `R_zyx(roll,pitch,yaw).T @ velocity_ned`；`ATTITUDE` 的机体角速度不再当作欧拉角导数变换。
- A50 航位推算姿态由度转换为弧度。上级配置的 `local_at_dvl_reset` 表示复位时建立的局部任务系，不表示地理北/东；检查器保留 A50 独立坐标，不自动与飞控原点或航向拼接。
- 缺少飞控 `LOCAL_POSITION_NED` 或 `ATTITUDE` 时，完整状态保持 `null` 并列出缺失消息；不以零位置、零速度补齐，也不把原始 DVL/IMU 拼接后称为融合估计。状态年龄只表示主机接收后的时间，不代表传感器延迟已标定。

本轮对 `udpin:0.0.0.0:14550` 和 `192.168.2.95:16171` 的实际连接尝试均被当前执行沙箱以 `PermissionError: [Errno 1] Operation not permitted` 拒绝；记录见 `hardware_check_report.json`。因此尚未收到本轮真机遥测，不能据此判断设备离线、固件版本或定位可用性。需要在具有本机网络访问权限的终端运行上述命令。

**实机推力映射仍有阻塞项：** `model.py` 中水平推进器 1/3 的继承坐标 `y<0`，但其文字标签为 starboard；这与声明的 FRD `+y` 向右存在内部冲突。当前数值仍作为未核验的估计保留，`geometry_status` 已明确标记此问题。需要实机确认推进器编号、安装位置、正推力方向以及牛顿到实际控制量的标定；不能直接把六维牛顿输出作为 MANUAL_CONTROL 或 PWM 输入。

本轮还修正了 Monte Carlo 初始化：估计状态使用 `P-Ptilde`，真实状态相对估计的误差使用 `Ptilde`，与 `MC_path.m` 一致。原演示的四段粗时间网格出现 RK4 数值发散；演示默认值改为原模型的二十段，控制器和水动力参数未改动。非有限仿真结果会显式报错，不再作为物理碰撞统计。新增回归测试覆盖这些定义及执行器的末段中止、陈旧状态、异常时间/数值、初始化异常清理和通信关闭后的重复发送。

使用二十段的端到端演示已完成局部优化、图查询、LBTS、mock 执行和绘图；固定种子的五次 Monte Carlo 样本未碰撞/越界。这只是小规模数值检查，不是实机跟踪或概率安全证明；mock 仍保持起点，最终位置误差约 0.403 m。
