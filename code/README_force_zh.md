当前交付在独立“轨迹推力执行”基础上完成 PWM/轨迹依赖分离、完整状态缓存和轮询优化，入口是 `python3 -m bluerov2.force_run`。规划、搜索、代价、协方差传播及反馈控制文件没有为本任务修改；推力执行继续使用 `bluerov2.execution.execute_lbts_plan` 的原反馈计算和启动/运行误差检查。原 goto、主机 PD、MANUAL_CONTROL 功能保留。

2026-11-05 本轮增加独立六路推力测试入口并核对依赖和官方曲线原始点；当前测试、环境受限项和未完成实机事项见 [本轮审查记录](reports/force_chain_review_20261105_zh.md)。

**2026-11-03 接口修订验收：170 项项目测试、26 项旧基础控制回归通过，当前主机代码与已有精确版本 ArduSub 3.5.4 SITL 的 15 项联测通过。** 本轮仅离线、历史日志回放和本机 SITL，没有连接/解锁真机、修改实机参数或刷固件。历史真机报告记录的是未部署 direct6 的 ArduSub 3.5.4，不能据本轮 SITL 宣布实机已支持独立输出。已有传感器日志按当前缓存实现回放仍在 0.100 s 后因缓存过期停止；未放宽门限。推进器标签/几何与实机对应尚未核验，不能从注释矛盾推定具体数值错误。详见 [当前修订报告](reports/interface_revision_20261103/REVIEW_zh.md)；[上一轮报告](reports/VALIDATION_zh.md) 保留为历史证据。

**算法对齐没有全部通过。** 原 `.m` 数值源码已通过隔离的 GNU Octave 6.4.0 运行，与 Python 比较。26 项中 25 项通过；同一组样本下机会约束概率存在 0.090/0.085 差异。没有擅自修复算法，详见本文算法验证说明及 `reports/proposed_chance_alignment.patch`。MATLAB 本体、原 MATLAB CasADi/IPOPT 和完整规划搜索尚未运行。

交付文件

| 文件 | 内容 |
|---|---|
| `bluerov2/state_adapter.py` | DVL 固定位姿/速度 + 飞控机体系角速度 → 完整十二维缓存；区分等待/复用/故障 |
| `bluerov2/t200.py` | 官方离线曲线，分别正反向、按电压插值 |
| `bluerov2/communication.py` | 只加载旧通信/logger 和连接字段，供纯 PWM 入口使用 |
| `bluerov2/thruster_output.py` | `send_thruster_forces` / `send_thruster_pwm`，完整六路协议、停止确认 |
| `bluerov2/force_run.py` | 计划、六路推力和纯 PWM 三个入口；计划导出元数据、实验报告/图 |
| `bluerov2/data/force_config.json` | 路径、映射、状态变换、电压、调度阈值、输出核验状态 |
| `bluerov2/data/*.xlsx`、`t200_curves.json` | 官方原始数据和离线预处理数据 |
| `tools/preprocess_t200.py` | 用标准 ZIP/XML 库完整读取 Excel，重现预处理，无 openpyxl 依赖 |
| `tools/replay_force_state.py` | 离线回放保存的传感器记录，统计更新、年龄、时间差及锁存停止；不连接设备 |
| `tools/verify_connected.py`、`verify_direct6_sitl.py` | 只读三路采集/时序核查，以及完整本机飞控进程联测 |
| `tools/diagnose_direct6.py` | 只读飞控版本/参数/六路回读/能力诊断；不依赖 DVL，不发送心跳或执行器指令，出站白名单只允许版本/参数读取及 QUERY |
| `tools/verify_matlab_alignment.py`、`alignment_reference.m` | 可重复的原 MATLAB 数值源码对齐验证 |
| `firmware/ardusub_3_5_4/` | 精确版本的源码补丁、协议说明、共享 C++ 核心和测试 |
| `tests/test_force_output.py`、`test_force_state.py`、`test_force_entry.py` | 新链路、原反馈、时序、原生固件核心、状态与停止回归测试 |
| `reports/` | 当前接口修订、历史只读诊断、算法差异及实际测试记录 |

安装和三类命令

```bash
cd /home/tj/blueROV2/code
python3 -m pip install -e '.[hardware,test]'
python3 -m bluerov2.force_run --help
```

普通命令默认**离线检查**，不会打开机器人连接、请求模式或解锁。硬件命令仅增加一次 `--execute`，不弹出反复确认，也不自动刷固件/修改参数。

```bash
# 1. 算法计划：先离线检查，再在所有核验完成后显式执行
python3 -m bluerov2.force_run --output reports/plan_check.json plan experiment.pkl
python3 -m bluerov2.force_run --output reports/experiment.json plan experiment.pkl --execute

# 2. 六路 PWM：参数按物理 MAIN1..MAIN6，最多 5 秒
python3 -m bluerov2.force_run --output reports/pwm_check.json pwm-test --pwm 1500 1500 1500 1500 1500 1500 --duration 0.5
python3 -m bluerov2.force_run --output reports/pwm_test.json pwm-test --pwm 1540 1500 1500 1500 1500 1500 --duration 0.5 --execute

# 3. 六路推力：参数按模型索引0..5，单位N；不加载定位或规划结果
python3 -m bluerov2.force_run --output reports/thrust_check.json thrust-test --forces-n 1 -1 2 -2 1.5 -1.5 --duration 0.5
python3 -m bluerov2.force_run --output reports/thrust_test.json thrust-test --forces-n 1 -1 2 -2 1.5 -1.5 --duration 0.5 --execute
```

16V官方死区约1472～1528μs，因此±20μs可能没有可辨别推力；示例1540μs位于正向死区外，反向可用1460μs逐路测试。非中立示例仅用于完成固件构建/测量和水中逐路验证后的显式测试；若连接到未打补丁实机，会因缺少 direct6 能力应答而拒绝输出。纯 PWM 不加载 DVL、原点、规划/动力学或 T200 曲线，按物理 MAIN1～6 测试。推力测试加载 T200 曲线和模型映射，不加载 DVL、原点或规划器；离线时优先用显式 `output.test_voltage_v` 检查电压能力，连接时用新鲜遥测电压（或明确配置值）重新映射。推力参数顺序是模型索引0..5，输出预览同时标出模型推力与物理 PWM。推力入口需要一次性 `channel_mapping_verified=true` 记录。三入口共用版本/参数/能力检查、输出独占、租约和停止确认。

统一安装入口为仓库根目录的 `python3 -m pip install -r requirements.txt`，等价于安装 `code[hardware,test]` 和旧通信后端依赖。也可在 `code/` 运行 `python3 -m pip install -e '.[hardware,test]'`。

计划的实际格式与导出

继续使用现有 `bluerov2.io.save_result` 的 `bluerov2-python-result-v1` pickle，包含原始 `ModelParameters` 和所有算法结果，不手工重排长轨迹。不读取旧八推进器 MAT 缓存。只加载自己生成、可信的 pickle；Python pickle 是对象格式，不能当作不可信交换格式。

| 字段 | 维度/意义 |
|---|---|
| `success` | 原算法的成功结果 |
| `nominalTrajectory` | `(12,N+1)`；`[N,E,D,roll,pitch,yaw,u,v,w,p,q,r]` |
| `controlTrajectory` | `(6,N)`；各推进器标称推力 N |
| `feedbackGains` | 长度 N；每个 `(6,12)`，原算法增益 |
| `time` | `(N+1,)`；秒，从 0 开始严格递增，可非等间隔 |
| `model`、`world` | 原算法模型和世界约束 |
| `execution_reference` | 导出函数附加单位、固定参考摘要、模型摘要；不会修改轨迹 |

在原规划调用成功获得 `result` 后，用以下函数替换保存调用：

```python
from bluerov2.force_run import save_force_plan

# 前提：生成 result 时使用的就是 task_origin.json 定义的固定任务坐标。
# 此函数只附加元数据，不会把地理 NED 或其他原点的结果自动变换过来。
save_force_plan('experiment.pkl', result, '/home/tj/blueROV2/task_origin.json')
```

现有结果可先用 `load_result` 读取后调用同一函数，但必须知道该结果的真实坐标基准。元数据不能把不一致的轨迹变成可执行轨迹；若旧计划使用其他原点/朝向，先按原算法接口在本任务坐标中生成，不能简单盖上摘要。示例演示计划通常不是实机原点的轨迹。

离线检查复用原执行器检查维度、时间、有限数值、速度及自由空间；实机准备阶段额外检查原起点位置/姿态/线速度/角速度条件、实际电压下标称推力能力、实时固件及参数、完整状态与输出能力。计划标称推力超算法或硬件范围直接拒绝，反馈超硬件能力也直接终止，不用大范围裁剪掩盖不可执行性。

状态来源与固定参考

本次不切换定位方案、不重复 EKF。没有 `LOCAL_POSITION_NED` 时，复用上级项目有效使用过的 `DvlFeed` 和 `parse_dead_reckoning`：

| 状态 | 来源 | 转换 |
|---|---|---|
| 位置 | A50 航位推算 x/y/z | `R_world @ p_dvl - saved_origin - R_world_body @ lever`，m |
| roll/pitch/yaw | A50 航位推算姿态 | 原始度 → rad；使用完整安装/世界旋转组合，再提取 ZYX |
| u/v/w | A50 vx/vy/vz | 默认 native sensor；撤销旧 feed 已做的矩阵后只应用一次传感器→机体旋转，并补偿已配置的杆臂转动速度 |
| p/q/r | 飞控 `ATTITUDE.rollspeed/pitchspeed/yawspeed` | 原版固件发送的是 `ahrs.get_gyro()`；单位 rad/s，机体系角速度，直接使用 |

若已经确认速度在 DVL 固定世界系，将 `state.velocity_frame` 设为 `dvl_world`，按同一完整姿态转入机体系；不能对 native 机体系速度重复世界旋转。原 feed 的同一矩阵曾同时用于位置和速度，本适配器明确区分世界与安装旋转，避免继承该含糊语义。

上级 `task_origin.json` 保存的是位置平移 `origin`、安装矩阵和航向目标 `yaw_target_rad`。**文件写入不执行设备 Reset，也不重置飞控 Home/EKF。** 上级说明记录 A50 Reset dead reckoning 会重设设备航位推算位置及姿态参考；这属于设备操作，当前连接已恢复，但现有参考文件没有设备 Reset 标识，无法仅凭一次静态采样核实最后复位时刻。此次代码不发送任何 reset/calibration 命令，不要求每次从水面重新建原点。

`local_at_dvl_reset` 是初始航向对齐后固定的右手世界系，不凭“复位”就成为地理北东；状态中的 N/E 在这里是沿用算法的第 1/2 轴名称。`world_from_dvl_world` 必须与保存原点的任务变换相同。机体系 FRD 的方向由独立的 `body_from_dvl_sensor` 定义，杆臂 `body_to_dvl_m` 为机体参考点指向 DVL 的机体向量。

新适配器不会把上级 PD 的 `yaw_target_rad` 再次扣成零航向：上级原点文件只平移位置，而航向目标是控制时保持的角度；对姿态单独减去它、位置仍不旋转会产生不一致。算法轨迹必须使用同一个固定 DVL 世界基准的姿态和位置。若需要以保存航向为新世界轴，必须统一变换位置、姿态和速度，不能仅调整 yaw；本次没有擅自这么做。

状态适配器返回 `state/valid/status/cached/age/timestamp/receipt_mono/component_age_s/receipt_skew_s/device_timestamp/quality/reference_id`。无有效缓存时返回 NaN、`valid=False,status=waiting`，启动最多等待 3 s，不取得输出会话。新的完整候选只有在有限性、质量、0.25 s 年龄、0.15 s 接收/设备采样时间差等检查全部通过后，才原子替换缓存。暂缺分量、质量暂未合格或分量不同步时，继续返回仍有效的完整缓存，不能拼接不合格候选的一部分。

复用缓存保留原 `timestamp`、各分量接收/设备时间、质量和参考标识，按当前时间重算年龄，**不重置时间或年龄**。一旦缓存年龄 `>=0.25 s` 即锁存故障并停止任务；本轮没有增大门限。非有限/畸形数据、心跳中断、设备时间回退/重启、坐标突变、程序异常和参考文件变化均保留原因并停止，不解释成普通缺包。故障锁存后正常数据恢复也不自动输出。

固定旋转和逆变换在初始化检查/计算；消息未更新时不重复状态解算。原点摘要启动时固定，默认每 1 s 检查一次变化（`state.reference_check_period_s` 可显式配置为正值），设备时钟、心跳、跳变和缓存年龄仍每轮监测。任务中不得进行设备 Reset；没有可见跳变且设备时间持续前进的外部复位，现有报文不保证能识别。

`age` 是主机接收年龄及已知 DVL 速度处理时间的下界，不包含尚未测定的网络及位姿延迟。速度采样/发送时间戳已保留，并检查重放、倒退、内部处理延迟及 DVL 位姿/速度采样偏差。当前报文来源没有被证明共享采样时钟；日志明确记录这一限制。实际核验时需要测量 DVL/飞控延迟及偏差，不能把“同一时刻收到”当作“同一时刻采样”。`state.frame_verified=false` 因此保持默认，不能为了启动而跳过核验。

T200 曲线和推力方向

来源：[官方产品](https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster-r2-rp/)，[官方原始 Excel](https://cad.bluerobotics.com/T200-Public-Performance-Data-10-20V-September-2019.xlsx)。本地 XLSX SHA256：`1399b76df7a176cfbb08760c11731199053336e7d92182dece11824c73285085`。

实际核对的工作表为 `READ ME FIRST` 和 `10 V`、`12 V`、`14 V`、`16 V`、`18 V`、`20 V`，各 201 个 1100～1900 μs、4 μs 间隔点。列名为 PWM (µs)、RPM、Current (A)、Voltage (V)、Power (W)、Force (Kg f)、Efficiency (g/W)，推力乘 `9.80665` 转 N。

文件“READ ME FIRST”的正文在 Excel 绘图文本中，预处理实际读取 `xl/drawings/drawing1.xml`：单个 2019 年 8 月 T200，Basic ESC R3，原始设置 BlHeli_S 16.6，静态系泊、零入口速度；ESC 电源端远端采样消除电源线压降；每个 PWM 启动 1.5 s 后采 2 s、10 Hz 共20点求均值，再休息30 s；小于300 RPM的数据置零。完整测试条件保存在 JSON `source_notes`。这是官方静态近似，**不是本机实测推力反馈**。

预处理与反查规则：

1. 正反向独立处理，保留官方不对称数据。按 PWM 离开中立的顺序对力幅值做等权 PAVA 单调回归；各表最大调整量已保存，约0～0.067 N。
2. 零力始终为1500 μs；正反向死区的零力锚点位于对应死区外沿。重复非零力选离中立最近的 PWM，使反查确定。
3. 先在相同 PWM 上按实际电压线性插值力曲线，再对相应支路逐段反查 PWM。舍入到整数 μs，并记录该量化 PWM 的官方静态估计力。
4. 电压限制10～20 V，无范围外推；推力超过该电压能力报错。输出层最多容忍0.05 N边界舍入裁剪，单独记录请求力/采用力/硬件饱和；算法原有限幅仍由原执行器计算。
5. 零推力清理不依赖电压，仍可在电压遥测失效后回中。非零力优先新鲜 `SYS_STATUS`，其次 `BATTERY_STATUS[0]` 有效电芯电压求和。没有遥测时仅允许显式 `output.test_voltage_v`，日志写 `configured_test_voltage`，默认 null。

`T200.thrust_to_pwm(force_n, voltage_v, thruster_id)` 中 `thruster_id` 为原厂推进器号。输出接口 `send_thruster_forces(forces_n)` 的六元素顺序是模型输入顺序；`send_thruster_pwm(pwm_us)` 的六元素顺序是**物理 MAIN1～MAIN6**。映射集中在 JSON 中，不散落编号。配置中的 `model_signature` 绑定该映射所对应的模型几何/推力限值；模型变化时不能继续沿用旧映射核验。

| 模型索引（从0） | 原厂编号 | 实测配置输出 | 候选 model→curve 符号 |
|---|---|---|---|
| 0 | 1 | MAIN1 / SERVO1_FUNCTION=33 | -1 |
| 1 | 2 | MAIN2 / 34 | -1 |
| 2 | 3 | MAIN3 / 35 | +1 |
| 3 | 4 | MAIN4 / 36 | +1 |
| 4 | 5 | MAIN5 / 37 | -1 |
| 5 | 6 | MAIN6 / 38 | +1 |

这些符号来自历史只读诊断中的 `MOT_*_DIRECTION`，只作为候选。原混控的方向设置不能证明“模型正力方向=实机方向”，模型中1/3横向位置符号与 starboard 标签存在矛盾，现有信息不足以确定标签、编号、坐标或符号中哪一项需修正。专用路径直接写物理 PWM，故意绕过 `MOT_DIRECTION`、混控及 servo 缩放；最终必须逐路测量实际作用力方向再更新配置并设置 `channel_mapping_verified=true`。本次没有改动模型几何来掩盖问题。若需修正模型/重新规划，先确认算法行为变化。

时序和停止

区间开始按原代码计算：`error=x_hat-x_bar`，姿态三个分量 `atan2(sin(error),cos(error))`；`feedback=-K@error`；`force=clip(f_bar+feedback, model.thrust_min, model.thrust_max)`。不重新计算 K，不添加噪声，不再使用规划 Kalman gain 更新融合状态。

按轨迹自身 `time[k]` 等待。默认起始迟到超过25 ms终止；即使没超过25 ms，只要已经越过该区间末端也终止。不得快速补历史指令。持有期间每5 ms轮询状态/通信，40 ms重发相同PWM；不重新计算反馈或根据电压变动重新映射同一区间。持有区间先判断缓存是否仍有效，再刷新输出；缓存过期后不能靠持续续租保留旧推力。状态失效、心跳中断、设备/参考变化、用户信号中止等进入清理；主机失效则由飞控最多200 ms的到期保护回中，详见固件说明。

主机在旧 `execute_lbts_plan` 中仅增加必要执行钩子：`poll/set_interval_deadline/on_step/require_stop_confirmation`，原有回调仍可用。原来的解锁/停止函数默认会发 MANUAL_CONTROL，不能直接沿用；本入口只复用其连接、消息处理、日志及心跳识别，解锁使用相同标准 MAVLink 命令而不发手动中立。

正常/异常都先请求本会话 STOP，必须收到六路 HAL 回读1500，再请求上锁并等待**新收到的**已上锁心跳，最后 RELEASE 并收到 IDLE。任一步未确认都报告 `stop_unconfirmed`，不报告完成。HAL readback只是飞控驱动端证据，不是电气测量或物理推力。原执行器的 `completed` 被命令入口写成 `sequence_completed`；`tracking_success=null`，不据时序完成自动判跟踪成功。

主机锁文件只协调遵守该锁的新执行器，不保证旧程序遵守此锁；飞控租约期间隔离旧 goto、trajectory、摇杆、GUIDED/姿态目标、servo/motor test和参数写入。ACQUIRE 要求已上锁且最近1秒无旧输入，因此不会自动接管活跃控制器。操作时仍应先停掉旧程序和QGC摇杆输出；QGC可保留只读遥测。正常退出后恢复旧入口。没有使用“六路力→四轴MANUAL_CONTROL”的替代方案。

实验日志

复用 `SessionLogger` 写 JSONL/CSV，原始消息与附加事件在 `logs/force/`：

- `control_step`：计划/实际时间、完整标称和实测状态、分量年龄/时间戳、误差、标称力、反馈修正、限幅前力、算法限幅后力和饱和。
- `force_mapping`：请求力、硬件采用力、量化PWM对应的官方静态估计力、发送PWM、电压及来源、硬件饱和、发送序号和主机时间。
- `pwm_frame`：包括保持重发的完整六路、序号、飞控到期时刻；`driver_readback`：应答序号、实际HAL回读、状态及接收时刻。
- `stop`：退出原因与停止确认，明确 `physical_thrust_confirmed=false`。

命令 `--output` 保存汇总 JSON；有控制步记录时保存同名 `.error.png`、`.output.png`。发送PWM与驱动端回读分别命名，不把发包成功当成推力实现。无需网页。

从六路PWM到短轨迹的操作顺序

1. 先读 `reports/verification_live_final.json` 和 `firmware/ardusub_3_5_4/README_zh.md`。历史只读报告读到3.5.4，已上锁MANUAL，六路1100/1500/1900范围，普通PWM，输出中立，当时电压15.960 V。它不代表本轮重新测量，历史证据见 `reports/verification_live_final.json`。只读复核命令：`python3 tools/diagnose_direct6.py`，它不切模式、不解锁、不写参数。
2. 在隔离源码区按精确版本应用补丁，完成SITL/对应板卡构建、接口路由/完整六路/看门狗/停止回读核验。**不自动刷写。** 未完成前保持 `firmware_verified=false`。完整 SITL 本轮通过15项联测；对应真实板卡构建和部署、电气输出测量尚未完成。
3. 停止旧输出入口，保持飞控已上锁MANUAL。纯 PWM 不要求将轨迹安装/定位/映射记录改成 true；入口会自动核实版本、电气参数和 direct6 能力。先执行全1500的0.5秒PWM测试，验证退出时有中立、上锁和释放证据，再在适合推进器运行的水中逐个测试小幅±PWM，确认1～6编号、作用力方向和通道对应。
4. 将实际核验结果写入唯一 `mapping`，再设置 `output.channel_mapping_verified=true`。核实模型标签、编号、位置和方向的实际对应；不能仅因注释矛盾改数字或符号。如确实要改原模型，先确认影响，本次未修改。核验兼容后才设置 `model_geometry_compatible_verified=true`。
5. 历史记录曾收到 A50 数据；现场应核实当前有效性并沿用已保存参考，不重复建立水面原点；核实姿态基准、sensor/world速度属性、安装旋转、杆臂及状态延迟。设置真实 `state` 参数并在核验后设置 `frame_verified=true`。任务中不复位设备和参考文件。
6. 使用原规划器生成该固定坐标内、起点接近当前状态的短计划；`save_force_plan` 导出，离线检查。显式一次 `plan ... --execute` 完成预检、解锁、时序执行、回中和上锁，发生异常不自动重试。
7. 分开验收输出与跟踪：先依据完整帧、HAL回读及实际测量确认独立输出；再用记录的状态、误差和图评价轨迹结果。系统显示 `sequence_completed` 仅表示时序完成，不能作为跟踪成功。

算法对齐与本轮实际测试

`reports/interface_revision_20261103/matlab_alignment.json` 和配套 `.input.mat/.reference.mat` 保存固定输入、原 `.m` 输出、源文件SHA256和误差。原始MATLAB是Heavy八推进器，不能当六推进器已移植版本；测试分别比较原八路模型以及把同一六路几何注入通用`.m`动力学的结果。对原有限差分函数的三处写死8维只在临时测试副本中替换为 `numel(u)`，不改原文件和运算。

已比较每种模型24组非零姿态/速度/推力的动力学与RK4，Jacobian，三个非等间隔区间的Riccati增益/Joseph协方差传播和代价，六路姿态包络/饱和反馈，以及原反向Dijkstra。六路动力学最大绝对误差约 `8.88e-16`、RK4 `5.55e-17`、反馈增益 `8.16e-10`；反馈运算与姿态误差结果在测试输入下相同。

差异：原 MATLAB `chanceCollisionProbability.m` 在正定位置协方差时用Cholesky生成样本，Python始终用特征分解。共享同一个200点Z时分布等价，有限样本位置却不同，复现实例风险Python=0.090、原.m=0.085。这可能影响靠近机会约束阈值时的边接纳/搜索结果。最小候选补丁保存在 `reports/proposed_chance_alignment.patch`，**没有应用**。需要用户确认后才修改原算法并重新验证，不能声称Python已经全部对齐。

重新运行对齐（安装Octave或使用本地MATLAB；无需机器人）：

```bash
python3 tools/verify_matlab_alignment.py --engine octave
# 或
python3 tools/verify_matlab_alignment.py --engine matlab --executable /实际路径/matlab
```

存在差异时脚本退出码2是预期的差异报告，不是掩盖失败。当前结果来自临时解包的Octave6.4.0，不修改系统安装；并未运行MATLAB本体。完整OCP、采样搜索和MATLAB CasADi数值对齐仍待适配环境。软件和原生固件核心测试：

```bash
python3 -m pytest -q
(cd .. && python3 -m pytest -q tests)
```

本轮实际结果见 `reports/interface_revision_20261103/project_tests.txt`、`legacy_tests.txt` 和 `direct6_sitl.json`。完整 Linux SITL 已通过；尚未运行实机板卡构建/部署、电气输出同步/看门狗测量、Companion 实机消息路由、通道物理方向和水中轨迹。状态时间回放仍会过期停止；软件和 SITL 通过不等于真机通过。

接口修订的复现命令（不接触真机）

```bash
cd /home/tj/blueROV2/code
python3 -m pytest -q tests
(cd /home/tj/blueROV2 && python3 -m pytest -q tests)
python3 tools/replay_force_state.py reports/connected_validation.raw.jsonl --output reports/state_replay.json
python3 tools/verify_direct6_sitl.py --binary firmware/ardusub_3_5_4/sitl_artifacts/ardusub --output reports/direct6_sitl_revision.json
```

SITL 测试自行启动本机进程并仅连接 `127.0.0.1:15760`。包括完整六路、抢占拒绝、旧控制覆盖隔离、无主机 finally 的固件超时回中、迟到命令锁存，以及无定位/曲线配置的实际 PWM CLI 正常与 SIGTERM 退出。HAL 回读不是实机电气测量。

`output.firmware_verified` 当前作为轨迹安装/物理验证记录保留；纯 PWM 不使用它代替接口检测，而是只读核实版本/电气参数并自动 QUERY direct6 能力。`channel_mapping_verified`、`model_geometry_compatible_verified`、`state.frame_verified` 仍为待实机核验记录，未改成 true。详情及后续 PWM→短轨迹步骤见修订报告。


2026-11-05 验证收尾说明

本轮运行结果见 [验证收尾报告](reports/closeout_20261105/REVIEW_zh.md)，旧报告保留其原轮次含义。

- 正常区间衔接：按原计划时间产生一次新推力并保持；边界不额外发送中立脉冲。整个任务结束或主机检测异常则主动进入原停止流程。若算法没有下一段输出，旧区间截止后 poll 不再续租；主机崩溃、断链或仅心跳继续时，飞控按最后已接纳帧的绝对到期时间回中并锁存。配置 200 ms 是租约上限，不是实测板卡停机时间。
- 推力数组（请求、采用、静态估计、饱和）按模型索引 0..5；PWM 和 HAL 回读按 MAIN1..6。每个推力运行报告及会话日志保存一次 mapping。图的 PWM 标签注明对应模型索引与曲线符号，颜色对应同一模型推进器；不要按两张图的列号直接配对。
- 离线 thrust-test 的 preview 使用明确配置的 output.test_voltage_v；为 null 时只检查输入，尚未核验该电压下的能力。执行时先读取新鲜 SYS_STATUS/BATTERY_STATUS，实际发送时重新读取电压并保存 output（以及 force_mapping 日志）。缺有效遥测且 test_voltage_v 为 null 时拒绝非零推力；明确配置非 null 值就是允许固定电压实验，来源始终标为 configured_test_voltage。该配置同时用于离线预览及实验备用电压；正式遥测实验请保持 null。预览不代替真正发送时的 output，刷新保持原 PWM，不随电压重算。

本机完整 SITL 和只读检查命令（前者仅运行本机模拟器）：

```bash
cd /home/tj/blueROV2/code
python3 tools/verify_direct6_sitl.py --binary firmware/ardusub_3_5_4/sitl_artifacts/ardusub --base-port 15760 --instance 41 --output reports/new_sitl_run.json
python3 tools/diagnose_direct6.py --output reports/new_readonly_run.json
```

SITL 使用临时目录和独立实例；占用端口时拒绝启动，可选其他 base-port/instance，不结束其他用户服务。只读诊断在占用的 UDP 端点上拒绝共享；无设备心跳时有界退出，不要求 DVL 在线。未收到专用应答只能表示该窗口未观察到应答，仍需区分补丁能力、目标和 Companion 路由。
