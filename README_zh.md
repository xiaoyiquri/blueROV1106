# BlueROV2 基础通信与单轴运动测试

本目录提供一个 Python 3 + `pymavlink` 小工具，用于旧版 Companion、Pixhawk/ArduSub BlueROV2 的第一阶段验证：识别真实飞控、读取遥测、在 MANUAL 模式发送短时低幅度单轴 `MANUAL_CONTROL`，然后回中、请求上锁并记录结果。

基础手动工具让 ArduSub 负责六推进器混控，不直接写电机 PWM，也不把 `level` 解释成速度、距离、推力或 PWM。单目标点控制和条件式 A50 外部导航桥见后文；当前阶段仍不重新编写 EKF、动力学辨识或通用路径规划。

## 安装

不需要先全面升级 Ubuntu。建议使用项目自己的虚拟环境：

```bash
cd /home/tj/blueROV2
python3 -m venv .venv
. .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python -c 'import pymavlink; print(pymavlink.__doc__)'
```

如果系统没有 `venv`，安装与发行版匹配的 `python3-venv` 包即可。`pymavlink` 版本和 Python 版本会影响旧固件兼容性；发现兼容问题时请把 `python --version` 和 `python -m pip show pymavlink` 一并记录。

## 先确认通信接口

USB 物理连接不等于串口通信。USB 线可能只是把电脑接入树莓派的网络、串口设备可能留在树莓派上，也可能由 Companion 把 MAVLink 转成 UDP。先在 Ubuntu 执行：

```bash
ip -br addr
ip route
ss -lunp
ls -l /dev/serial/by-id/ 2>/dev/null
```

根据实际输出选择端点：

* `udpin:0.0.0.0:14550` 表示本机监听 UDP 14550；`14551` 只是另一个常见端口示例，不能预设它一定可用。
* `/dev/serial/by-id/实际设备路径` 表示本机能打开的串口；使用串口时另行给出 `--baud`。如果串口在树莓派而不在 Ubuntu，本机的 `/dev/serial/...` 不会凭空出现。
* `udpout:地址:端口`、`tcp:地址:端口` 等可由 `pymavlink` 支持，但必须先确认 Companion 的路由方向和端口。

首次诊断建议退出 QGroundControl，避免同一个 UDP 接收端口或串口被争用。若必须与 QGroundControl 并行，必须在 Companion 端配置独立、可双向使用的 MAVLink 路由端点；仅把 Python 的端口号改掉不能创建路由。QGroundControl 留作监视时，关闭其操纵杆输入，避免它与本程序同时发送控制。程序不会主动重启飞控、改 Companion 路由、改持久参数或关闭保护机制。

## 子命令

三个子命令都要求显式 `--connection`，不会自动猜测端点：

* `diagnose`：默认 15 秒连接并收集遥测；不解锁、不切换模式、不发送 `MANUAL_CONTROL`。它会发送本程序的 GCS `HEARTBEAT`，这是为保持链路和诊断而发的 MAVLink 消息，不等于执行器控制。可选请求 `AUTOPILOT_VERSION` 或消息频率；旧固件拒绝时记录 `COMMAND_ACK` 并继续降级。
* `status`：诊断别名，默认 5 秒，不主动请求额外版本/频率消息。
* `move`：显式运动操作。连接、识别飞控、确认当前未解锁后，请求 MANUAL，等待后续心跳确认，持续中立输入，发送非中立单轴动作，回中，再请求上锁并等待上锁心跳。

### 诊断示例

```bash
python rov_control.py diagnose \
  --connection udpin:0.0.0.0:14550 \
  --duration 15 \
  --request-version \
  --message-interval ATTITUDE=10
```

串口示例（路径和波特率必须按现场替换）：

```bash
python rov_control.py status \
  --connection /dev/serial/by-id/实际设备路径 \
  --baud 115200 \
  --duration 5
```

诊断结束会显示目标 system/component、模式、解锁状态和接收消息数，并在 `--log-dir`（默认 `logs/`）留下 JSONL 与 CSV。诊断“未发送执行器控制”不表示“完全没有 MAVLink 消息”：程序心跳和显式的版本/频率请求仍可能发送。

### 单轴动作示例

默认软件幅值上限是水平/偏航 400、垂直偏移 200，默认动作时长 1 秒，默认时长上限 5 秒（硬上限 30 秒）。可显式把上限调低；即使调高，也只能在协议范围内，不能保证实际推力或速度按比例变化。

```bash
python rov_control.py move \
  --connection udpin:0.0.0.0:14550 \
  --action forward \
  --level 100 \
  --duration 1.0
```

如果链路上有多个自动驾驶候选目标，请明确指定目标：

```bash
python rov_control.py move \
  --connection udpin:0.0.0.0:14551 \
  --target-system 1 \
  --target-component 1 \
  --action yaw-left --level 80 --duration 0.8
```

串口动作示例：

```bash
python rov_control.py move \
  --connection /dev/serial/by-id/实际设备路径 \
  --baud 115200 \
  --action up --level 60 --duration 0.7
```

可用动作是 `forward`、`backward`、`left`、`right`、`yaw-left`、`yaw-right`、`up`、`down`。负时长、零时长、NaN、无穷值、负 level、非法动作和超过协议/软件上限的值都会在连接或解锁之前拒绝。

## 分段轨迹执行器

[rov_trajectory.py](/home/tj/blueROV2/rov_trajectory.py) 接收 JSON 轨迹文件，把它展开为多个定时 `MANUAL_CONTROL` 段。它复用相同的目标识别、MANUAL 模式确认、解锁、20 Hz 发送、心跳监视、回中和上锁清理流程。

先只解析轨迹，不连接飞控：

```bash
python3 rov_trajectory.py run \
  --connection udpin:0.0.0.0:14550 \
  --file examples/sequence.json \
  --dry-run
```

确认文件后执行：

```bash
python3 rov_trajectory.py run \
  --connection udpin:0.0.0.0:14550 \
  --file examples/sequence.json \
  --transition-neutral 0.2 \
  --log-dir logs
```

`examples/sequence.json` 展示“前进→下潜→后退→上浮”。每段必须给出 `action`/`level`/`duration`，也可直接给 `axes: {"x": ..., "y": ..., "z": ..., "r": ...}`；段与段之间默认插入 0.2 秒中立输入。轨迹总时长默认上限 60 秒，单段默认不能超过 30 秒，水平/偏航和垂直幅值沿用基础工具的保守上限。

`examples/circle_open_loop.json` 展示圆周近似：它同时发送前进和偏航输入。`radius_m` 只是计划目标和日志元数据，`forward_level`、`yaw_level`、电池电压、推进器负载和水流共同决定实际曲率；没有位置/DVL 闭环时，程序不能保证机器人从当前位置画出精确半径或完整圆，也不能把时间换算成米。需要真实半径、闭合误差和轨迹跟踪时，应另行加入经过核验的位置反馈和控制器。

圆周近似示例：

```bash
python3 rov_trajectory.py run \
  --connection udpin:0.0.0.0:14550 \
  --file examples/circle_open_loop.json \
  --log-dir logs
```

轨迹执行仍是显式运动操作：启动不会隐式解锁，异常、Ctrl+C、心跳过期、模式变化和飞控自行上锁都会停止非中立输入；执行结束会回中并请求上锁。它不会直接修改六个推进器输出、持久参数或 ArduSub 混控矩阵。

## 单目标点闭环控制（定位就绪后使用）

[rov_goto.py](/home/tj/blueROV2/rov_goto.py) 是独立的单目标点控制程序。它只向 ArduSub 发送 `SET_POSITION_TARGET_LOCAL_NED`，切换到 GUIDED 后由飞控位置控制器和六推进器混控执行；GUIDED 阶段不会发送 `MANUAL_CONTROL`。程序从飞控 `LOCAL_POSITION_NED` 读取位置/速度，并从 `EKF_STATUS_REPORT` 检查估计器状态。连接端点、波特率、来源 ID、超时和到达阈值都可放在 [goto_config.json](/home/tj/blueROV2/goto_config.json)，命令行目标坐标覆盖目标值。

任务坐标固定为 NED：+X 北、+Y 东、+Z 向下。程序连接后先检查心跳、外部导航配置、EKF 标志、位置/速度新鲜度，再在静止窗口采集任务参考点；它不重置飞控 EKF 原点或 Home，也不能仅凭一帧位置证明机器人在水面。目标 `(2,3,1)` 会转换为“任务参考点 + (2,3,1)”的飞控局部 NED 目标。

只读检查命令：

```bash
python3 rov_goto.py \
  --connection udpin:0.0.0.0:14550 \
  --config goto_config.json \
  --check
```

目标执行命令：

```bash
python3 rov_goto.py \
  --connection udpin:0.0.0.0:14550 \
  --config goto_config.json \
  --x 2 --y 3 --z 1 \
  --log-dir logs
```

到达判定同时要求水平误差、垂直误差和三轴速度满足 `goto_config.json`，并持续保持 `arrival_hold_s`。定位过期、EKF 标志异常、模式改变、飞控自行上锁、目标更新/任务超时或用户中止都会停止发送目标并请求上锁；程序不会人工确认到达，也不会用回中 `MANUAL_CONTROL` 代替取消 GUIDED 目标。

### 当前 DVL/EKF 现状

已检查 `/home/tj/blueROV2/recording/`：

* [dvl_a50.py](/home/tj/blueROV2/recording/src/dvl-a50/scripts/dvl_a50.py) 和 C++ 节点从 `192.168.2.95:16171` 读取 A50 JSON，发布 `/dvl/data` 速度和 `/dvl/position` DVL 自身航位推算；没有把数据发回 ArduSub。
* [mavlink_bridge.py](/home/tj/blueROV2/recording/src/recording/recording/mavlink_bridge.py) 只监听 `udpin:0.0.0.0:14600` 并发布 ROS IMU/气压/电机输出；它没有 DVL 注入，而且 IMU orientation 被固定为单位四元数，不能用于估计器姿态融合。
* 现有真实日志没有 `LOCAL_POSITION_NED`，最新检查到的 `EKF_STATUS_REPORT` 标志不足以同时证明水平位置和水平/垂直速度有效，因此 `rov_goto.py` 会拒绝启动导航。

这意味着现有 ROS DVL 服务目前不能作为“已接入飞控”的证据；它可以继续用于观察和记录，但不要同时启动两条 DVL→飞控注入链。新增桥只在确认没有可复用注入服务、且固件支持时使用。

你的飞控版本已从 `AUTOPILOT_VERSION` 核实为 ArduSub 3.5.4。官方旧版 DVL 集成文档明确说明其 `VISION_POSITION_DELTA` 接入从 ArduSub 4.1-beta 起支持；3.5.4 不能被本项目静默当作支持版本使用。[官方 DVL 集成说明](https://www.ardusub.com/developers/dvl-integration.html) 还要求 EKF3、关闭 EKF2、`VISO_TYPE=1` 和 `EK3_SRC1_* = 6` 等配置，并以 “FUSING ODOMETRY” 等状态作为融合证据。

位置估计消息的官方兼容路径和坐标/质量语义见 [Non-GPS Position Estimation](https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html)；GUIDED 位置目标的消息、坐标系和 `type_mask` 规则见 [ArduSub Modes](https://ardupilot.org/sub/docs/modes.html)。

因此本项目不自动升级固件、修改 Companion 路由或写入持久参数。若后续确认要使用官方 A50 接入链路，需要人工完成适用版本升级和参数配置，再重新运行 `--check`。相关参数现值应先读取确认：`EK3_ENABLE`、`AHRS_EKF_TYPE`、`EK2_ENABLE`、`VISO_TYPE`、`EK3_SRC1_POSXY`、`EK3_SRC1_VELXY`。修改后重启飞控，再检查 `STATUSTEXT`、`EKF_STATUS_REPORT` 和 `LOCAL_POSITION_NED`；程序只读取并核对这些参数，不会替用户修改。

### 不改变当前固件时：主机侧 DVL 外环

如果当前 ArduSub/Companion 不能注入外部导航，又不希望修改版本，可以使用 [rov_host_goto.py](/home/tj/blueROV2/rov_host_goto.py)。这条链路是：`A50 TCP JSON → Ubuntu 位置误差控制器 → MANUAL_CONTROL → ArduSub 混控`。ArduSub 仍负责操纵输入到六个推进器的分配，但不参与位置闭环；程序不发送 `SET_POSITION_TARGET_LOCAL_NED`，也不会把主机估计冒充飞控 EKF。

主机外环的任务原点保存在 `host_goto_config.json.origin_file`（当前为 `task_origin.json`）。第一次任务会从当前静止位置建立并保存原点；后续启动读取同一个原点，因此重复输入同一坐标时，程序会判断机器人是否已经在该固定世界/任务坐标目标附近，不会再次把当前位置当成零点。若确实要开始新的实验坐标系，必须在机器人静止、DVL 已复位且确认旧任务结束后，人工删除这个文件再启动。

主机外环使用 A50 的 `position_local` 航位推算和有效速度报告，在任务开始时取一段静止样本作为参考点，以固定任务坐标计算误差，再使用“当前 DVL yaw 减去复位时 yaw”的相对航向把误差转换为机体前/右/上输入。这样在局部任务坐标中，复位时的 `+X` 对应前进、`+Y` 对应右移；直接使用绝对 yaw 会把两个方向混合。A50 JSON 的位置单位是米、姿态角单位是度，代码会在进入控制计算和 MAVLink 角增量前分别转换到内部米/弧度表示。正 `z` 目标仍表示向下；MANUAL_CONTROL 的 `z` 正偏移表示上浮，所以代码对此做了反向映射。

这条路径接受 DVL 航位漂移、姿态漂移、水流、安装矩阵和增益误差，`(2,3,1)` 只能解释为相对于本次 DVL/任务参考的控制目标，不能声称是真实北东地理坐标或精确到达。当前已根据现场确认采用 `task_frame=local_at_dvl_reset`：复位瞬间 DVL 指示灯方向为任务 +X、右侧为 +Y、向下为 +Z，单位矩阵和零 yaw 偏移只对这个局部任务定义有效；它不等同于地理北东 NED。

```bash
# 先关闭 QGroundControl 操纵杆；确认没有另一个程序同时占用 A50 TCP 数据流
python3 rov_host_goto.py \
  --config host_goto_config.json \
  --x 2 --y 3 --z 1
```

现有 `recording/src/dvl-a50/` 节点也是 A50 TCP 读取器，但它只发布 ROS `/dvl/data` 和 `/dvl/position`，没有向飞控注入定位。主机外环默认直接读取 A50 `192.168.2.95:16171`，应确保同一时刻只有一个程序承担该设备的 TCP 读取；如果现场 ROS 节点已经独占连接，应停用它或改成单一转发服务后再运行外环，不能为了“多一份数据”启动多条读取/注入链路。主机外环的 `--check` 尚不执行控制，实际运动前仍须人工核对 DVL 速度有效标志、四波束状态、安装方向、缆绳和现场停止措施。

先做只读 DVL 检查（不会连接飞控、解锁或发送 `MANUAL_CONTROL`）：

```bash
python3 rov_host_goto.py --config host_goto_config.json --check
```

检查输出中的 `dvl_transport=OK` 只表示 TCP 报文、速度有效标志和波束数据可读；必须同时看到 `dvl_position_quality=PASS`，且 `position_std_m` 不超过配置上限。质量不合格时命令返回码为 2，即使速度接近零也不能开始目标运动。

### A50 静止陀螺校准和航位推算复位

执行前先关闭 QGroundControl 操纵杆，并确认没有 ROS DVL 节点或其他程序持续读取 `192.168.2.95:16171`。可以先查找进程：

```bash
ps -ef | grep -E 'dvl_a50|dvl-sensor|ros2|mavlink_bridge' | grep -v grep
```

停止实际启动 DVL 节点的终端或服务；不要盲目执行 `killall`，也不要同时保留多个 A50 TCP 读取器。

1. 在 Ubuntu 浏览器打开 DVL 地址。按当前现场配置通常是 `http://192.168.2.95/`；如果页面打不开，使用实际 DVL 地址和 `ip route` 检查网络。当前程序能读到 `192.168.2.95:16171`，只证明 TCP 数据端口可达，不代表 Web 地址或配置已核验。
2. 将 DVL 固定在实际安装姿态，保持整机、DVL 支架、线缆和桌面都不动。校准期间不要手持移动、碰撞支架或启动推进器。
3. 在 Web GUI 的 Dashboard/More 菜单选择 **More → Calibrate gyro**，等待页面报告成功后再进行下一步。A50/A125 的校准结果会保存到设备；不需要每次上电重复校准。若页面提示失败，先保持静止并检查温度、安装振动，再记录提示信息。
4. 让机器人进入实际试验水域，DVL 朝向底部，四个波束有清晰的底部视线。确认声学开关已启用、`velocity valid` 有效、底锁稳定、海拔和 FOM/不确定度合理。不要在空气中把漂移的航位推算当成测试结果；官方说明 DVL 在空气中运行会产生很大的位置漂移。
5. 在机器人仍然静止时，在 Dashboard 的 dead-reckoning 区域点击 **Reset**（有些 GUI 版本把它放在 Configuration/maintenance 的 Reset dead reckoning）。这是任务参考的建立动作，会把当前位置设为新的航位推算原点，并把 roll/pitch/yaw 参考置零；复位后约几十毫秒内才会反映到输出。
6. 等待几秒后重新运行：

   ```bash
   python3 rov_host_goto.py --config host_goto_config.json --check
   echo $?
   ```

   期望看到 `status=0`、`velocity_valid=True`、`beams_valid=True`、`dvl_position_quality=PASS`，并且 `position` 从接近零开始、`position_std_m` 明显低于 `5.0`。这个 5 米是本项目的保守软件门槛，不是 A50 的精度承诺。
7. 保持机器人不动，连续执行几次检查，确认位置没有突跳、速度接近零、yaw 不快速变化。若 `position_std_m` 仍为数百/数千米，或 yaw 在静止时快速变化，不要设置 `frame_transform_verified=true`，也不要执行 goto；应保存 Web GUI 截图和 DVL diagnostic log，检查底锁、气泡、振动、安装方向、温度和其他声学设备。

官方流程和限制见 [A50/A125 Dead reckoning](https://docs.waterlinked.com/dvl/dead-reckoning/)、[A50/A125 Web GUI](https://docs.waterlinked.com/dvl/dvl-a50_a125-gui/) 和 [Settings and configuration](https://docs.waterlinked.com/dvl/configuration/)。程序不自动发送 `calibrate_gyro` 或 `reset_dead_reckoning`，避免在错误时机改变 DVL 参考；如 Web GUI 不可用，应先停止其他 TCP 读取器，再根据对应 JSON 协议确认命令和 ACK。

这条路径的控制责任仍然是显式的：启动时已解锁会拒绝接管；确认进入 MANUAL、解锁后才发送非中立输入；到达、超时、定位过期、模式改变或 Ctrl+C 会停止输入并有界请求上锁。通信断开时 Python 不能保证最后一条消息送达，必须单独核实飞控失联保护。

在兼容固件上，[dvl_mavlink_bridge.py](/home/tj/blueROV2/dvl_mavlink_bridge.py) 可以读取 A50 TCP JSON 并发送 `VISION_POSITION_DELTA`。它要求用户显式提供已经核验的 DVL→NED 3×3 安装旋转矩阵：

```bash
python3 dvl_mavlink_bridge.py \
  --connection <已配置的飞控双向端点> \
  --dvl-ip 192.168.2.95 \
  --dvl-port 16171 \
  --rotation r00 r01 r02 r10 r11 r12 r20 r21 r22
```

桥接程序会检查飞控版本低于 4.1 时直接退出，不发送不兼容消息；它不会把 DVL 自身位置直接冒充飞控位置，也不会默认使用单位矩阵。当前 3.5.4 上不要运行它来“试试看”。

两条路径是互斥的运行方案：

| 方案 | 需要的飞控条件 | Ubuntu 发送内容 | 定位闭环由谁完成 | 当前设备结果 |
|---|---|---|---|---|
| 主机侧外环 | 现有 3.5.4 也可运行，前提是 DVL TCP 和安装矩阵已核验 | `MANUAL_CONTROL` | Ubuntu 根据 A50 航位推算计算外环 | 可以作为不升级版本的折中方案，接受漂移和控制精度限制 |
| ArduSub EKF/GUIDED | 必须是支持外部导航注入的 ArduSub/Companion 组合，并正确配置参数 | DVL 注入消息；目标用 `SET_POSITION_TARGET_LOCAL_NED` | ArduSub EKF 与 GUIDED 位置控制器 | 当前已核实的 3.5.4 缺少所需链路，`rov_goto.py --check` 应拒绝启动 |

不要同时运行主机外环、DVL MAVLink 注入桥和其他位置控制程序。第一种方案不需要 `LOCAL_POSITION_NED`，第二种方案必须看到新鲜的飞控位置/速度、有效 EKF 标志以及外部导航参数和状态证据；“收到 DVL JSON”或“收到某条位置消息”单独都不够。

## ID、模式和轴映射

程序发送自己的 GCS 心跳，默认 `source-system=255`、`source-component=190`；可用参数覆盖。source ID 是“谁在发消息”，与 `target-system/target-component`（“消息发给哪一个飞控”）不同。程序只接受带自动驾驶类型的 `HEARTBEAT` 作为飞控候选，过滤 GCS、相机和其他组件；指定目标后仍会从后续心跳确认状态。多个候选目标时程序报错，不随意选择。不要把飞控自身 ID 当成 source ID，也不要未经诊断改 GCS 接受参数。

`MANUAL_CONTROL` 按 ArduSub 的标准/历史兼容映射实现：

| 轴 | 协议范围 | 中立 | 本程序含义 |
|---|---:|---:|---|
| `x` | -1000..1000 | 0 | 前后（正为前进） |
| `y` | -1000..1000 | 0 | 横移（负为左，正为右） |
| `z` | 0..1000 | 500 | 垂直（正偏移为 up，负偏移为 down） |
| `r` | -1000..1000 | 0 | 偏航（负为 yaw-left，正为 yaw-right） |

每次动作都发送 `buttons=0`，其余三个轴保持 `x=0, y=0, z=500, r=0`。这里的 x/y 是机体平移操纵量，不能解释为期望俯仰/横滚角。不同装配、机架方向和电机方向可能让现场的“左/右/前后”与预期相反，第一次必须在适当水域以更小幅度观察，并立即保留现场停止方式。

### 推进输入、ArduSub 混控和参数

本程序的 [rov_control.py](/home/tj/blueROV2/rov_control.py) 只决定归一化操纵量：`map_action()` 把 `forward` 变成 `x`，`run_timed_action()` 按 20 Hz 发送 `MANUAL_CONTROL`。它不直接改变六个 PWM，也不替代 ArduSub 的混控。

当前飞控报告的固件是 ArduSub 3.5.4。该版本的实际链路是：`GCS_MAVLink_Sub` 接收 `MANUAL_CONTROL` → `joystick.cpp` 把轴转换为 RC override → `mode_manual.cpp` 读取 forward/lateral/throttle/yaw → `AP_Motors6DOF` 按 `FRAME_CONFIG` 分配六个电机。3.5.4 的源码见 [joystick.cpp](https://github.com/ArduPilot/ardupilot/blob/ArduSub-3.5.4/ArduSub/joystick.cpp) 和 [Parameters.cpp](https://github.com/ArduPilot/ardupilot/blob/ArduSub-3.5.4/ArduSub/Parameters.cpp)。

3.5.4 中水平/偏航输入使用 `rpyScale = 0.4 × gain`。因此在 `JS_GAIN_DEFAULT=0.5` 时，`forward --level 250` 理论上约产生 `1500 + 50 = 1550` 的 forward 通道输出；这只是飞控输出指令，不是实际 RPM 或推力。`JS_THR_GAIN` 只缩放垂直 `z` 轴，不能用来增大前进推力。

本机当前读到的关键参数如下：

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `JS_GAIN_DEFAULT` | 0.5 | 水平、偏航操纵增益；范围 0.1..1.0 |
| `JS_GAIN_MIN/MAX` | 0.25 / 1.0 | 增益边界 |
| `JS_THR_GAIN` | 1.0 | 仅垂直 z 轴增益；范围 0.5..4.0 |
| `MOT_SPIN_MIN/MAX` | 0.15 / 0.95 | 电机输出低端/高端归一化范围，不是直接 RPM 设置 |
| `RC_SPEED` | 200 | ESC 信号更新频率，不改变稳态推力幅值 |
| `MANUAL_SLEW_RATE` | 100 | 手动输出变化斜率，影响响应快慢，不改变最终输入幅值 |
| `FRAME_CONFIG` | 1 | 当前为 Vectored，不能用来调大推力 |

想临时增大前进、后退、横移或偏航，优先提高命令的 `--level`，例如软件默认上限内使用 `--level 400`。提高 `--max-horizontal-level` 只是放宽本程序输入限制，不会改变 ArduSub 的增益。

想持久改变水平/偏航操纵增益，可在 QGroundControl 参数页面修改 `JS_GAIN_DEFAULT`，例如从 0.5 改到 0.75；降低到 0.35 会更柔和。该参数是启动时 joystick gain 的默认值，修改后是否需要重启取决于旧固件的参数加载方式；不要在推进器可能运行时修改或重启。

`MOT_SPIN_MIN`、`MOT_SPIN_MAX`、`MOT_PWM_MIN`、`MOT_PWM_MAX` 涉及 ESC 标定和输出边界。它们不应被当作“油门增益”随意修改；当前设备报告的 `MOT_PWM_MIN/MAX=0` 也不能据此盲目写成 1100/1900。先按 ESC/推进器制造商范围完成独立标定，再决定是否调整低端启动阈值。不要修改 `SERVO1_FUNCTION..6` 或 `FRAME_CONFIG` 来解决幅值问题；当前 `SERVO1_FUNCTION..6=33..38` 已对应 Motor1–Motor6。

模式切换和解锁都不使用强制参数。程序从后续飞控 `HEARTBEAT` 确认模式/解锁，再进行下一步；`COMMAND_ACK` 只作为另一份证据，不能用“发送函数没抛异常”代替。启动时若已解锁，程序拒绝自动接管，不会在失败清理中擅自上锁。解锁请求已发出但确认超时时，程序把责任标记为“可能已接管”，进行有界回中和上锁尝试。

操纵消息约 20 Hz，程序心跳 1 Hz，状态摘要约 1 Hz；调度和超时使用 `time.monotonic()`。接收采用非阻塞短轮询，并限制每轮的消息数量/时间预算，避免遥测洪水饿死控制发送。不要把程序的 `finally` 当作独立安全装置：断电、强制杀进程或网络完全断开时它无法运行。飞控端 GCS/Pilot control failsafe 必须单独核实，不能保证“断线一定停机”。

## 状态、遥测和日志

运动流程状态为 `DISCONNECTED → CONNECTED → MODE_CONFIRMING → NEUTRAL → ARM_CONFIRMING → MOVING → STOPPING → DISARM_CONFIRMING → DONE`；错误进入 `FAULT`。Ctrl+C、终止信号、心跳过期、模式意外变化、飞控自行上锁、链路异常和确认超时都会停止非中立输入，不会自动重新解锁或恢复运动。在有控制责任且链路仍可用时，清理会有界发送中立并请求上锁。输出/日志区分：

* **已确认上锁**：收到上锁后的 `HEARTBEAT` 解锁标志为 false；
* **已请求上锁但未确认**：发出了命令或收到了 ACK，但没有及时得到上锁心跳；
* **链路断开、最终状态未知**：无法继续取得心跳或发送清理消息。

JSONL 保存每条原始消息、发送的四轴值、UTC、本机单调时间、飞控 system/component、消息中可用的 `time_boot_ms`/`time_usec` 时间戳、状态机、动作、模式、解锁状态、`COMMAND_ACK`、`STATUSTEXT` 和退出原因。CSV 是便于分析的摘要。`ATTITUDE` 的角度和角速度原始单位分别是弧度、弧度/秒；接收时间不等于传感器采样时间，当前只建立可追溯时间记录，不假设 IMU/DVL 已精确同步。

电池字段保留 MAVLink 原始约定：电压 mV、电流 cA（厘安）和剩余百分比；未知/无效字段留空。

`SYS_STATUS`/`BATTERY_STATUS` 的无效值会保留为空。`LOCAL_POSITION_NED.z` 和压力相关字段原样保存并标记“深度来源/符号/零点待核实”；有位置消息不代表存在或启用了 DVL，也不会把 `VFR_HUD.alt` 或任意压力值直接称为深度。程序不从电机输出推断真实转速或推力。

查看结果：

```bash
ls -lh logs/
tail -n 5 logs/*_session.jsonl
python - <<'PY'
import csv, glob
path = sorted(glob.glob('logs/*.csv'))[-1]
with open(path, newline='', encoding='utf-8') as f:
    rows = list(csv.DictReader(f))
print(path, 'rows=', len(rows))
print(rows[-1] if rows else 'empty')
PY
```

定性响应（是否前后/横移/转向、姿态是否异常、是否回中）不能替代定量速度或距离测量。若将来要做定量分析，需另外核实深度、DVL 型号/接口/融合状态、时间同步、试验水域和标定方法。

## 安全与排查顺序

推进器动作只在适当的水中环境进行，人员、手、衣物和缆绳远离推进器；空气中不要长时间运行推进器。保留人工断电/现场停止措施，QGroundControl 的操纵杆必须关闭。

建议首次顺序是：

1. 关闭 QGroundControl 操纵杆（首次可完全退出），运行上面的 `diagnose`。
2. 检查输出的目标 ID、`mode`、`armed`、`STATUSTEXT`，打开 JSONL 看 `HEARTBEAT`、`COMMAND_ACK` 和版本消息。
3. 确认飞控在安全、未解锁状态；在水中以较低 `--level`、较短 `--duration` 运行一个方向。
4. 现场观察方向，确认动作结束后轴回到中立且日志显示“已确认上锁”。
5. 再逐个测试其他方向；不要把“能运动”当成速度、距离或 DVL 定位验证。

常见问题：

* **无心跳**：核对 IP/路由、端口、监听方向、防火墙、Companion 路由和 QGroundControl 端口争用；串口则核对设备路径、权限和波特率。
* **端口占用**：用 `ss -lunp` 找进程；先退出 QGroundControl 或在 Companion 配置独立双向端点。
* **遥测正常但控制无效**：确认 QGroundControl 操纵杆已关闭、模式确实是 MANUAL、飞控允许该 source ID，且程序收到持续心跳；不要直接改参数掩盖原因。
* **解锁被拒绝**：查看 `COMMAND_ACK` 与 `STATUSTEXT`，人工检查安全开关、漏水/电池/模式和飞控配置；程序不会使用 force arm。
* **方向反常**：立即停止，检查现场装配、电机方向和机架方向；不要由本程序自动改写电机或持久参数。
* **无法确认上锁**：保留日志，使用 QGroundControl/现场观察核实；“已请求”不等于“已确认”。
* **消息频率请求被拒绝**：旧版固件可能不支持 `MAV_CMD_SET_MESSAGE_INTERVAL`；记录失败后使用已有频率，不因此自动运动或盲目增加数据率。

## 官方资料

这些链接同时说明了旧版 Companion 的历史背景和 ArduSub 的兼容注意事项：

* [ArduSub Pymavlink（旧文档，已归档）](https://www.ardusub.com/developers/pymavlink.html)：连接、GCS 心跳、MANUAL_CONTROL、模式和解锁示例；旧示例含无限阻塞逻辑，本程序使用有界事件循环。
* [ArduSub Pilot Control](https://ardupilot.org/sub/docs/pilot-control.html)：MANUAL_CONTROL 及 z 轴 0..1000 的历史兼容说明。
* [ArduPilot MANUAL_CONTROL/RC input](https://ardupilot.org/dev/docs/mavlink-rcinput.html)：x/y/z/r 轴定义和范围。
* [ArduSub Pilot Control Failsafe](https://ardupilot.org/sub/docs/pilot-control-failsafe.html)：Pilot input 丢失行为及 `FS_PILOT_INPUT`。
* [ArduSub Failsafe 总览](https://ardupilot.org/sub/docs/failsafe-landing-page.html)：与模式、RC/GCS 失联相关的保护。
* [Blue Robotics：BlueROV2 R3 及更早版本软件设置](https://bluerobotics.com/learn/bluerov2-software-setup-r3-and-older/)：旧 R1/R2/R3 使用 Pixhawk + Companion 的背景、网络和 QGroundControl。
* [Water Linked A50/A125 TCP JSON API](https://docs.waterlinked.com/dvl/dvl-json-protocol/) 与 [DVL 坐标轴](https://docs.waterlinked.com/dvl/axes/)：A50 的 TCP 端口 16171、速度/航位推算字段、米/度单位、机体轴和安装旋转偏移。
* [MAVLink MANUAL_CONTROL](https://mavlink.io/en/messages/common.html#MANUAL_CONTROL)：协议字段；ArduSub 的 z 轴历史实现以 ArduSub 文档为准。

## 本地测试与实机未验证项

运行：

```bash
python -m unittest discover -v
python3 -m py_compile rov_control.py rov_trajectory.py rov_goto.py rov_host_goto.py dvl_mavlink_bridge.py
```

本地测试覆盖轴映射、协议/软件上限、NaN/非法参数拒绝、消息来源过滤、姿态单位保留、`buttons=0`、NED 原点/到达有效性、A50 角度单位转换、DVL 安装变换拒绝和主机外环垂直符号。未连接真实飞控或真实 DVL，因此尚未验证实际 Companion 路由、Ubuntu 网卡地址、串口设备、Pixhawk 硬件、ArduSub 固件版本、模式 ID、解锁前置条件、DVL/IMU 接口、GCS/Pilot failsafe 参数、安装矩阵或推进器方向。

第一次请执行：

```bash
ip -br addr; ip route; ss -lunp; ls -l /dev/serial/by-id/ 2>/dev/null
python rov_control.py diagnose --connection udpin:0.0.0.0:14550 --duration 15 --request-version
```

上面的端点只是命令格式示例，执行前请把它替换为前一步确认的真实端点。

继续实机排查时请提供：完整诊断命令（隐藏敏感地址亦可）、Ubuntu/Python/pymavlink 版本、`ip -br addr`/`ss -lunp`/串口列表、Companion 路由端点、JSONL/CSV 日志中目标 ID、`HEARTBEAT`、`COMMAND_ACK`、`STATUSTEXT` 和最终状态，以及 ArduSub/Pixhawk 固件版本。不要上传电池序列号、网络凭据或其他不必要的隐私信息。
