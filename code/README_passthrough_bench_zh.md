# 原版 ArduSub 3.5.4 直通电气台架验证

新增入口为 `python3 -m bluerov2.passthrough_bench`。**它是推进器无动力时的诊断程序，不是运动控制或轨迹执行后端。** 默认只读。它不使用 direct6、MANUAL_CONTROL、GUIDED、规划、DVL、原点、T200 曲线或模型推力方向，也不解锁、上锁或切换模式。

先前已在精确版本源码及 HAL 逐次跟踪中复现原电机循环覆盖直通值；新工具没有隐藏或修复这个固件问题。即使它报告电气台架观测完成，也不代表独立输出或真实轨迹验收通过。

## 本轮完成内容

- 新增 `bluerov2/passthrough_bench.py`，复用原 `rov_control.Controller` 的连接识别、有限收包和日志；关闭诊断心跳，避免只读检查维持其他会话的 GCS failsafe。
- 默认模式只允许版本请求及 35 项电气输出相关参数读取。输出/参数写入默认禁止，已占用的 UDP 端口不共享、不抢占。
- 显式隔离动力后，可将六路 FUNCTION 暂设为 1，在一个 RC override 包中发送六路固定 PWM；通道 7/8 忽略，其他通道不受测试指令控制。
- 参数修改前将实际读取的原值保存为 `original_parameters.json`，同步文件和目录到磁盘。部分配置失败、用户中断或数据/时序异常也进入回中、恢复及读回确认流程。
- 仅修改六个 FUNCTION，不修改 failsafe、输出范围、模型、增益或算法。原输出模式及反馈执行文件保持原样。
- 增加可控制时钟的离线测试，并复用 `tools/verify_passthrough_sitl.py --bench-check` 联测，没有另建 SITL 框架。

## 1. 默认只读

使用工程现有 Python 环境（pymavlink、pytest）即可；不需要 ROS2/MAVROS 或新的 Python 包。现有连接配置默认为 `/home/tj/blueROV2/host_goto_config.json`，只读取其中的通信字段。

```bash
cd /home/tj/blueROV2/code
python3 -m bluerov2.passthrough_bench
```

输出会打印本次报告路径。默认模式不会发送 RC override、PARAM_SET、心跳或解锁/模式指令。进入台架前要求当前已是未解锁的 MANUAL；程序不会代你改变这些状态。

只读分支不占用执行器互斥锁；真正改参数/发 PWM 的台架及恢复分支仍要求独占本机输出锁。UDP 连接在两种模式下都会拒绝共享已经占用的端口。

## 2. 仅在六路推进器动力已物理隔离后执行

必须由现场人员确认推进器/电调动力已隔离、飞控及通信仍供电。**不能根据飞控未解锁、1500 回读或配置布尔值推断动力已隔离。** 若没有完成隔离，停留在只读模式。先退出其他正在使用同一端口/输出的控制程序；工具不会自动终止它们。

确认后只需要一条显式命令，不进行循环人工确认：

```bash
python3 -m bluerov2.passthrough_bench --execute-power-isolated-bench
```

该标志是操作者对物理隔离的声明，软件无法自动验证。不要在推进器接通动力时运行这条命令。

执行顺序：

1. 读取并检查固件 3.5.4 / 05fed49b、已上锁 MANUAL、电气配置和实际 failsafe 参数，拒绝接管已被其他用途重分配的通道。
2. 检查新鲜的中立 RC 输入及六路驱动回读，保存六个 FUNCTION 的实际原值。
3. 将 SERVO1_FUNCTION 至 SERVO6_FUNCTION 依次设为 1，并逐项核对响应和再次读取的值。
4. 中立保持 0.2 秒，然后保持下面的完整六路图样 0.3 秒；每 0.04 秒重发同一帧，不执行任何轨迹反馈计算。
5. 完全停止发送 RC override，继续收取反馈，观察当前 `FS_PILOT_TIMEOUT` 对应的输入回中及 `Lost manual control` 事件。不修改该超时值，不靠主机持续回中代替这一阶段。
6. 结束或异常后显式发送中立，等待新的输入/输出中立样本及未解锁心跳，恢复实际备份参数并再次读取确认。

| RC 输入 / MAIN 输出 | 1 | 2 | 3 | 4 | 5 | 6 |
|---|---:|---:|---:|---:|---:|---:|
| 中立 μs | 1500 | 1500 | 1500 | 1500 | 1500 | 1500 |
| 图样 μs | 1540 | 1460 | 1550 | 1450 | 1560 | 1440 |

这是物理通道辨识图样，不是“前进”推力分配；没有对任何模型推力方向作验证或改动。

## 3. 现场电气测量

在无推进器动力的条件下，按板卡及测量设备的电气要求连接六路信号测量设备，记录完整波形。至少覆盖图样发送前、发送中、停止发送后超过实际 `FS_PILOT_TIMEOUT` 的一段时间。

重点测量：六路脉宽是否实际出现且对应正确，是否夹杂原电机循环的 1500 μs 脉冲，各通道的更新时间偏差，以及断流后是否持续回到中立。实际重复频率以本机配置和测量为准，不能把发送频率当成 PWM 电气频率。

**不要只看第一帧或一次 SERVO_OUTPUT_RAW。** 该版本的低频遥测可能一直显示请求值，而中间仍有其他 HAL 写入。没有波形原始记录，本工具始终保持 `electrical_output_verified=false`；软件回读也不能证明推进器真实停止或真实推力。

## 4. 状态与失败处理

| 状态 | 含义 |
|---|---|
| `read_only_completed` | 只读收集完成，没有启动输出事务。 |
| `bench_observations_complete` | 收到了完整图样、静默后的中立输入/输出样本及该固件的输入失联事件；仍不是电气或运动验收。 |
| `watchdog_neutral_not_observed` | 在有限观测窗口内没有确认输入与输出回中。不能据此宣称通过。 |
| `failsafe_event_unconfirmed` | 观测到回中，但未收到对应飞控失联事件；原因可能包括事件丢失或另一个写入者，保留未确认状态。 |
| `cleanup_unconfirmed` | 中立、未解锁心跳或参数恢复未全部确认。保持动力隔离，按备份恢复，不启动其他控制方式。 |
| `bench_aborted / rejected` | 检查、时间、用户中断或通信等异常；错误原因写入报告。 |

报告分别记录发送的 PWM、RC 输入、驱动采样回读、恢复参数及状态。`telemetry_cleanup_confirmed` 只代表该测试的遥测清理确认；`electrical_stop_confirmed` 不会因遥测确认而变成 true。发生故障后不会自动恢复图样或重启任务。

该版本没有本测试所需的固件会话独占、帧截止时间和故障锁存协议；本地锁不能替代这些机制。因此工具只适用于隔离动力的台架诊断，不能升级为实际动力控制器。

## 5. 进程崩溃后的参数恢复

FUNCTION 修改会持久化。SIGINT/SIGTERM 的清理不等于能处理 kill -9、主机断电或断链。备份在任何写入之前保存；不要在未确认恢复前重新接通动力。

仍保持动力隔离、连接同一台机器人，使用实际报告目录中的备份：

```bash
python3 -m bluerov2.passthrough_bench \
  --execute-power-isolated-bench \
  --restore-backup reports/本次目录/original_parameters.json
```

恢复分支不会发送非中立图样。它核对飞控标识、六个备份键及当前配置，仅恢复该工具拥有的 FUNCTION 改动。3.5.4 可能报告 UID=0，因此不能仅靠 UID 证明是同一台物理设备；现场人员仍须确认连接对象。已经恢复时不重复改写参数。

## 6. 可复现测试

```bash
cd /home/tj/blueROV2/code
python3 -m pytest -q tests/test_passthrough_bench.py tests/test_passthrough_review.py
python3 -m pytest -q tests
(cd /home/tj/blueROV2 && python3 -m pytest -q tests)
python3 tools/verify_passthrough_sitl.py \
  --binary reports/passthrough_review_20261103/sitl_artifacts/ardusub-stock \
  --bench-check \
  --output reports/passthrough_bench_revision_20261103/stock_sitl_bench_repeat.json
```

这里的二进制仅为本机 Linux SITL，不能刷入飞控。子测试只连接它自己启动的 localhost 实例；不会把真实机器的动力隔离状态写为“已确认”。旧 3.5.4 编译时排除了 SITL pilot-input failsafe，所以台架 CLI 在该模拟环境应返回 `watchdog_neutral_not_observed`、退出码 1，然后恢复六个参数。父联测要求正确识别这一负结果，**不是通过放宽阈值把固件看门狗判成通过**。

本轮实际执行结果及代码散列见 `reports/passthrough_bench_revision_20261103/REVIEW_zh.md` 和 `manifest.json`。此前源码/模拟器证据保留在 `reports/passthrough_review_20261103/`，与本轮真机只读结果分开。
