# 2026-11-05 验证收尾

本轮实际运行：完整工程 **249 passed，无排除项**；旧工程 **26 passed**；先前排除的 UDP 用例单独 **1 passed**；当前 Python + 完整 ArduSub SITL **20/20 检查通过**。真机只读连接未收到心跳，因此实际板卡型号、系统/组件 ID、当前固件和 direct6 能力本轮未能确认。没有向真机发任何消息，没有刷机、写参数、解锁或输出；模拟器中的解锁/故障注入仅发生在 localhost。

## 修改及保持不变的内容

| 文件 | 本轮修改原因 |
|---|---|
| `bluerov2/force_run.py` | 修复 thrust-test 未校验电压年龄配置；执行不再先按离线固定电压作能力拒绝，而在连接后按实际电压来源检查；保留实际发送映射到 report.output；运行报告/会话日志保存一次通道映射；绘图物理 PWM 顺序不变，标签及颜色对应模型索引/方向 |
| `tools/verify_direct6_sitl.py` | 扩展原框架的必要用例，独立 instance/base-port、端口预检、记录 T200/测试脚本摘要及进程退出状态；修复扩展后提前关闭同一控制器 logger 的测试脚本问题 |
| `tools/diagnose_direct6.py` | 复用仅通信配置，取消无关 DVL 采集；不发送 GCS 心跳；序列化前白名单只允许版本读取、参数读取及无会话 QUERY；占用 UDP 端口时拒绝共享；保存发送清单并有界退出 |
| `tests/test_force_output.py`、`tests/test_force_entry.py` | 增加受控时钟区间截止/正常衔接/停止幂等、电压来源/过期/保持、预览与执行、非恒等通道映射绘图检查 |
| `tests/test_direct6_diagnosis.py`（新增） | 白名单允许只读请求，拒绝接管/FRAME/STOP/RELEASE、解锁、RC override、参数写入、心跳和错误目标 |
| `README_force_zh.md` | 当前记录链接、停止语义、日志通道及电压来源说明、SITL/只读准确命令 |

本轮没有修改规划、动力学、反馈增益或时序。`execution.py`、`thruster_output.py`、`t200.py`、`state_adapter.py`、`model.py/dynamics.py/lbts.py/kprm.py`、旧控制入口、实际配置及飞控补丁均与 baseline 摘要一致。没有另加看门狗、协议、控制模式或第二套映射。代码差异见 `changes.patch`；完整前后摘要、命令/退出状态见 `manifest.json`。

## 停止与保持：最终源码及本轮证据

| 情形 | 最终实现及本轮验证 |
|---|---|
| 正常区间衔接 | `execution.py` 每段只计算一次原反馈，设置绝对 `clock+time[k+1]` 截止时间，再发送新推力。`ThrusterOutput.poll()` 只在截止前重发保持 PWM；边界不自动发送中立帧。原生 C++ 联测与完整 SITL 两段不等长计划均通过 |
| 整个任务结束 | 原执行器 `zero()` 主动发送零力，然后 `stop()` 执行 STOP → 中立 HAL 确认 → 上锁新心跳 → RELEASE 确认。PWM/thrust-test 通过入口 finally 调 stop。正常结束、异常、SIGTERM、停止确认失败均有现有/新增测试 |
| 状态失效/异常 | 计划持有 poll 先验证状态再刷新；失败进入 finally。状态恢复不会解除任务故障锁存。当前停止调用为幂等；重复 stop 返回同一结果，不再发送 |
| 主机崩溃/通信断开/控制命令断流 | 固件 `Core::tick()` 在原快速输出循环按飞控毫秒时间与帧的绝对 deadline 比较，到期 FAULT/六路1500。不需要主机 finally |
| 心跳/QUERY 继续但没有 FRAME | 它们不修改租约 deadline。SITL 在持续心跳和 QUERY 下实际观察到 FAULT 中立 |
| 通信 poll 继续但无下一段算法输出 | 区间截止后旧 PWM 不再续租。持续调用实际 Python poll、保持心跳/QUERY，固件仍到期回中；本轮不是用主机主动 STOP 模拟这一结果 |
| 故障后迟到帧 | 固件 FAULT 不接受 FRAME 恢复，原生测试及完整 SITL 均通过；随后主动清理确认成功 |

非等间隔计划为 `[0,0.12,0.35] s`，本轮主机实际采样时间为约 `[0.000018,0.121866,0.350121] s`，未修改任何门限。核对全部发送控制帧和已采样 HAL 回读，段间未发现额外中立；这不能证明实际板卡引脚电气边沿没有瞬态。

**延迟边界：**主动停止在正常结束或主机检测异常后执行，包含主机调度/末次状态读取、发送队列、网络及飞控处理，随后还要等待停止、上锁和释放应答；没有应答不能报成功。看门狗由最后接纳控制帧的 FC 绝对期限决定，200 ms 是从接纳时刻算的最大允许剩余租约（使用旧 FC 时间通常更短），再加快速循环执行及实际输出驱动延迟。区间截止只禁止续租，上一段最后已接纳帧可保持至该期限；不承诺“每段末立即中立”。CPU 主循环卡死和物理 ESC 动态不属于本次租约时序测量。

本轮普通断流首次中立观察在最终 FC 期限后 **0 ms**；算法缺下一段用例为 **15 ms**。这两个数字使用飞控 `fc_ms` 与 `remaining_ms`，包含查询观察离散性，不是物理响应时间，也不是对全部运行的最坏延迟保证。主机区间/发送时刻使用 monotonic 时间。真实板卡六路电气时序仍待测。

## 日志通道及电压

- `requested_force_n/adopted_force_n/static_estimated_force_n/hardware_saturated` 以及反馈力均是模型索引 0..5；`pwm_us`、发送帧与 HAL 回读是物理 MAIN1..6。每个推力运行记录一次 mapping，不将两种顺序按列号硬配；新增反序通道图测试验证了索引及图例。
- 离线 preview 仅在明确配置 `test_voltage_v` 时作曲线/能力检查；没有指定则不假装完成电压相关检查。执行时新鲜 SYS_STATUS 优先，其次 BATTERY_STATUS；缺有效遥测且 test_voltage_v=null 时非零输出失败。
- 现有显式固定电压实验仍允许；必须非 null 配置，来源记为 `configured_test_voltage`。本轮 SITL thrust-test 使用 **16 V configured_test_voltage**，并不是真机电压测量；实际发送 `[1453,1544,1558,1437,1444,1444] μs` 与同一 T200 映射/采样 HAL 相符。正式遥测实验配置应保持 null。
- thrust-test 的 `plan.preview` 为解锁前检查，`output` 和 `force_mapping` 日志为实际发送时的映射、来源、序号。刷新不重新读取电压或重算 PWM，受控测试已验证。

## 本轮实际命令及结果

工作目录 `/home/tj/blueROV2/code`：

```bash
python3 -m pytest -vv --tb=long tests/test_passthrough_bench.py::test_existing_udp_listener_is_not_shared_or_taken_over
python3 -m pytest -q tests
python3 tools/verify_direct6_sitl.py --binary firmware/ardusub_3_5_4/sitl_artifacts/ardusub --base-port 15760 --instance 41 --output reports/closeout_20261105/direct6_sitl_attempt2.json
python3 tools/diagnose_direct6.py --output reports/closeout_20261105/live_readonly.json
```

旧工程工作目录 `/home/tj/blueROV2`：`python3 -m pytest -q tests`。

| 证据 | 结果/退出码 |
|---|---|
| `udp_test.txt` | 单独 1 passed / 0；原用例用 bind(127.0.0.1,0) 随机独立端口，不必修改测试 |
| `targeted_attempt1.txt` | 相关执行/入口/端口回归 110 passed / 0 |
| `diagnosis_guard_tests.txt` | 只读出站约束 6 passed / 0 |
| `project_tests.txt` | 最终全量 249 passed，未排除 / 0 |
| `legacy_tests.txt` | 原基础功能 26 passed / 0；不是水中动作重测 |
| `direct6_sitl_attempt1.json/.txt/.console.log` | 本轮首次 SITL 已能启动，扩展用例因复用已关闭 logger 失败 / 1；属于本轮测试脚本引入问题，保留堆栈 |
| `direct6_sitl_attempt2.json/.txt/.console.log/.status.jsonl` | 修复 logger 生命周期后 20/20 / 0；正常/中断 PWM CLI、thrust CLI 的子报告及独立会话日志由父报告定位 |
| `live_readonly.json/.txt` | 成功创建 UDP 监听，5秒未等到飞控心跳 / 1；发送列表为空 |

上轮 `/tmp/direct6-sitl-current.console.log` 的真实错误是 RC UDP5501 `Operation not permitted`，不是 Address already in use。本轮随机 UDP 用例和完整模拟器端口均可用。未通过停止用户进程来腾端口；测试脚本先检查 TCP15760 和独立实例 RC5911。模拟器由本次进程启动并结束（SIGTERM，子进程 -15 是主动清理），父验证脚本退出0；`cleanup.json` 记录收尾检查。

## 当前代码和固件对应关系

- 上游 ArduSub 3.5.4：`05fed49b8ee1db3c6bca924e0b334b2e0ece980d`。
- 当前补丁 SHA256：`6535da7bb7398d7e50f8992797cde0cf2100e15cd1408d49a7d9c11ab0f66fd1`。
- 本轮运行二进制 SHA256：`9ead073779cc43961c76dd1b28a3425265a7eb72850623365b632fa82b19153f`，同时匹配构建目录和固定构建清单。
- 本轮重新校验官方归档摘要和 source_manifest 原文件，临时展开后应用当前 direct6.patch；ArduSub/libraries 的 **1148 个运行时源文件全部匹配原构建目录**。共享 direct6_core.h 与补丁内文件相同，证据见 `firmware_identity_initial.json`、`firmware_identity.json`。因此无须重新构建同一 SITL；没有把另一个二进制的历史通过当作本轮结果。
- 源码归档不含主机兼容补丁需要的子模块，首次在临时目录尝试应用 host compatibility patch 失败；该补丁只改构建脚本，不属于运行时比较。本轮运行时对照只应用 direct6.patch，既有完整构建清单保留子模块/工具链依据。
- 最终 SITL 报告固定当前 Python（含 t200.py）、测试脚本和补丁摘要，manifest 再核对它们与最终代码一致。Linux SITL 可执行文件不能刷入飞控。

## 只读实机结果及下一步条件

本轮实际配置是 `udpin:0.0.0.0:14550`（监听 Ubuntu 本机；它本身不是飞控 IP），主机源身份255/191，目标由合法心跳识别。监听未收到飞控心跳，不能确定本轮对端IP、实际 system/component、固件标识、board_version 或专用能力应答。`network_readonly.json` 显示有线 enp0s31f6 为 DOWN，无192.168.2网段配置，USB列表无可识别的飞控设备。既有 Ubuntu→Companion→飞控路径本轮未打通；没有修改网卡、设备配置或启动 DVL。

历史记录曾为目标1/1、ArduSub3.5.4/05fed49b、board_version=0、未观察到BS6应答；这些不是本轮读数。board_version=0 不能选具体板卡构建目标，单独无BS6应答也不能证明没安装补丁，还需排除目标与DATA64路由。当前无设备，因此本轮能力查询甚至没有发送，结论只能是未读取。

**结论：本轮软件收尾完成，尚不具备选择实际板卡目标并准备刷写部署的完整条件。**已具备当前主机+固定补丁的SITL证据；最少还需恢复现有通信链路，并取得飞控实物厂商/准确型号与硬件修订（标签或已有硬件资料），以及可恢复的现固件/参数备份，再匹配3.5.4板卡工具链和构建目标。本轮不猜目标、不构建或刷写实际板卡。后续实机电气输出、六路方向、板卡看门狗与水中轨迹均未验证；规划机会约束/状态链路历史遗留项未在本轮扩大处理。
