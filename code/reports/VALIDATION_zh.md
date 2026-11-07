> 本文为接口修订前的历史采集/构建报告，其中“本轮/当前”均指该历史轮次。当前代码与本次实际重测见 [接口修订报告](interface_revision_20261103/REVIEW_zh.md)。本次没有再次接触真机。

2026-11-03 真机连接与轨迹推力执行验证

本轮已实际执行只读采集、软件回归、原 MATLAB 数值源码比较、完整 ArduSub SITL 构建和主机接口联测。**总体验收未通过，当前不能启动真机 LBTS 轨迹推力执行。** 连接齐全解决了数据可达性，尚未解决实机独立输出、状态时序和模型对应问题。没有解锁或驱动真机，没有刷机、写实机参数或复位 DVL/任务原点。

结果与证据

| 项目 | 本轮结果 | 证据 |
|---|---|---|
| Python 项目回归 | 125 passed，2.96 s | `python_tests_current.txt` |
| 上级原基础功能回归 | 26 passed，0.20 s；单元测试，不是水中运动复测 | `legacy_tests_current.txt` |
| 官方 T200 离线映射、六路维度/顺序/正反向/电压/死区/越界 | 软件测试通过 | `../tests/test_force_output.py` |
| 原反馈计算、姿态包络、非等间隔保持、状态失效与停止失败 | 软件测试通过 | `../tests/test_force_output.py`、`test_force_state.py` |
| C++ 共享固件核心 | Wall/Wextra/Werror、ASan/UBSan 通过 | `firmware_validation.json`；测试命令见 README |
| 精确 ArduSub 3.5.4 完整 SITL 构建 | 通过，718 项构建任务 | `firmware_build.log`、`../firmware/ardusub_3_5_4/sitl_build_manifest.json` |
| 完整飞控进程与实际 Python 接口 SITL 联测 | 11/11 通过 | `direct6_sitl.json`、`direct6_sitl.status.jsonl`、`sitl_session_logs/` |
| DVL/飞控/GV7 当前数据可达性 | 通过只读接收；不等于坐标、时延合格 | `connected_validation.json`、`.raw.jsonl` |
| 真机最后状态 | ArduSub 3.5.4、已上锁 MANUAL、六路驱动回读均 1500 us，电压 15.960 V | `verification_live_final.json` |
| 真机独立输出接口 | **未部署：QUERY 无 BS6! 应答** | `verification_live_final.json` |
| 原 .m 数值源码对齐（GNU Octave 6.4.0） | **25/26 通过，1 项差异**；没有运行 MATLAB 本体和完整 OCP/搜索 | `matlab_alignment_current.json`、`.input.mat/.reference.mat` |
| 实机六路电气 PWM、物理方向、断流、短轨迹 | **未执行/未验收** | 受下述阻塞条件限制 |

SITL 实际验证内容

不是 mock 飞控：运行完整构建的 ArduSub，TCP 端口固定为本机 127.0.0.1:15760，进程由测试脚本自行启动并结束；不读取真机连接配置作为输出端点。

- QUERY 空闲/上锁/机架状态正确，获得会话并解锁后初始六路中立。
- 完整六路指定 PWM 到达 HAL 回读，通道值与完整帧相同。
- 注入旧 MANUAL_CONTROL、GUIDED 位置目标、模式变更、DO_SET_SERVO、电机测试命令，不能覆盖专用输出；这些注入只发生在 SITL。
- 第二来源不能抢占已有会话。
- 停止发送控制 FRAME，仅继续只读 QUERY，不调用主机 stop/finally，固件自动锁存 FAULT 和六路中立。此次中立观测在最后已接纳期限后 0 个 FC 毫秒；这是本轮观测值，不是板卡电气响应或所有时延的零延迟保证。
- 迟到 FRAME 不能恢复旧动力；STOP、上锁、RELEASE 均有应答。
- 原实际 Python `ThrusterOutput` 完成新会话、等待新鲜已解锁心跳、N→官方曲线→PWM→完整固件 HAL，并确认停止。
- 此轮 SITL 输入 `[1,-1,2,-2,1.5,-1.5] N`，显式测试电压 16 V，集中候选映射得到 `[1453,1544,1558,1437,1444,1444] us`；驱动回读匹配。该结果不证明物理推进器方向，SITL 测试电压与实机遥测不混用。

固件核心、完整补丁和主机接口均已验证，但**没有构建实机板卡镜像，也没有向真实飞控安装接口**。完整 SITL 使用仅针对主机构建脚本的兼容补丁；旧上游源码在 GCC 11 上的特定警告降级已记录，不把这个 Linux 可执行文件用于刷机。运行时改动仍只有 `direct6.patch`，未加入其他控制器。

传感器真实采集结果

连续采集时间：2026-11-03 17:24:15（北京时间）起 20 s。直接复用 `recording/src/` 的实际端点，所有 TCP 套接字只接收，MAVLink 采集不发送消息。

| 数据 | 实际采集 | 说明 |
|---|---|---|
| A50 velocity | 179 条，8.95 Hz，179 条速度/波束有效 | 接收最大间隔 0.387861 s |
| A50 position_local | 92 条，4.60 Hz，92 条状态有效 | 接收最大间隔 0.279590 s；姿态单位度 |
| 飞控 ATTITUDE | 200 条，10 Hz | 姿态 rad，rollspeed/pitchspeed/yawspeed 为机体系 rad/s |
| 飞控 SCALED_IMU2 | 200 条，10 Hz | 加速度 mG、角速度 mrad/s；此消息没有姿态 |
| 外置 GV7 原始 MIP | 2000 包，100 Hz，校验错误 0 | 包含磁场、delta-theta、delta-velocity；delta-theta 不能直接当 rad/s |
| GV7 融合欧拉角 | 199 条，9.95 Hz，有效标记为 1 | `0x82/0x05`；不是所有 0x82 包都包含姿态 |

实际 ROS 图没有传感器节点，仅有 `/rosout` 和 `/parameter_events`；`recording/install` 不存在。这不影响通过已有直接 TCP/MAVLink 链路验证传感器。未启动现有 DVL C++ 节点，因为其构造函数会发送 acoustic_enabled 配置；未启动可能配置 GV7 滤波器的驱动。

发现并处理：

1. `recording/.../mavlink_bridge.py` 原先将 mG 除以 1000 后直接发布为 ROS m/s²，少乘了 9.80665；现已修正，并用回归测试核验。其单位四元数只是占位，现按 ROS 约定设置 `orientation_covariance[0]=-1`，不允许下游把它当真实姿态。新执行器继续使用 A50 融合姿态和飞控机体系角速度，不切换定位/融合。
2. DVL `time` 约 108 ms，是上次速度报告间隔；采样时间必须使用 `time_of_validity`，不是这个字段。旧 feed 现额外保留采样/发送时间元数据，不改变原 goto 的位置、速度、输出或控制律。
3. DVL 发送时间相对主机时钟约差 76881687.708 s，不能直接相减作为传输年龄。DVL 内部采样至发送约 0.0946～0.1283 s；GV7 GPS 时间有效位为 0，也不能当作已同步 UTC。只读测试没有改任何设备时间。
4. 469 次消息事件上的接收时序检查，有 153 次分量时间差超过 0.15 s、5 次年龄超过 0.25 s，311 次仅接收时序符合条件。这个统计不是控制周期成功率，也没有计入所有未知传输延迟；结果已足以说明当前阈值不能保证连续执行。**没有放宽阈值、插值状态或改变控制时序来掩盖问题。**
5. 新状态适配器额外拒绝速度时间戳缺失、倒退/重放、内部采样延迟过长，以及 DVL 位姿/速度采样时间差超限。`age` 含已知速度处理时间，但仍是未计未知网络/位姿延迟的下界，日志注明限制。
6. GV7 安装矩阵目前只是 launch 文件注释，并未作为驱动参数生效。不能据此签署机体系已对齐；也没有把原始 delta-theta 或无效位置/速度字段用零补齐为有效状态。

原点和算法限制

- 保留原 `task_origin.json`，本轮未修改；它定义位置平移和航向目标，文件写入不执行设备航位推算 Reset。没有引入水面原点要求。
- 该文件没有保存 DVL Reset 标识，因此单次静态读取不能证明它与当前设备参考之间没有跨重启/复位变化。需在现有固定参考下核对方向/参考连续性，不能因为文件存在就盖上坐标验证标记。
- 原六推进器模型对 1/3 号 starboard 标签与 FRD 横向位置符号存在冲突，几何尺度也未实机确认。本轮不修改模型或重算增益；推力模型到实机方向仍为候选映射。
- Octave 重跑原 .m 数值核验仍为 25/26。相同 200 个 Z 样本下，Python 特征平方根与原 MATLAB Cholesky 生成不同有限样本，风险 0.090 对 0.085。最小修复保存在 `proposed_chance_alignment.patch`，**未应用，须获得算法行为修改确认**。原反馈代数、姿态包络、动力学/RK4/增益等已通过所列数值测试；不能把这写成完整 Python/MATLAB 规划器等价。

接下来真正缺少的验证

1. 确认真实飞控板卡型号，在对应工具链构建并由操作人员安装已审查的独立输出补丁。当前原固件不会响应接口，不能通过 Python 命令补齐；不使用任意电机通道 DO_SET_SERVO 代替。
2. 在适合的现场条件下，完成真实 MAIN1～6 电气输出、中立、范围、更新一致性、停发超时和停止确认测量；再做水中有限逐路测试，记录编号及实际作用力方向。
3. 按已有固定原点核对 DVL/姿态/角速度的轴、安装变换、杆臂和采样延迟；当前时序门限不满足的问题需先解决数据链路，任何拟议算法/约束/控制时序变更都另行确认。
4. 处理原模型几何问题和上述算法差异后，使用原算法导出的短计划做实测反馈执行，分别记录时序完成和跟踪误差。当前没有真实轨迹误差或推力反馈，旧 mock 图只代表软件模拟。

可重复命令

```bash
cd /home/tj/blueROV2/code
# 只读采集：不会解锁、写参数或驱动推进器
python3 tools/verify_connected.py --duration 20 --output reports/connected_validation.json
python3 tools/diagnose_direct6.py --output reports/verification_live_final.json
# 软件回归
python3 -m pytest -q
cd /home/tj/blueROV2 && python3 -m pytest -q tests
cd /home/tj/blueROV2/code
# 完整本机 SITL；脚本只接受二进制文件，输出固定到本机专用端口
python3 tools/verify_direct6_sitl.py --binary firmware/ardusub_3_5_4/sitl_artifacts/ardusub
```

详细执行、计划导出、逐路 PWM 和短轨迹命令仍见 `../README_force_zh.md`。所有真实配置核验标志仍为 false，不能为了启动而直接改成 true。
