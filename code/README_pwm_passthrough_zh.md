# PWM passthrough 本轮核实结果（2026-11-03）

后续已新增默认只读的无动力台架验证入口，操作和恢复命令见 [台架操作说明](README_passthrough_bench_zh.md)。本文件保留前轮证据；新增台架入口不代表直通运动后端已经验收。

本轮按你的要求实际检查了不使用 direct6 的 RC/PWM 直通路线，并新增可运行的离线源码测试和完整 SITL 复现程序。
**当前结论：可以让六个 RC 输入到达直通输出，但针对工程历史记录对应的原版 ArduSub 3.5.4，仅动态修改 SERVOx_FUNCTION 不能满足本任务的独立、持续且不被混控覆盖的输出要求。因此未把真机执行器切换成这条尚不合格的路线。**

没有连接真机、修改真机参数、解锁真机或刷写固件。没有改 LBTS、动力学、反馈律、增益、控制区间时序、状态适配器或原基础控制。没有安装 ROS2/MAVROS；使用现有 pymavlink 即可验证底层机制，引入 ROS 不会改变飞控内部输出路径。

本轮不使用 direct6 通信接口进行测试，也不部署 direct6。原有 force_run / thruster_output 文件保留原样，**它们仍是之前的独立输出实现，不是新建的 passthrough 真机入口**。请不要把本报告中的 SITL 命令当成真机驱动命令。

## 哪些事实已经核实

| 项目 | 本轮实际结果及证据边界 |
|---|---|
| 直通功能编号 | 精确 3.5.4 源码支持 FUNCTION=1（同序 RC 输入），或 MAIN1..6 分别指定 51..56（RCIN1..6）。两组都实际做了 SITL。 |
| 六路通信 | 一个 RC_CHANNELS_OVERRIDE 包携带 MAIN1..6 对应的六个 PWM；通道 7/8 填 65535 忽略，不使用 MANUAL_CONTROL 或 GUIDED 目标。 |
| 输入与低频输出回读 | 无 direct6 补丁的完整 SITL 中，两组配置均收到请求六路值；每组 27 个 SERVO_OUTPUT_RAW 样本都显示请求值。**这本身不足以证明两次采样之间没有其他写入。** |
| 原电机循环是否退出 | 不退出。原始 AP_Motors6DOF::output_to_motors 仍对 motor_enabled 通道调用 rc_write；后者使用初始化时的缓存映射直接写 HAL，不因运行中改变 FUNCTION 而停止。原始函数编译测试已复现覆盖。 |
| 更高频的实际提交 | 仅加 HAL 日志、不改变输出控制逻辑的 SITL 中，每组约 0.58 秒窗口记录到 233 次电机中立 push，以及 30 次完整直通值写入，二者交替出现。另有逐通道更新形成的中间向量。**这不是保证独立输出的通道。** |
| 上锁是否足以阻止直通 | 原直通函数没有 armed 检查；SITL 在上锁状态仍回读到非中立 PWM。实际板卡 PX4IO、安全开关及电气输出尚未测量，不能把 SITL 回读直接称为实机桨已转。引用的开源项目 README 也明确提示直通绕过 arming checks。 |
| 断流保护 | 原版实机分支的 pilot input failsafe 会调用 set_neutral_controls；原始函数测试验证这一点。不能说原版完全没有回中机制。但是它不是 direct6 的会话/有效期/故障锁存语义；收到新的或迟到的有效 override 可清除此输入故障。 |
| SITL 断流边界 | 3.5.4 用编译条件排除了 SITL 的 pilot input failsafe。本轮超过配置的 0.2 秒不发控制包后仍保留输入，**不能据此推断真实板卡相同，也不能用这个 SITL 验收实机断流保护。** |
| 独占输出 | MAVLink override 检查来源 SYSID_MYGCS，但不绑定 component/session。同一 sysid 的另一 component 已在 SITL 成功覆盖六路输入。本地锁不足以证明 QGC/另一主机不会争用。 |
| 参数恢复 | 两组 SITL 均先读取原始 SERVO1..6_FUNCTION，退出前发中立并恢复实际备份值 33..38，再显式读回确认。真机没有参数写入。 |
| 参数是否“临时” | 原版 GCS_MAVLINK::handle_param_set 会执行 vp->save(force_save)。所以改 FUNCTION 是持久化参数写入，主机 finally 恢复不是固件自动临时配置；崩溃、断电和断链不能保证恢复。 |

不要把“包发送成功”“某次 PWM 回读正确”“SITL 检查程序完成”解释为“本路线已经满足真机输出验收”。Direct6 不是所有固件和硬件上的唯一方法，但本次提出的参数切换方式不能直接替换本项目 3.5.4 的执行端逻辑。

## 新增文件

- tools/verify_passthrough_source.py：提取精确原版函数，使用记录 HAL 编译运行。分别覆盖实机条件分支和 SITL 条件分支；这是源码函数测试，不是完整板卡测试。
- tools/verify_passthrough_sitl.py：只启动自己的 localhost SITL，测试 FUNCTION=1 / 51..56、完整六路包、其他 component 写入、输入保持和原参数恢复；没有真机 endpoint 参数、解锁命令或 direct6 消息。旧 SITL 内部固定端口不能并行占用，入口使用进程锁串行运行本工具。
- tests/test_passthrough_review.py：15 项离线协议/边界/消息来源/参数新回读测试。
- reports/passthrough_review_20261103/：原始报告、源码函数测试、HAL 原始 CSV 与摘要、编译日志、开源项目固定提交快照、二进制及散列。没有删除既有实现或把未验证标记改为 true。

## 本轮实际执行的命令

在已有 Python 环境中运行，依赖现有 pytest、pymavlink；源码函数测试还需要系统 g++。不要求 DVL、原点、计划文件或 T200 曲线。

    cd /home/tj/blueROV2/code
    python3 -m pytest -q tests
    # 185 passed；其中新增直通诊断测试 15 项。

    cd /home/tj/blueROV2
    python3 -m pytest -q tests
    # 原基础功能 26 passed。

    cd /home/tj/blueROV2/code
    python3 tools/verify_passthrough_source.py --source /tmp/ardusub-stock354-passthrough --output reports/passthrough_review_20261103/source_probe.json
    # 两个编译条件分支均通过复现断言；断言验证的是限制确实存在，不是输出链路合格。

    python3 tools/verify_passthrough_sitl.py --binary reports/passthrough_review_20261103/sitl_artifacts/ardusub-stock --output reports/passthrough_review_20261103/stock_sitl.json
    # 原版完整 SITL；必须与其他旧版 SITL 串行，不能同时占用 UDP 5501。

    PASSTHROUGH_TRACE_FILE=/home/tj/blueROV2/code/reports/passthrough_review_20261103/hal_trace.csv python3 tools/verify_passthrough_sitl.py --binary reports/passthrough_review_20261103/sitl_artifacts/ardusub-observed --output reports/passthrough_review_20261103/observed_sitl.json
    # 增加 HAL 观测日志的完整 SITL；与上一条串行执行。

两个二进制都是 Linux x86_64 SITL 程序，**不能刷入飞控**。原版二进制与观测版分开保存，观测版补丁为 sitl_observation_only.patch，只记录提交、不拦截或改写 PWM。
精确原版源码由既有构建工作区的隔离副本撤销 direct6 补丁得到；参与该补丁的五个原始文件均与已有官方源码 manifest 散列一致。仍使用已有仅限主机构建的 Python/GCC 兼容处理，详见 stock_source.json、build_manifest.json 及编译日志。没有更改实机固件。

若 /tmp 中源码被清理，按既有 firmware/ardusub_3_5_4/sitl_build_manifest.json 的固定官方归档/子模块提交重建源码，并应用 sitl_host_compat.patch，**不应用 direct6.patch**。构建参数为：

    CXXFLAGS='-Wno-error=array-bounds -Wno-error=unused-but-set-variable -Wno-error=maybe-uninitialized -Wno-ignored-qualifiers -Wno-address-of-packed-member -Wno-class-memaccess' python3 modules/waf/waf-light configure --board sitl --no-submodule-update --disable-tests --no-autoconfig
    python3 modules/waf/waf-light sub -j4

本轮曾尝试同时运行两套旧 SITL，其中原版实例因 UDP 5501 被占用退出；失败报告保存在 stock_sitl_failed_parallel.*。随后加入诊断入口资源锁，串行复跑，不将该失败或修复隐藏为“第一次全通过”。

## 真机下一步与尚未完成事项

1. **本轮没有重新读取真机版本。** 目标 3.5.4 来自工程历史采集报告；所有新源码和 SITL 结论都固定到 commit 05fed49b8ee1db3c6bca924e0b334b2e0ece980d。如果实机已经升级，应针对实际版本重新核查，不将此结论外推到所有 ArduSub。
2. 对该版本，尚未找到只改这六个 FUNCTION 就能阻止旧电机写入的可验证配置。因此没有交付声称“已可真机执行”的 passthrough 后端，也没有把原 direct6 执行器偷偷用于这轮测试。
3. 坚持不加飞控输出补丁时，下一项是核实某个原版固件/受支持配置是否实际取消原电机输出写入，并验证它的输入超时、上锁输出、通道一致性和排他条件；不能靠换 MAVROS 包消除底层问题。本轮没有自动升级固件或更改硬件配置。
4. 当前 PWM 电气输出、板卡断流回中、安装通道/方向以及带真实 12 维状态的闭环轨迹均未由本轮验收。前述软件测试通过不代表真机轨迹通过。此前 MATLAB/Python 机会约束平方根差异和物理几何待核实项保持不变。

源码和外部资料的固定来源、文件散列见 reports/passthrough_review_20261103/build_manifest.json 与 upstream_reference.json。
外部项目快照为 Robotic-Decision-Making-Lab/mavros_thruster_hardware，提交 5a6937dc8ff020b755e4b8ff22a913528db315d7；其 README 和实现作为参考保存，并附原 LICENSE，未将其 ROS 控制框架移入本工程。
