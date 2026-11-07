# 2026-11-03 轨迹推力接口修订与验证

本轮直接修改现有工程，完成 PWM/轨迹依赖分离、完整合格状态缓存及必要轮询优化。**软件修订完成；项目测试 170/170、旧基础控制 26/26、同一 ArduSub 3.5.4 完整 SITL 15/15 通过。原 .m 数值对齐仍为 25/26，实机独立输出及真实轨迹未验收。**

本轮没有连接真实机器人，没有解锁/驱动真机、写飞控参数、复位定位或刷固件。SITL 中的解锁、故障注入和输出仅针对脚本自行启动的本机进程。本报告中的实时采集数据均来自已有历史文件；不把历史测量当成本轮测量。工作目录无 Git 元数据，使用 `baseline.json`、`before/*.before` 和 `final_manifest.json` 保存修改前后依据。

## 1. 问题分类与实际调用链

| 分类 | 审查证据 | 本轮处理 |
|---|---|---|
| 确定缺陷 | 包初始化、force_run 顶层及输出层提前导入规划/曲线；run 在分支前读取旧完整配置、原点及 T200 | 已提前分支；新增仅通信加载器，包导入延迟执行 |
| 确定缺陷 | 原 StateAdapter 把首次合格状态以后的任何异常锁存；缺少完整合格快照复用 | 改为等待/完整缓存/明确故障，按异常类型区分，不匹配异常字符串 |
| 确定重复工作 | 每次状态轮询检查旋转并解线性方程、读取/哈希原点文件；同一持有轮询两次服务 MAVLink | 初始化预计算；仅新源消息解算；参考文件默认 1 Hz；持有轮询服务一次通信 |
| 条件性问题 | 连续繁忙 DVL 的收包虽有预算，但逐行 JSON 解析无界 | 新推力模式启用接收及解析预算，超载终止；默认旧调用不变 |
| 条件性问题 | 低频源与高频读取容易被一律解释为状态丢失 | 缓存消除此误判；真实过期仍停止，不保证现有链路能连续执行 |
| 未验证 | 注释中的 starboard 与 FRD 数值 y 符号矛盾；实物编号/位置/符号对应未测 | 提供静态数值表，未翻转坐标/符号，不能据注释认定哪一个数值错误 |
| 已复现的算法差异 | 原 .m 正定协方差用 Cholesky，Python 用特征平方根 | 同输入和固定 Z 继续定位，未修改概率/规划逻辑 |
| 未验证的实机行为 | 历史真机报告未出现 direct6 能力应答，真实输出/物理停止未测 | 保留事实和后续步骤，本轮未重新连接真机 |

### 分支依赖（修改后的实际代码）

| 分支 | 模块、配置及设备 |
|---|---|
| PWM 离线 | numpy、标准库、thruster_output/communication 的无设备导入；只读取配置路径、输出租约/刷新条件、六路 PWM 和时长；**不读取连接文件、不导入旧 backend、不打开设备** |
| PWM --execute | 以上 + 旧 rov_control/pymavlink/SessionLogger、连接字段、目标心跳、实时版本/必要电气参数、direct6 QUERY/ACQUIRE/FRAME/STOP/RELEASE；mapper=None；**无 DVL/原点/旋转/规划/动力学/推力符号/曲线/电压要求** |
| plan 离线 | execution、host_tracking/旧配置读取、io、state_adapter、T200；读取固定原点/旋转、曲线和可信计划，核查模型签名、单位/参考、维度/时间、原执行器条件；**不连接设备** |
| plan --execute | 以上 + 真实状态连接、安装/模型/通道/方向记录、新鲜电压、起点一致性、独立输出；启动无完整状态则等待最多 3 s，超时不取得输出会话 |

纯 PWM 六元素顺序为物理 MAIN1～6；推力向量顺序为模型输入。`mapping` 只用于推力模式。原 `rov_control` 通信、目标识别、心跳及 logger 继续复用；旧 host PD、GUIDED、MANUAL_CONTROL 不参与新执行。

### 故障在哪里产生、如何停止

- `StatePending` 表示尚未得到新的完整合格候选：缺消息/采样时间元数据、暂时质量不足、分量不同步或候选过旧。有合格且新鲜的缓存则返回缓存；从未合格则 `status=waiting,valid=False`，不补零。
- `StateFault` 表示缓存过期、心跳/连接失效、设备/接收时间回退、重启、位置/姿态突变、原点文件变化。其他无法识别的解析/程序错误保留异常类型及原因为终止故障。
- 全部候选检查成功后才原子替换缓存。内部按源预解析结果不等于对外接受候选；不同步/不合格的新旧分量不会形成新的有效状态。
- 缓存保存完整 12 维状态、原接收时间、设备采样/发送时间、质量、固定参考标识和形成时间。复用时不改这些时间；每次按当前 monotonic 时间计算年龄。重复设备时间戳即使重新收到，也不能获得新的年龄。年龄 `>=0.25 s` 时停止，0.15 s 时间差等原门限没有增大。
- `StateAdapter.fault` 锁存终止故障。控制边界由原执行器检查；持有区间的 poll 先检查状态，失效即抛出并进入 finally，**不会继续续租旧推力**。原执行器的 zero/stop 以及入口 finally 都进入同一个幂等停止路径。
- 固件超时另行锁存 FAULT、回中；主机 STOP 必须确认六路 HAL 中立、新鲜已上锁心跳和 RELEASE。没有确认不会报告成功。数据恢复或迟到 FRAME 不能自动恢复本任务输出。

### 频率、时序与优化边界

| 操作 | 实际频率/条件 |
|---|---|
| 原始状态发布 | 历史日志 DVL velocity 8.95 Hz、pose 4.60 Hz，FC ATTITUDE 10 Hz |
| 主机收取/读取状态 | 持有期间按现有 wait_poll_period=0.005 s 轮询；启动与控制边界也读取 |
| 状态数值解算 | 对应源记录更新才解算；没有新消息只做年龄、心跳、故障和到期参考检测等必要工作 |
| 原反馈计算 | 只在原 time[k] 区间开始一次；非等间隔、姿态包络、K、限幅、规划索引和保持语义不变 |
| PWM 租约刷新 | refresh_period_s=0.04 s，重发同一完整 PWM，不重算反馈或推力映射 |
| 原点文件检查 | 启动固定摘要，默认 1 s 检查变化；可选 state.reference_check_period_s，设备回退/跳变检测不依赖文件 |

固定安装/世界旋转只验证一次，feed 逆旋转用预先保存的转置，取消每轮 solve。旋转计算、源数据解析只在新消息时进行。受控测试验证 200 Hz 读取、10 Hz 源可完成 10 次更新及 190 次缓存复用，同一源记录不反复做旋转/读文件。没有声称获得未经基准测量的 CPU 加速倍数。

MAVLink 继续用旧有限消息/时间预算。新推力模式的 DVL 每轮最多 8 次 recv、65536 字节、接收预算 5 ms；最多 16 条 JSON、每行 16384 字符、缓冲 1 MiB，超载即报错停止，不把未处理的整批旧消息在后续轮询重新标成新收到。单条不完整行的传输/缓冲延迟仍不是已测采样时间，年龄仍是下界。旧 DvlFeed.poll 不传可选参数时行为不变；旧 host_tracking 默认也不启用新增解析限制。未引入异步框架、线程或第二套融合。

### 输出独占及 verified 的含义

主机 flock 仅协调使用同一锁的新执行器，旧程序不一定遵守。真正的执行端独占由 direct6 的来源/会话、序号及租约保证；ACQUIRE 需要已上锁、接口 ready 且最近 1 s 无旧输入，不能自动接管活跃控制器。会话期间旧控制/混控不能覆盖输出，SITL 已注入旧消息验证。操作前仍须停止旧 goto/trajectory/操纵杆输出。

保留 ArduSub 3.5.4 版本、FRAME_CONFIG、MOT_PWM_TYPE、SERVO1～6 功能/范围/中立/反向、SYSID_MYGCS 等检查。PWM 不再把 `firmware_verified` 当作代替自动能力检测的布尔门槛，而通过实时 QUERY 核实接口；轨迹分支保留它作为安装/物理验证记录。通道/推力方向、模型对应、状态安装记录仍需实测核验；没有将任何 false 改为 true。

## 2. 本轮实际测试

| 命令/证据 | 本轮实际结果 |
|---|---|
| 修改前 python3 -m pytest -q tests | 125 passed，2.97 s；`baseline_tests.txt` |
| 修改后 python3 -m pytest -q tests | **170 passed，3.75 s**；`project_tests.txt` |
| 在 /home/tj/blueROV2 执行 python3 -m pytest -q tests | **26 passed，0.20 s**；`legacy_tests.txt` |
| 现有 tools/verify_direct6_sitl.py | **15/15**；`direct6_sitl.json`、控制台/状态/会话 JSONL |
| 当前原 .m/Octave 数值对齐 | **25/26**，退出码 2，保留真实失败；`matlab_alignment.json` 及固定输入/参考 MAT |
| 现有 raw.jsonl 按当前适配器 5 ms 轮询回放 | 正确缓存并在过期时锁存停止；`state_replay.json`；不是连续轨迹验收 |
| 默认纯 PWM 离线命令 | checked、actuation_requested=false；`pwm_offline.json` |

测试覆盖：无初始状态等待/超时且不启动；读取快于发布的缓存复用；临时缺包/不同步/质量不足；时间与年龄不重置；到期停止；新完整状态原子替换；不合格候选不污染；非有限/未知异常、设备回退/参考变化/断开锁存；恢复不重新输出；无 DVL/原点/模型/曲线的 PWM 子进程及模拟执行；轨迹保留原点/模型/安装检查；非等间隔一次反馈及保持；正常/异常/中断停止和未确认不成功。既有曲线、反馈、C++ 核心、维度、通道及基础功能测试均保留。

没有删除断言或放宽阈值。旧状态测试的 mock 仅提供 current()、部分期待“任何候选不完整即锁存”，最初发生 16 个不匹配；更新为真实 poll/质量接口及本任务明确要求的缓存语义，并增加“缓存到期仍必须锁存”等断言后通过。临时备份的 __init__.py 曾干扰无目标 pytest 收集，备份改用 .before 后消除。SITL 新增日志断言首次误用 logger 字段，之后重跑又发现跨轮旧日志干扰 SIGTERM；改用真实 record_kind 和每轮唯一目录，仅当前进程证据可满足断言。失败文件 `direct6_sitl_first_attempt.json`、`direct6_sitl_stale_log_attempt.json` 保留；最终 15/15 的证据是 `direct6_sitl.json`。

### 本轮 SITL 与代码/固件版本

继续使用已有测试框架和完整已编译的 ArduSub，不是 mock 后端。固件基准 commit `05fed49b8ee1db3c6bca924e0b334b2e0ece980d`、direct6.patch 及二进制未改。二进制 SHA256：`9ead073779cc43961c76dd1b28a3425265a7eb72850623365b632fa82b19153f`。已有构建清单见 `firmware/ardusub_3_5_4/sitl_build_manifest.json`，本轮报告还记录主机源码及补丁摘要。未重新构建板卡镜像。

在原 11 项基础上，增加无定位/模型/映射/曲线配置的完整真实 PWM CLI、六路非中立 HAL 回读、正常停止确认及 SIGTERM 中断后回中/上锁/释放。其余包括第二来源拒绝、旧 MANUAL/GUIDED/servo/motor-test 不覆盖、停止刷新但不调用主机 finally 时由固件自行回中、故障后迟到命令不能恢复。本轮中立首次观测为最后期限后 0 个 FC 毫秒，仅为 SITL 状态观测精度，不能写成板卡零延迟。HAL readback 与发送 PWM 分开记录，均不是实测物理推力。

## 3. 尚未解决的证据与影响

### 状态时间：历史记录 + 本轮回放

输入 `reports/connected_validation.raw.jsonl` 为历史 20 s 被动采集；本轮仅从硬盘读取。DVL 位姿接收间隔 median 0.217927 s、p95 0.233406 s、max 0.279590 s；速度 median 0.105448 s、max 0.387861 s；FC 角速度 max 0.155865 s。已知速度采样到发送约 0.094578～0.128258 s。设备与主机时间未同步，未知传输/位姿延迟、跨设备采样偏差没有凭空补齐。

以首个目标心跳为回放起点，缓存最初年龄 0.151220 s；2 次新的完整候选、18 次缓存返回后，于 **0.100 s** 因原速度数据年龄达到 0.25 s 而停止。后面 3956 次读取保持故障，不自动恢复。整个回放区间的“最新候选仅时间门限”检查为 3334 次合格、98 次年龄超限、544 次接收时间差超限，候选年龄最大 0.490724 s；这些是诊断统计，不是独立控制任务成功率。

结论：缓存修复了“读取频率高就误报缺包”的软件语义，不能让已有真丢包/时延变成新鲜状态。没有增大 0.25/0.15 s 门限，没有插值/外推或状态预测。需另行改善/测量当前数据链路连续性；若最终需要改变控制周期或算法约束，先提出证据和影响再确认。GV7 未参与该控制状态计算本身不是缺陷；本轮继续使用已有 A50 融合姿态和 FC body gyro，不另起 EKF。

### 几何：静态判断，不替实物作结论

`geometry_review.json` 列出全部六路位置、正方向、分配列、原厂号、MAIN 通道和候选曲线符号。模型索引 0/2 的位置分别为 (0.156,-0.111,0.085) / (-0.156,-0.111,0.085) m，注释分别叫 front/rear-starboard，FRD 声明 y 向右。此矛盾只能证明描述/对应不一致，无法单凭源码确定是标签、编号、位置值或符号应改。原模型的估计尺度及物理正力方向均未现场验收。模型、几何、分配矩阵及候选符号全部原样保留。

### MATLAB/Python：继续定位但未改算法

用同一个固定 200 列 Z、相同正定位置协方差和障碍输入重跑原 .m（GNU Octave 6.4.0）。Python 特征平方根得到 18/200=0.090；原 .m Cholesky 得 17/200=0.085。9 列样本的碰撞掩码不同；两种平方根重建协方差误差分别仅约 5.55e-17 / 3.47e-18。诊断代码仅把 Cholesky 代入固定输入就得到 0.085，与原 .m 一致，具体矩阵及列号见 `matlab_alignment.json → chance_example.fixed_sample_diagnosis`。

最小建议仍是“正定用原 .m 的 Cholesky，失败才回退特征平方根”；此改动会改变固定样本点、有限样本风险及可能的边接纳/搜索结果，属于算法行为变化，**未实施**。现有候选补丁 `reports/proposed_chance_alignment.patch` 仅供之后确认，没有改变概率阈值来通过测试。动力学/RK4、原增益/协方差传播、姿态包络及六路反馈等本轮所列数值用例通过；没有运行 MATLAB 本体、其 CasADi/IPOPT 全 OCP 或完整规划搜索，不声称全算法等价。

## 4. 修改文件与不变项

| 文件 | 原因 |
|---|---|
| bluerov2/__init__.py | 延迟加载公开算法 API，避免 PWM 导入规划；处理同名 lbts 函数的导入兼容性 |
| bluerov2/communication.py（新增） | 只复用旧通信/logger 和连接配置 |
| bluerov2/force_run.py | 提前按模式加载，PWM 无关依赖移除；启动只重试等待态；持有先检查状态再续租 |
| bluerov2/state_adapter.py | 完整缓存、时间/质量/参考保存、候选原子提交、明确故障类型、静态预计算 |
| bluerov2/thruster_output.py | 移除 T200 导入；PWM-only 可 mapper=None；避免持有轮询重复 service_io；自动能力检查 |
| bluerov2/host_tracking.py | 可选 bounded_parse，仅新推力调用启用；旧入口默认不变 |
| ../rov_host_goto.py | 既有 DvlFeed.poll 增加默认关闭的解析预算，未改旧 PD/原点/停止逻辑 |
| tests/test_force_state.py、test_force_entry.py（新增） | 可控时间、完整缓存、依赖隔离、等待/异常/中断/轨迹检查 |
| tools/verify_direct6_sitl.py | 扩展原有 SITL；真实 CLI/中断、当前代码摘要、每轮独立证据 |
| tools/verify_matlab_alignment.py | 增加平方根/固定样本碰撞诊断，不改受测算法 |
| tools/replay_force_state.py（新增） | 只读历史记录离线回放与时间统计，FC 源按该记录的 1/1 选取 |
| README_force_zh.md、CHANGES_force_zh.md、报告 | 当前命令、问题分类、历史/本轮/待实机分开 |

`model.py/dynamics.py/kprm.py/lbts.py/execution.py` 与本轮开始的 SHA256 一致；原动力学参数、推进器几何和分配、规划搜索/机会约束、K/反馈/控制周期/非等间隔时序全部未改。`rov_control.py`、`task_origin.json`、飞控补丁/二进制及核验 false 标志未改。改过的上级 DvlFeed 增量仅为默认关闭的预算，旧基础功能 26 项回归通过；不是水中动作重测。详见 `final_manifest.json`。

## 5. 安装、离线及 SITL 命令

以下命令由当前入口和 pyproject 定义；已有环境已具备依赖，本轮实际测试用 Python 3.10.12。普通计划 pickle 仅加载本工程生成且可信的文件。

```bash
cd /home/tj/blueROV2/code
python3 -m pip install -e '.[hardware,test]'
python3 -m bluerov2.force_run --help
# 默认只做离线检查；不读取 DVL/原点/曲线或打开机器人连接
python3 -m bluerov2.force_run --output reports/pwm_check.json pwm-test --pwm 1500 1500 1500 1500 1500 1500 --duration 0.5
# 原计划格式、元数据导出方法见 README_force_zh.md 的 save_force_plan
python3 -m bluerov2.force_run --output reports/plan_check.json plan experiment.pkl
python3 -m pytest -q tests
(cd /home/tj/blueROV2 && python3 -m pytest -q tests)
python3 tools/replay_force_state.py reports/connected_validation.raw.jsonl --output reports/state_replay.json
python3 tools/verify_direct6_sitl.py --binary firmware/ardusub_3_5_4/sitl_artifacts/ardusub --output reports/direct6_sitl_revision.json
```

当前临时解包的 Octave 环境已实际用以下命令运行；退出 2 表示上述一项真实差异，不能当成全通过：

```bash
OCTAVE_HOME=/tmp/bluerov-octave/root/usr \
LD_LIBRARY_PATH=/tmp/bluerov-octave/root/usr/lib/x86_64-linux-gnu/octave/6.4.0:/tmp/bluerov-octave/root/usr/lib/x86_64-linux-gnu:/tmp/bluerov-octave/root/usr/lib \
python3 tools/verify_matlab_alignment.py --executable /tmp/bluerov-octave/root/usr/bin/octave-cli --output reports/interface_revision_20261103/matlab_alignment.json
```

此 /tmp 路径不是永久安装依赖；有正常 Octave 安装时用 `--engine octave --executable /实际路径/octave-cli`，有 MATLAB 时用 `--engine matlab --executable /实际路径/matlab`。

## 6. 真机后续顺序（本轮没有执行）

1. **先 PWM 接口验证**：确认对应实机板卡的 direct6 固件已由操作人员构建、安装，且实际版本/电气配置/能力应答符合契约。历史报告说明当时原固件未部署此接口，本轮不据此断言现在的实机状态；旧 goto/trajectory/摇杆输出停止，飞控上锁。PWM 分支不要求 DVL、原点、曲线或推力方向验证记录。
2. 一次显式执行全 1500、0.5 s PWM，检查六路电气中立及停止确认；再在适合推进器工作的环境逐路有限小幅 PWM。确认 MAIN1～6、电调中立/范围、通道一致性、指令断流回中及退出。不要将 HAL 回读当示波器或物理推力测量。
3. **再完整轨迹验证**：据逐路实际方向/位置确定模型对应与 mapping；核对固定原点、传感器安装、速度坐标、角速度与采样时间，并解决现有记录显示的真实过期问题。只有实测记录成立才更新 verified。算法/几何/周期若必须更改，单独确认后再做；本次未改。
4. 用原算法在同一固定参考下生成起点符合条件的短计划，通过 save_force_plan 导出；离线检查后一次显式 plan --execute。记录误差、原反馈及限幅、采用力、发送/回读 PWM、状态年龄/时间、退出与停止。分别验收输出与实际跟踪。

将下述命令留给具备上述前提的现场人员；本轮未运行它们，无需在 PWM 前把无关定位记录改为 true：

```bash
python3 -m bluerov2.force_run --output reports/pwm_neutral.json pwm-test --pwm 1500 1500 1500 1500 1500 1500 --duration 0.5 --execute
python3 -m bluerov2.force_run --output reports/pwm_main1.json pwm-test --pwm 1540 1500 1500 1500 1500 1500 --duration 0.5 --execute
python3 -m bluerov2.force_run --output reports/short_trajectory.json plan experiment.pkl --execute
```

本轮没有电气输出、通道推力方向、板卡看门狗、Companion 实机路由或水中轨迹结果；没有把 sequence_completed 写成 tracking_success。软件/SITL 已通过与真实机器人是否可开始完整轨迹是两项不同验收。
