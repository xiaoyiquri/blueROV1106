# 本轮真机只读核实（2026-11-03）

本轮已与真机通信，但没有进行运动测试。没有写参数、解锁/上锁、发送 RC override / MANUAL_CONTROL / GUIDED / direct6 指令，也没有改变算法。

实际采集确认：

- 飞控为 ArduSub 3.5.4，flight_custom_version 为 05fed49b，与此前核查的官方源码提交一致。
- 采集窗口内为 MANUAL、未解锁；六路 SERVO_OUTPUT_RAW 均为 1500 μs。这是驱动端采样回读，不是推进器真实推力或物理停止验收。
- 41 个只读参数全部取得，SERVO1..6_FUNCTION 仍为 33..38；PWM 范围 1100..1900、trim 1500、REVERSED=0。
- FS_PILOT_INPUT=2，FS_PILOT_TIMEOUT=3.0 秒。未将其改为更短时间。
- SYS_STATUS 电压约 15.435 V。
- 复用既有 DVL 只读采集器，在两秒采集窗口收到 18 条速度与 9 条位置报告，采集结束时满足现有 0.25 秒新鲜度条件。此结果不证明完整 12 维状态坐标、同步或安装映射已经验收，也不作为 PWM 单独测试的额外门槛。

端口与控制来源：

首次检查 QGroundControl 占用 UDP14550；保存的 Vehicle1 配置 JoystickEnabled=true。该保存值不等于本轮观察到了实际摇杆包。随后端口自行释放，本程序没有终止 QGroundControl，也没有抢占 socket。飞控读取期间没有发送 GCS 心跳，避免维持别的会话的 GCS failsafe。

发送白名单仅允许一次 MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES（520）与 41 次 PARAM_REQUEST_READ。原始接收与请求记录保存在同目录 JSONL/CSV。

为何没有进入运动：

此前精确 3.5.4 原始函数测试与逐次 HAL 跟踪已证明，仅改 FUNCTION 为 1 或 51..56 仍存在原电机循环写入。现在从真机读到的版本/提交与该证据对应；连接成功或回读一次正确不能消除这个输出覆盖问题。此前软件/SITL 结果保留为历史证据，没有冒称本轮实际测试。

此外，本轮已询问机器人是否入水、推进器周围是否安全以及现场能否立即断电，截止本报告没有收到答复。因此没有发起短时运动，也没有用旧手动控制替代你要求的直通方案。

尚未验证：实际六路电气输出的一致性、实际断流回中、停止确认、物理位移/跟踪效果。后续必须先解决并验证直通输出路径的已知覆盖问题；现场确认本身不足以把该路径判定为合格。

原始报告：read_only.json、conclusion.json；前一阶段 DVL 采集报告的位置见 conclusion.json。
