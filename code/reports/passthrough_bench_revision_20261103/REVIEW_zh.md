# 本轮台架入口与验证报告（2026-11-03）

## 交付

已新增可运行的默认只读/隔离动力台架入口、参数备份恢复、45 项新测试和中文操作说明。详细命令见项目根目录 README_passthrough_bench_zh.md。

本轮没有把原版 3.5.4 直通路径改造成已验收的运动后端。未收到现场推进器动力隔离确认，因此未在真机上执行非中立输出或任何 PARAM_SET。没有解锁、上锁、改变模式或刷固件。

## 本轮实际验证

- 全部项目测试：230 passed；原基础功能：26 passed；两组定向测试合计60 passed（包含45项本轮新增台架测试和15项此前直通协议测试）。
- 新增测试覆盖只读默认零输出、无关 DVL/原点/曲线依赖、完整六路帧、参数备份失败、部分配置失联、超时、用户中断、参数恢复、恢复中意外解锁、时序超限、设备时钟回退、缺失停止确认、故障后不恢复输出，以及读写分支的不同互斥需求。
- 最终程序在真机默认只读模式完成版本与35项参数采集。没有 PARAM_SET 或 RC override，输出类型仅 COMMAND_LONG（520）与 PARAM_REQUEST_READ。采集时为未解锁 MANUAL；固件仍为3.5.4 / 05fed49b。
- 复用了原版3.5.4完整SITL进程运行台架CLI，确认六路图样进入RC输入及采样驱动回读，退出时恢复六个原始FUNCTION值并确认未解锁心跳。
- 3.5.4编译时排除了SITL的pilot-input failsafe：子CLI正确报告watchdog_neutral_not_observed并返回1，然后主动清理和恢复参数。父联测要求这个负结果，并未把看门狗判定通过。
- 真机只读与SITL使用同一份冻结主机代码，SHA256见manifest.json及各报告bench_source_sha256。

## 本轮发现并修复的问题

首次并行全量回归有1项新测试失败：只读入口也占用全局输出锁，与并行台架程序冲突。保留了原始失败日志；只读分支不再申请执行器输出锁，真正输出/恢复分支仍独占，并增加了相应回归测试。没有删断言、扩大状态时限或屏蔽故障。

台架只发送固定0.3秒图样；发生刷新超限不补发历史帧。原规划、动力学、反馈执行器、状态适配器及旧基础控制文件与前轮散列一致，未作行为修改。原输出覆盖问题的源码/HAL跟踪作为历史证据引用，没有重复包装成本轮电气测试。

## 尚未验证

- 实机FUNCTION切换、非中立PWM、静默后的实际飞控回中事件、实机参数恢复：动力隔离确认后才能运行，当前未执行。
- 六路电气波形、更新时间一致性、物理断流回中：尚无示波器/逻辑分析仪数据。
- 真实推进器推力、运动、轨迹跟踪：本工具不执行也未验收。

telemetry_cleanup_confirmed仅表示数字反馈中的清理确认。electrical_output_verified、electrical_stop_confirmed和trajectory_or_motion_verified不会由软件回读自动置为true。

## 主要文件

- bluerov2/passthrough_bench.py：入口与小范围台架事务。
- tests/test_passthrough_bench.py：可控制时钟的无硬件测试。
- tools/verify_passthrough_sitl.py：仅新增bench-check，复用已有localhost模拟器。
- README_passthrough_bench_zh.md：只读、隔离动力台架、恢复及测量说明。
- live_readonly_release/report.json：本轮最终真机只读原始报告。
- stock_sitl_bench_release.json：本轮最终原版SITL父报告；子报告位置包含在文件中。
- manifest.json：代码散列、测试结果、未变动文件及未完成的物理条件。
