这是针对已读取到的 **ArduSub 3.5.4，commit `05fed49b8ee1db3c6bca924e0b334b2e0ece980d`** 的最小源码补丁。`reports/live_diagnosis.json` 的 `flight_custom_version` 解码为 `05fed49b`，与上游标签一致。补丁已完成完整 ArduSub SITL 构建及11项完整进程/实际 Python 接口联测，见 `../../reports/direct6_sitl.json`。尚未构建真实板卡镜像或刷入飞控，SITL不能替代电气和水中验证。

为什么 Python 不能单独完成

原版 `ArduSub/motors.cpp::motors_output()` 每个快速循环调用 `motors.output()`；`AP_Motors6DOF::output_to_motors()` 会再次混控并写输出。原版 `DO_SET_SERVO` 经 ServoRelayEvents 检查通道用途；电机测试接口是独立测试状态，不能据此证明连续六路控制正确。直接发送六次命令还缺少完整帧提交、顺序控制和与本轨迹一致的失效处理。因此本实现不使用这两条路径。

最小修改范围

| 文件 | 作用 |
|---|---|
| `Sub.h` | 加入会话状态和声明 |
| `direct6_core.h`（新增） | 完整帧校验、会话所有权、序号、有效期、超时锁存回中 |
| `direct6.cpp`（新增） | MAVLink 接收、旧输入隔离、HAL 输出回读状态 |
| `GCS_Mavlink.cpp` | 接收专用载荷；会话期间阻止旧操纵、目标、模式和参数写入 |
| `motors.cpp` | 会话期间在 `cork/write(六路)/push` 后直接返回，跳过原电机循环 |
| `ArduSub.cpp` | 会话期间暂停已有 servo relay 重复事件，防止另一个输出写入者 |
| `failsafe.cpp` | 会话有效期间由专用 200 ms 看门狗负责输入超时，不要求伪造 pilot/MANUAL_CONTROL |

GCS 心跳、通信、姿态遥测、漏水/电池等其他 failsafe 继续运行。任何内部模式变化导致 MANUAL 条件不满足时，专用输出锁存中立，不切回混控继续带动力输出。CPU 主循环失效保护沿用原固件；本补丁的 200 ms 指令看门狗在正常快速循环内运行。

接口约定

复用 MAVLink 1 可承载的 `DATA64`（ArduPilotMega 中64字节的数据容器，type=0xD6、len=34），这是一条**项目专用协议**，不是通用 ArduSub PWM API。需要确认 Companion 路由传递此消息。完整六路装在同一 MAVLink 包，MAVLink 校验包完整性。实际顺序使用载荷中的32位序号。每个MAVLink1包约74字节，40ms重发约1.85kB/s，返程状态同量；仍需实测旧Companion串口带宽和遥测负载。

前 34 字节使用小端 `'<4sBBBBIIIH6H'`，其余补零至 64 字节：

| 字段 | 请求 `BT6!` | 应答 `BS6!` |
|---|---|---|
| 4 字节 magic | `BT6!` | `BS6!` |
| uint8 | 版本 1 | 版本 1 |
| uint8 | QUERY=0 / ACQUIRE=1 / FRAME=2 / STOP=3 / RELEASE=4 | state低2位：IDLE=0/HELD=1/FAULT=2；bit2 armed，bit3 ready |
| uint8 × 2 | 目标飞控 system/component | 当前所有者 system/component |
| uint32 | 随机会话 ID，非零 | 会话 ID |
| uint32 | 帧序号 | 最近接纳的控制序号 |
| uint32 | 飞控启动时钟中的绝对到期毫秒 | 飞控当前启动毫秒 |
| uint16 | 保留，必须为 0 | 剩余有效毫秒 |
| uint16 × 6 | 物理 MAIN1～MAIN6 的 PWM μs | 六路 HAL driver readback μs |

ACQUIRE 仅在已上锁、标准 Vectored、MANUAL、无电机测试，且最近 1 秒无旧操纵/模式指令时接纳。初始序号必须为 1。会话以来源 system/component 和随机 ID 共同绑定，不自动接管其他会话。接纳后由 Python 发送中立帧并显式请求解锁。

每个 FRAME 先验证全部通道为 1100～1900 μs，再一次提交六路；过期、重复/倒退序号、非法 PWM、解锁后再次上锁、模式条件失效或到期均锁存 1500 μs。到期时间必须在飞控当前时间后的 1～200 ms 内。回中后不能重放旧帧恢复动力，必须完成停止、上锁和 RELEASE 后重新启动。停止命令只作用于该会话所有者；普通 MAVLink 上锁命令仍可由其他来源请求。

Python 以 40 ms 周期重发已保持的 PWM，控制反馈只按计划区间计算。重复发送使用新协议序号，PWM 不变。200 ms 是固件断流回中界限；实际界限另加一个快速循环及板卡输出驱动延迟，需示波器/SITL 核验。

HAL 的 `cork/push` 提供批量提交；不同板卡定时器组的六路电气边沿同步程度仍需测量。HAL 回读能证明驱动端记录的 PWM，不能证明电调、桨转速或真实推力。不要把回读字段改名为实测推力。

本机验证与应用

```bash
cd /home/tj/blueROV2/code
g++ -std=c++11 -Wall -Wextra -Werror firmware/ardusub_3_5_4/test_direct6.cpp -o /tmp/direct6-test
/tmp/direct6-test
```

在**另一个源码工作区**检出精确标签，先备份现有固件/参数；以下步骤不刷写设备：

```bash
git clone --branch ArduSub-3.5.4 --depth 1 https://github.com/ArduPilot/ardupilot.git ardusub-direct6
cd ardusub-direct6
git rev-parse HEAD
# 必须为 05fed49b8ee1db3c6bca924e0b334b2e0ece980d
git submodule update --init --recursive
git apply --check /home/tj/blueROV2/code/firmware/ardusub_3_5_4/direct6.patch
git apply /home/tj/blueROV2/code/firmware/ardusub_3_5_4/direct6.patch
```

按 **3.5.4 对应版本**的开发环境准备其旧工具链。在隔离工作区先构建 SITL（该版本通常使用 `./waf configure --board sitl`、`./waf sub`；若依赖/工具链不兼容，先解决构建，不能继续声称已适配）。真实板卡目标需依据实际板卡型号选择；本次 `board_version=0` 不能证明 Pixhawk 的具体硬件型号，因此不提供猜测的板卡目标或刷写命令。

完整固件/SITL 必须再验证：

1. 原始固件无 `BS6!` 应答；补丁固件 QUERY 应答，IDLE 时原功能保持。
2. ACQUIRE 拒绝已解锁、非标准机架、非 MANUAL、活跃旧操纵和第二所有者。
3. 解锁后一个完整六路帧产生 MAIN1～MAIN6 指定值；故意输入旧 MANUAL/GUIDED/servo/motor-test 指令不能覆盖它。
4. 暂停/杀死主机进程，200 ms 加执行延迟内六路中立，FAULT 锁存；迟到包不能恢复。
5. 上锁、模式内部变化、无效帧、FC 重启均不继续旧推力。
6. 停止后 HAL 回读中立、收到新上锁心跳、RELEASE 应答 IDLE。断开应答链路时主机必须报告 `stop_unconfirmed`。
7. 在实际板卡用输出测量确认六路时序、范围、中立和 CPU 失效保护。

完成构建、安装与上述证据后才把配置中的 `output.firmware_verified` 设为 true。程序依然要求实时版本、参数和专用能力应答，不会仅凭布尔值驱动原固件。

恢复

会话正常释放后，已上锁状态下恢复原控制入口。异常时专用接口保持 FAULT 中立；先停止所有控制程序，核实中立并上锁。若需要恢复未修改固件，人工恢复备份的精确固件及核对参数，再只读诊断。补丁不写持久参数，没有六路 servo function 的切换/还原脚本。不要对未确认已停止的真机执行升级或复位来代替停止确认。

来源：上游 [ArduSub-3.5.4](https://github.com/ArduPilot/ardupilot/tree/ArduSub-3.5.4)。`source_manifest.json` 保存参与修改的上游文件 SHA256；`direct6_core.h` 与补丁内新增文件相同，本机测试和 Python 协议集成测试编译的是这一份实际核心代码。完整 SITL 已编译并验证；真实板卡构建与输出测量仍待完成，原控制器未被删除。


本轮完整 SITL 证据（2026-11-03）

`sitl_artifacts/ardusub` 是 Linux x86_64 测试程序，不能刷入真实飞控。`sitl_build_manifest.json` 固定官方源码归档/子模块版本、散列、主机工具链和兼容处理；测试记录见 `../../reports/firmware_build.log`、`../../reports/direct6_sitl.json`。

在 Ubuntu 22.04/Python 3.10 下，旧 waf 的生成器 StopIteration、旧 pymavlink array.fromstring 需要主机兼容处理，另为官方归档注入精确版本元数据。修改完整保存于 `sitl_host_compat.patch`，仅涉及构建脚本，不属于飞控运行时补丁。旧上游在 GCC 11 的 array-bounds/unused-but-set-variable/maybe-uninitialized 警告降级仅用于本次 SITL，不据此认可板卡飞行构建。

复现需要先从 manifest 的固定 source_urls 解包主源码、waf、mavlink 及其 pymavlink 子模块到对应目录，根目录保存 manifest 中 submodule_refs 为 submodule-refs.json。然后在隔离源码目录执行：

```bash
git apply --check /home/tj/blueROV2/code/firmware/ardusub_3_5_4/direct6.patch
git apply /home/tj/blueROV2/code/firmware/ardusub_3_5_4/direct6.patch
git apply /home/tj/blueROV2/code/firmware/ardusub_3_5_4/sitl_host_compat.patch
CXXFLAGS='-Wno-error=array-bounds -Wno-error=unused-but-set-variable -Wno-error=maybe-uninitialized -Wno-ignored-qualifiers -Wno-address-of-packed-member -Wno-class-memaccess' python3 modules/waf/waf-light configure --board sitl --no-submodule-update --disable-tests --no-autoconfig
python3 modules/waf/waf-light sub -j4
cd /home/tj/blueROV2/code
python3 tools/verify_direct6_sitl.py --binary /隔离源码目录/build/sitl/bin/ardusub
```

本轮11项包括六路完整帧到HAL、旧控制指令隔离、所有权、断流无主机stop自动回中、迟到帧不能恢复、回中/上锁/释放，以及真实Python推力接口到HAL的联测。固件验证标志在真机配置中继续为false。
