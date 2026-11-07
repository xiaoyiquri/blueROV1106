#!/usr/bin/env python3
"""基于飞控 EKF 位置反馈的单目标点 GUIDED 控制。

本程序只发送 SET_POSITION_TARGET_LOCAL_NED 位置目标，ArduSub 负责位置控制
和六推进器混控。它不会把 DVL 速度直接当作位置，也不会在没有有效飞控位置、
速度和估计器状态时退化为 MANUAL_CONTROL。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from statistics import mean
from typing import Any, Dict, List, Optional, Sequence, Tuple

from pymavlink import mavutil

from rov_control import (
    ControlError,
    Controller,
    SessionLogger,
    State,
    install_signal_handlers,
)


# MAVLink EKF_STATUS_FLAGS（common.xml）中与本任务相关的位。
EKF_ATTITUDE = 1 << 0
EKF_VELOCITY_HORIZ = 1 << 1
EKF_VELOCITY_VERT = 1 << 2
EKF_POS_HORIZ_REL = 1 << 3
EKF_POS_HORIZ_ABS = 1 << 4
EKF_POS_VERT_AGL = 1 << 5
EKF_POS_VERT_ABS = 1 << 6
EKF_CONST_POS_MODE = 1 << 7
EKF_UNINITIALIZED = 1 << 10
EKF_GPS_GLITCH = 1 << 11

LOCAL_NED_FRAME = 1
POSITION_ONLY_TYPE_MASK = 0x0DF8
POSITION_FIELDS = ("x_m", "y_m", "z_m", "vx_m_s", "vy_m_s", "vz_m_s")

DEFAULT_CONFIG = {
    "position_timeout_s": 1.0,
    "ekf_timeout_s": 1.0,
    "parameter_timeout_s": 3.0,
    "target_update_hz": 5.0,
    "horizontal_tolerance_m": 0.25,
    "vertical_tolerance_m": 0.20,
    "speed_threshold_m_s": 0.15,
    "arrival_hold_s": 1.0,
    "origin_sample_duration_s": 2.0,
    "origin_max_speed_m_s": 0.10,
    "origin_jump_threshold_m": 2.0,
    "task_timeout_s": 60.0,
    "require_external_nav": True,
}


class GotoError(ControlError):
    pass


def finite_positive(value: Any, name: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise GotoError(f"{name} 必须是数字") from exc
    if not math.isfinite(number) or number <= 0:
        raise GotoError(f"{name} 必须是有限正数")
    return number


def finite_number(value: Any, name: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise GotoError(f"{name} 必须是数字") from exc
    if not math.isfinite(number):
        raise GotoError(f"{name} 必须是有限数")
    return number


def load_config(path: Path) -> Dict[str, Any]:
    config = dict(DEFAULT_CONFIG)
    if path.exists():
        try:
            loaded = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise GotoError(f"读取配置失败：{exc}") from exc
        if not isinstance(loaded, dict):
            raise GotoError("配置根对象必须是 JSON 对象")
        config.update(loaded)
    for key in (
        "position_timeout_s",
        "ekf_timeout_s",
        "parameter_timeout_s",
        "target_update_hz",
        "horizontal_tolerance_m",
        "vertical_tolerance_m",
        "speed_threshold_m_s",
        "arrival_hold_s",
        "origin_sample_duration_s",
        "origin_max_speed_m_s",
        "origin_jump_threshold_m",
        "task_timeout_s",
    ):
        config[key] = finite_positive(config[key], key)
    if not isinstance(config.get("require_external_nav"), bool):
        raise GotoError("require_external_nav 必须是布尔值")
    if config["target_update_hz"] > 20:
        raise GotoError("target_update_hz 不能超过 20 Hz")
    return config


def _parameter_requests(controller: Controller, names: Sequence[str], timeout: float) -> None:
    for name in names:
        controller.master.mav.param_request_read_send(
            controller.target_system,
            controller.target_component,
            name.encode("ascii"),
            -1,
        )
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        controller.service_io()
        if all(name in controller.vehicle.parameters for name in names):
            return
        time.sleep(0.005)


def request_telemetry_intervals(controller: Controller) -> None:
    """请求本次进程所需的临时消息频率；旧固件拒绝时仍由有效性检查阻止导航。"""
    requests = (
        (getattr(mavutil.mavlink, "MAVLINK_MSG_ID_LOCAL_POSITION_NED", 32), 10.0),
        (getattr(mavutil.mavlink, "MAVLINK_MSG_ID_EKF_STATUS_REPORT", 193), 5.0),
    )
    for message_id, frequency in requests:
        controller.master.mav.command_long_send(
            controller.target_system,
            controller.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            message_id,
            1_000_000.0 / frequency,
            0,
            0,
            0,
            0,
            0,
        )
        controller.record_event("request_goto_message_interval", message_id=message_id, frequency_hz=frequency)


def external_nav_evidence(controller: Controller, config: Dict[str, Any]) -> Tuple[bool, str]:
    """只依据参数和当前 EKF 证据判断外部导航配置，不把位置消息本身当成融合证据。"""
    if not config["require_external_nav"]:
        return True, "配置关闭了 external-nav 参数证据要求"
    names = ("EK3_ENABLE", "AHRS_EKF_TYPE", "EK2_ENABLE", "VISO_TYPE", "EK3_SRC1_POSXY", "EK3_SRC1_VELXY")
    missing = [name for name in names if name not in controller.vehicle.parameters]
    if missing:
        return False, f"未读到外部导航配置参数：{', '.join(missing)}"
    params = controller.vehicle.parameters
    if round(params["EK3_ENABLE"]) != 1:
        return False, f"EK3_ENABLE={params['EK3_ENABLE']}，未启用 EKF3"
    if round(params["AHRS_EKF_TYPE"]) != 3:
        return False, f"AHRS_EKF_TYPE={params['AHRS_EKF_TYPE']}，不是 EKF3"
    if round(params["EK2_ENABLE"]) != 0:
        return False, f"EK2_ENABLE={params['EK2_ENABLE']}，本路径要求关闭 EKF2"
    if round(params["VISO_TYPE"]) != 1:
        return False, f"VISO_TYPE={params['VISO_TYPE']}，本路径要求 MAVLink 外部导航类型 1"
    if round(params["EK3_SRC1_POSXY"]) != 6 or round(params["EK3_SRC1_VELXY"]) != 6:
        return False, (
            f"EK3_SRC1_POSXY={params['EK3_SRC1_POSXY']}, "
            f"EK3_SRC1_VELXY={params['EK3_SRC1_VELXY']}，不是 ExternalNav(6)"
        )
    return True, "EKF3、VISO_TYPE 和水平位置/速度 ExternalNav 参数已读到"


def estimator_status(controller: Controller, config: Dict[str, Any]) -> Tuple[bool, str, int]:
    status = controller.vehicle.ekf_status
    if not status or controller.vehicle.last_ekf_status_mono is None:
        return False, "没有 EKF_STATUS_REPORT", 0
    age = time.monotonic() - controller.vehicle.last_ekf_status_mono
    if age > config["ekf_timeout_s"]:
        return False, f"EKF_STATUS_REPORT 过期 {age:.2f}s", 0
    flags = int(status.get("flags", 0))
    horiz_pos = flags & (EKF_POS_HORIZ_REL | EKF_POS_HORIZ_ABS)
    vert_pos = flags & (EKF_POS_VERT_AGL | EKF_POS_VERT_ABS)
    required = EKF_ATTITUDE | EKF_VELOCITY_HORIZ | EKF_VELOCITY_VERT
    missing = required & ~flags
    if not horiz_pos:
        return False, f"EKF 没有水平位置有效标志 flags={flags}", flags
    if not vert_pos:
        return False, f"EKF 没有垂直位置有效标志 flags={flags}", flags
    if missing:
        return False, f"EKF 缺少速度/姿态有效标志 0x{missing:x}, flags={flags}", flags
    if flags & (EKF_CONST_POS_MODE | EKF_UNINITIALIZED | EKF_GPS_GLITCH):
        return False, f"EKF 处于常位置/未初始化/GPS glitch 状态 flags={flags}", flags
    return True, "EKF 姿态、水平/垂直速度和位置有效", flags


def current_position(controller: Controller, config: Dict[str, Any]) -> Tuple[bool, str, Optional[Dict[str, float]]]:
    position = controller.vehicle.local_position
    if not position or controller.vehicle.last_local_position_mono is None:
        return False, "没有 LOCAL_POSITION_NED", None
    age = time.monotonic() - controller.vehicle.last_local_position_mono
    if age > config["position_timeout_s"]:
        return False, f"LOCAL_POSITION_NED 过期 {age:.2f}s", None
    result: Dict[str, float] = {}
    for field in POSITION_FIELDS:
        value = position.get(field)
        if value is None or not math.isfinite(float(value)):
            return False, f"LOCAL_POSITION_NED.{field} 缺失或无效", None
        result[field] = float(value)
    return True, "位置和速度新鲜且有限", result


def readiness(controller: Controller, config: Dict[str, Any]) -> Tuple[bool, List[str]]:
    reasons: List[str] = []
    ok_external, external_reason = external_nav_evidence(controller, config)
    if not ok_external:
        reasons.append(external_reason)
    ok_ekf, ekf_reason, _ = estimator_status(controller, config)
    if not ok_ekf:
        reasons.append(ekf_reason)
    ok_pos, pos_reason, _ = current_position(controller, config)
    if not ok_pos:
        reasons.append(pos_reason)
    return not reasons, reasons


def collect_origin(controller: Controller, config: Dict[str, Any]) -> Tuple[float, float, float]:
    deadline = time.monotonic() + config["origin_sample_duration_s"]
    samples: List[Tuple[float, float, float]] = []
    while time.monotonic() < deadline:
        controller.service_io()
        ok, reason, position = current_position(controller, config)
        if not ok or position is None:
            raise GotoError(f"任务原点采样失败：{reason}")
        ok_ekf, reason_ekf, _ = estimator_status(controller, config)
        if not ok_ekf:
            raise GotoError(f"任务原点采样失败：{reason_ekf}")
        speed = math.sqrt(position["vx_m_s"] ** 2 + position["vy_m_s"] ** 2 + position["vz_m_s"] ** 2)
        if speed > config["origin_max_speed_m_s"]:
            raise GotoError(f"任务开始时速度 {speed:.3f}m/s 超过 origin_max_speed_m_s")
        samples.append((position["x_m"], position["y_m"], position["z_m"]))
        time.sleep(0.01)
    if len(samples) < 3:
        raise GotoError("任务原点样本不足")
    return tuple(mean(sample[index] for sample in samples) for index in range(3))  # type: ignore[return-value]


def relative_error(position: Dict[str, float], target: Tuple[float, float, float], origin: Tuple[float, float, float]) -> Tuple[float, float, float, float, float]:
    current = (position["x_m"] - origin[0], position["y_m"] - origin[1], position["z_m"] - origin[2])
    error = (target[0] - current[0], target[1] - current[1], target[2] - current[2])
    horizontal = math.hypot(error[0], error[1])
    vertical = abs(error[2])
    speed = math.sqrt(position["vx_m_s"] ** 2 + position["vy_m_s"] ** 2 + position["vz_m_s"] ** 2)
    return horizontal, vertical, speed, current[0], current[1]


def send_position_target(controller: Controller, target: Tuple[float, float, float]) -> None:
    if controller.master is None or controller.target_system is None:
        raise GotoError("尚未连接到目标飞控")
    # SET_POSITION_TARGET_LOCAL_NED 的时间字段是飞控 boot 时间（毫秒），
    # 不是 Ubuntu 的 wall/monotonic 时间；沿用最近一帧飞控位置的时间戳。
    raw_time_boot_ms = controller.vehicle.local_position.get("time_boot_ms") if controller.vehicle.local_position else None
    time_boot_ms = int(raw_time_boot_ms) & 0xFFFFFFFF if raw_time_boot_ms is not None else 0
    controller.master.mav.set_position_target_local_ned_send(
        time_boot_ms,
        controller.target_system,
        controller.target_component,
        LOCAL_NED_FRAME,
        POSITION_ONLY_TYPE_MASK,
        target[0],
        target[1],
        target[2],
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,  # yaw ignored by type mask; no accidental zero-heading command
        0.0,  # yaw rate ignored
    )
    controller.logger.write_log(
        "tx_position_target_local_ned",
        state=controller.state.value,
        target_x_m=target[0],
        target_y_m=target[1],
        target_z_m=target[2],
        target_time_boot_ms=time_boot_ms,
        type_mask=POSITION_ONLY_TYPE_MASK,
        coordinate_frame=LOCAL_NED_FRAME,
        **controller.snapshot(),
    )


def check_guided_safety(controller: Controller, config: Dict[str, Any]) -> Tuple[bool, str]:
    if not controller.vehicle.heartbeat_fresh(time.monotonic(), controller.heartbeat_timeout):
        return False, "飞控 HEARTBEAT 过期"
    if controller.vehicle.mode != "GUIDED":
        return False, f"飞控模式意外变化为 {controller.vehicle.mode!r}"
    if controller.vehicle.armed is not True:
        return False, "飞控自行上锁"
    ok, reason, _ = current_position(controller, config)
    if not ok:
        return False, reason
    ok, reason, _ = estimator_status(controller, config)
    if not ok:
        return False, reason
    return True, "GUIDED 和定位状态正常"


def run_goto(controller: Controller, config: Dict[str, Any], target_relative: Tuple[float, float, float]) -> None:
    controller.connect()
    try:
        if controller.vehicle.armed:
            raise GotoError("启动时飞控已解锁，拒绝自动接管")
        request_autopilot_version(controller)
        request_telemetry_intervals(controller)
        names = ("EK3_ENABLE", "AHRS_EKF_TYPE", "EK2_ENABLE", "VISO_TYPE", "EK3_SRC1_POSXY", "EK3_SRC1_VELXY")
        _parameter_requests(controller, names, config["parameter_timeout_s"])
        controller.service_io()
        ok, reasons = readiness(controller, config)
        controller.record_event("goto_readiness", ready=ok, reasons=reasons, target_relative=target_relative)
        if not ok:
            raise GotoError("定位不满足导航启动条件：" + "；".join(reasons))
        origin = collect_origin(controller, config)
        target_fcu = (origin[0] + target_relative[0], origin[1] + target_relative[1], origin[2] + target_relative[2])
        controller.record_event("goto_origin", origin_x_m=origin[0], origin_y_m=origin[1], origin_z_m=origin[2], target_relative=target_relative, target_fcu=target_fcu)

        controller.request_mode("GUIDED")
        controller.set_state(State.NEUTRAL)
        controller.request_arm(send_neutral=False)
        controller.action_name = "goto"
        controller.set_state(State.MOVING)
        controller.record_event("goto_start", target_fcu=target_fcu)

        deadline = time.monotonic() + config["task_timeout_s"]
        period = 1.0 / config["target_update_hz"]
        next_send = time.monotonic()
        arrival_since: Optional[float] = None
        previous_absolute: Optional[Tuple[float, float, float]] = None
        while time.monotonic() < deadline and not controller.stop_requested:
            now = time.monotonic()
            controller.service_io(now)
            safe, reason = check_guided_safety(controller, config)
            if not safe:
                raise GotoError(reason)
            ok, reason, position = current_position(controller, config)
            if not ok or position is None:
                raise GotoError(reason)
            absolute = (position["x_m"], position["y_m"], position["z_m"])
            if previous_absolute is not None:
                jump = math.sqrt(sum((absolute[index] - previous_absolute[index]) ** 2 for index in range(3)))
                if jump > config["origin_jump_threshold_m"]:
                    raise GotoError(f"检测到位置/原点跳变 {jump:.3f}m，终止任务")
            previous_absolute = absolute
            horizontal, vertical, speed, current_x, current_y = relative_error(position, target_relative, origin)
            controller.record_event(
                "goto_sample",
                current_x_m=current_x,
                current_y_m=position["y_m"] - origin[1],
                current_z_m=position["z_m"] - origin[2],
                error_horizontal_m=horizontal,
                error_vertical_m=vertical,
                speed_m_s=speed,
            )
            within = (
                horizontal <= config["horizontal_tolerance_m"]
                and vertical <= config["vertical_tolerance_m"]
                and speed <= config["speed_threshold_m_s"]
            )
            if within:
                if arrival_since is None:
                    arrival_since = now
                elif now - arrival_since >= config["arrival_hold_s"]:
                    controller.record_event("goto_arrived", hold_duration_s=now - arrival_since)
                    return
            else:
                arrival_since = None
            if now >= next_send:
                send_position_target(controller, target_fcu)
                next_send = now + period
            time.sleep(0.005)
        if controller.stop_requested:
            raise GotoError(controller.termination_reason or "收到终止请求")
        raise GotoError("目标任务超时")
    finally:
        if controller.master is not None and controller.control_responsibility and not controller.disarm_request_attempted:
            try:
                # GUIDED 阶段不发送 MANUAL_CONTROL；上锁本身会停止推进器。
                controller.stop_and_disarm(send_neutral=False)
            except Exception as exc:
                controller.state = State.FAULT
                controller.record_event("goto_cleanup_exception", reason=str(exc))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="BlueROV2 ArduSub NED 单目标点闭环控制")
    parser.add_argument("--connection", help="pymavlink 连接字符串；默认从配置文件读取")
    parser.add_argument("--baud", type=int)
    parser.add_argument("--target-system", type=int)
    parser.add_argument("--target-component", type=int)
    parser.add_argument("--source-system", type=int)
    parser.add_argument("--source-component", type=int)
    parser.add_argument("--log-dir", type=Path)
    parser.add_argument("--config", type=Path, default=Path("goto_config.json"))
    parser.add_argument("--connect-timeout", type=float)
    parser.add_argument("--confirm-timeout", type=float)
    parser.add_argument("--heartbeat-timeout", type=float)
    parser.add_argument("--check", action="store_true", help="只读定位/估计器/外部导航检查")
    parser.add_argument("--x", type=float)
    parser.add_argument("--y", type=float)
    parser.add_argument("--z", type=float)
    return parser


def make_controller(args: argparse.Namespace, logger: SessionLogger) -> Controller:
    return Controller(
        connection=args.connection,
        baud=args.baud,
        source_system=args.source_system,
        source_component=args.source_component,
        target_system=args.target_system,
        target_component=args.target_component,
        logger=logger,
        connect_timeout=finite_positive(args.connect_timeout, "connect-timeout"),
        confirm_timeout=finite_positive(args.confirm_timeout, "confirm-timeout"),
        heartbeat_timeout=finite_positive(args.heartbeat_timeout, "heartbeat-timeout"),
    )


def request_autopilot_version(controller: Controller) -> None:
    controller.master.mav.command_long_send(
        controller.target_system,
        controller.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
    )


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        config = load_config(args.config)
    except GotoError as exc:
        parser.error(str(exc))
        return 2
    args.connection = args.connection or config.get("connection")
    if not args.connection:
        parser.error("必须通过 --connection 或 goto_config.json.connection 提供连接端点")
    args.baud = args.baud if args.baud is not None else int(config.get("baud", 115200))
    args.source_system = args.source_system if args.source_system is not None else int(config.get("source_system", 255))
    args.source_component = args.source_component if args.source_component is not None else int(config.get("source_component", 190))
    args.target_system = args.target_system if args.target_system is not None else config.get("target_system")
    args.target_component = args.target_component if args.target_component is not None else config.get("target_component")
    args.log_dir = Path(args.log_dir or config.get("log_dir", "logs"))
    args.connect_timeout = args.connect_timeout if args.connect_timeout is not None else float(config.get("connect_timeout", 15.0))
    args.confirm_timeout = args.confirm_timeout if args.confirm_timeout is not None else float(config.get("confirm_timeout", 5.0))
    args.heartbeat_timeout = args.heartbeat_timeout if args.heartbeat_timeout is not None else float(config.get("heartbeat_timeout", 3.0))
    if not args.check and any(value is None for value in (args.x, args.y, args.z)):
        parser.error("执行 goto 时必须同时提供 --x、--y、--z")
    target = None if args.check else tuple(finite_number(value, name) for value, name in ((args.x, "x"), (args.y, "y"), (args.z, "z")))
    logger = SessionLogger(args.log_dir, action="goto")
    controller = make_controller(args, logger)
    install_signal_handlers(controller)
    exit_code = 0
    original_error: Optional[BaseException] = None
    try:
        controller.connect()
        request_autopilot_version(controller)
        request_telemetry_intervals(controller)
        names = ("EK3_ENABLE", "AHRS_EKF_TYPE", "EK2_ENABLE", "VISO_TYPE", "EK3_SRC1_POSXY", "EK3_SRC1_VELXY")
        _parameter_requests(controller, names, config["parameter_timeout_s"])
        controller.service_io()
        external_ok, external_reason = external_nav_evidence(controller, config)
        ekf_ok, ekf_reason, flags = estimator_status(controller, config)
        pos_ok, pos_reason, position = current_position(controller, config)
        print(f"target={controller.target_system}/{controller.target_component} mode={controller.vehicle.mode} armed={controller.vehicle.armed}")
        print(f"external_nav={external_ok}: {external_reason}")
        print(f"ekf={ekf_ok}: {ekf_reason} flags={flags}")
        print(f"position={pos_ok}: {pos_reason} value={position}")
        controller.record_event("goto_check", external_nav_ok=external_ok, external_nav_reason=external_reason, ekf_ok=ekf_ok, ekf_reason=ekf_reason, ekf_flags=flags, position_ok=pos_ok, position_reason=pos_reason)
        if args.check:
            if not (external_ok and ekf_ok and pos_ok):
                exit_code = 2
            return exit_code
        run_goto(controller, config, target)  # type: ignore[arg-type]
        if controller.disarm_confirmed:
            print("goto 完成：已确认上锁。")
        else:
            exit_code = 2
    except KeyboardInterrupt:
        controller.stop_requested = True
        controller.termination_reason = "Ctrl+C"
        original_error = GotoError("Ctrl+C")
        exit_code = 130
    except (ControlError, OSError, ValueError) as exc:
        original_error = exc
        controller.termination_reason = str(exc)
        if controller.state not in (State.DONE, State.FAULT):
            controller.state = State.FAULT
        print(f"错误：{exc}", file=sys.stderr)
        exit_code = 1
    finally:
        if original_error:
            controller.record_event("goto_exit", reason=str(original_error), final_status=("confirmed_disarmed" if controller.disarm_confirmed else "unknown_or_unconfirmed"))
        if not args.check and controller.control_responsibility:
            if controller.disarm_confirmed:
                print("最终状态：已确认上锁。")
            elif controller.disarm_request_sent:
                print("最终状态：已请求上锁但未确认；请按日志和现场状态核实。", file=sys.stderr)
                if exit_code == 0:
                    exit_code = 2
            else:
                print("最终状态：链路断开或清理未完成，最终解锁状态未知。", file=sys.stderr)
                if exit_code == 0:
                    exit_code = 2
        controller.close()
        logger.close()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
