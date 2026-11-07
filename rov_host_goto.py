#!/usr/bin/env python3
"""不升级旧 ArduSub 时的主机侧 DVL 外环单目标点控制。

本程序使用 Water Linked A50 的 DVL 航位推算/速度，在 Ubuntu 计算位置误差，
再发送 MANUAL_CONTROL。ArduSub 只负责输入转换和六推进器混控；本程序明确
不使用 ArduSub EKF/位置控制器，因此存在 DVL 漂移、安装矩阵和增益标定限制。
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import time
from pathlib import Path
from typing import Any, Dict, Optional, Sequence, Tuple

from dvl_mavlink_bridge import DvlSample, parse_dead_reckoning, parse_velocity_validity
from rov_control import Controller, ControlError, SessionLogger, State, install_signal_handlers


def finite(value: Any, name: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ControlError(f"{name} 必须是数字") from exc
    if not math.isfinite(number):
        raise ControlError(f"{name} 必须是有限数")
    return number


def mat_vec(matrix: Sequence[float], vector: Sequence[float]) -> Tuple[float, float, float]:
    if len(matrix) != 9:
        raise ControlError("dvl_to_task_ned_rotation 必须包含 9 个元素")
    return (
        matrix[0] * vector[0] + matrix[1] * vector[1] + matrix[2] * vector[2],
        matrix[3] * vector[0] + matrix[4] * vector[1] + matrix[5] * vector[2],
        matrix[6] * vector[0] + matrix[7] * vector[1] + matrix[8] * vector[2],
    )


def wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def dvl_yaw_rad(yaw_deg: float, yaw_offset_rad: float, yaw_sign: float) -> float:
    """A50 航位推算 yaw 是度，主机控制内部统一使用弧度。"""
    return wrap_pi(yaw_sign * math.radians(yaw_deg) + yaw_offset_rad)


def circular_mean(angles: Sequence[float]) -> float:
    if not angles:
        raise ControlError("没有可用航向样本")
    sine = sum(math.sin(angle) for angle in angles)
    cosine = sum(math.cos(angle) for angle in angles)
    if abs(sine) < 1e-12 and abs(cosine) < 1e-12:
        raise ControlError("航向样本无法确定平均方向")
    return math.atan2(sine, cosine)


def load_config(path: Path, require_transform: bool = True) -> Dict[str, Any]:
    try:
        config = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ControlError(f"读取配置失败：{exc}") from exc
    if not isinstance(config, dict):
        raise ControlError("配置根对象必须是 JSON 对象")
    if config.get("task_frame", "local_at_dvl_reset") != "local_at_dvl_reset":
        raise ControlError("主机侧外环当前只支持 local_at_dvl_reset 任务坐标")
    if require_transform and not config.get("frame_transform_verified", False):
        raise ControlError("未核验 DVL→任务 NED 安装矩阵；请先填写 frame_transform_verified=true")
    matrix = [finite(value, "dvl_to_task_ned_rotation") for value in config.get("dvl_to_task_ned_rotation", [])]
    if len(matrix) != 9:
        raise ControlError("dvl_to_task_ned_rotation 必须包含 9 个元素")
    config["dvl_to_task_ned_rotation"] = matrix
    config["yaw_offset_rad"] = finite(config.get("yaw_offset_rad", 0.0), "yaw_offset_rad")
    config["yaw_sign"] = finite(config.get("yaw_sign", 1.0), "yaw_sign")
    if abs(config["yaw_sign"]) != 1.0:
        raise ControlError("yaw_sign 必须是 1 或 -1")
    for key in (
        "dvl_timeout_s",
        "max_position_std_m",
        "position_jump_threshold_m",
        "origin_sample_duration_s",
        "origin_max_speed_m_s",
        "horizontal_kp_level_per_m",
        "horizontal_kd_level_per_m_s",
        "vertical_kp_level_per_m",
        "vertical_kd_level_per_m_s",
        "yaw_kp_level_per_rad",
        "arrival_horizontal_tolerance_m",
        "arrival_vertical_tolerance_m",
        "arrival_speed_threshold_m_s",
        "arrival_hold_s",
        "task_timeout_s",
    ):
        config[key] = finite(config[key], key)
        if config[key] <= 0:
            raise ControlError(f"{key} 必须是正数")
    config["max_horizontal_level"] = int(config["max_horizontal_level"])
    config["max_vertical_offset"] = int(config["max_vertical_offset"])
    if not 0 <= config["max_horizontal_level"] <= 1000 or not 0 <= config["max_vertical_offset"] <= 500:
        raise ControlError("幅值上限超出 MANUAL_CONTROL 协议范围")
    config["origin_file"] = str(config.get("origin_file", "task_origin.json"))
    if not config["origin_file"].strip():
        raise ControlError("origin_file 不能为空")
    return config


def load_origin_state(path: Path, config: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    """读取固定任务原点；文件不存在表示第一次建立任务坐标。"""
    if not path.exists():
        return None
    try:
        state = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ControlError(f"读取固定任务原点失败：{exc}") from exc
    if not isinstance(state, dict):
        raise ControlError("固定任务原点文件必须是 JSON 对象")
    if state.get("task_frame") != config["task_frame"]:
        raise ControlError("固定任务原点的 task_frame 与当前配置不一致")
    saved_matrix = state.get("dvl_to_task_ned_rotation")
    if saved_matrix != config["dvl_to_task_ned_rotation"]:
        raise ControlError("固定任务原点的安装矩阵与当前配置不一致；请删除原点文件后重新建立")
    try:
        origin = tuple(finite(value, "origin") for value in state["origin"])
        if len(origin) != 3:
            raise ControlError("固定任务原点必须包含 3 个坐标")
        yaw_target = finite(state["yaw_target_rad"], "yaw_target_rad")
    except (KeyError, TypeError) as exc:
        raise ControlError(f"固定任务原点字段缺失：{exc}") from exc
    return {
        "origin": origin,
        "yaw_target_rad": yaw_target,
        "created_unix": state.get("created_unix"),
    }


def save_origin_state(path: Path, config: Dict[str, Any], origin: Tuple[float, float, float], yaw_target: float) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    state = {
        "version": 1,
        "task_frame": config["task_frame"],
        "dvl_to_task_ned_rotation": config["dvl_to_task_ned_rotation"],
        "yaw_target_rad": yaw_target,
        "origin": list(origin),
        "created_unix": time.time(),
    }
    temporary = path.with_name(path.name + ".tmp")
    try:
        temporary.write_text(json.dumps(state, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        temporary.replace(path)
    except OSError as exc:
        raise ControlError(f"保存固定任务原点失败：{exc}") from exc


def wait_for_valid_sample(feed: "DvlFeed", config: Dict[str, Any], timeout: float) -> Tuple[float, Tuple[float, float, float], Tuple[float, float, float], float]:
    deadline = time.monotonic() + timeout
    last_error = "尚未收到 DVL 航位推算"
    while time.monotonic() < deadline:
        try:
            return feed.current(config["dvl_timeout_s"], config["max_position_std_m"])
        except ControlError as exc:
            last_error = str(exc)
            time.sleep(0.01)
    raise ControlError(f"等待有效 DVL 数据超时：{last_error}")


class DvlFeed:
    def __init__(self, host: str, port: int, matrix: Sequence[float], yaw_offset: float, yaw_sign: float):
        self.host = host
        self.port = port
        self.matrix = matrix
        self.yaw_offset = yaw_offset
        self.yaw_sign = yaw_sign
        self.sock: Optional[socket.socket] = None
        self.buffer = ""
        self.velocity_valid = False
        self.beams_valid = False
        self.latest_velocity: Optional[Tuple[float, Tuple[float, float, float]]] = None
        # Preserve A50 acquisition timestamps for the independent force executor.
        # Existing goto uses the same velocity/pose behavior as before.
        self.latest_velocity_metadata: Optional[Dict[str, Any]] = None
        self.latest: Optional[Tuple[float, DvlSample, Tuple[float, float, float], float]] = None

    def connect(self) -> None:
        self.sock = socket.create_connection((self.host, self.port), timeout=5.0)
        self.sock.setblocking(False)

    def poll(self, *, max_reports=None, max_line_bytes=None, max_buffer_bytes=None) -> None:
        if self.sock is None:
            self.connect()
        assert self.sock is not None
        while True:
            try:
                chunk = self.sock.recv(8192)
            except (BlockingIOError, socket.timeout):
                break
            if not chunk:
                raise ControlError("DVL TCP 连接断开")
            self.buffer += chunk.decode("utf-8", errors="replace")
            if max_buffer_bytes is not None and len(self.buffer) > max_buffer_bytes:
                raise ControlError("DVL JSON line buffer exceeded receive budget")
        reports = 0
        while "\n" in self.buffer:
            # Optional limits are used by direct execution only. Legacy callers
            # retain the original unlimited parsing behavior. Fail on overload
            # instead of re-dating queued reports on subsequent polls.
            if max_reports is not None and reports >= max_reports:
                raise ControlError("DVL JSON report budget exceeded")
            line, _, self.buffer = self.buffer.partition("\n")
            reports += 1
            if max_line_bytes is not None and len(line) > max_line_bytes:
                raise ControlError("DVL JSON line exceeded parse budget")
            if not line.strip():
                continue
            try:
                data = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ControlError(f"DVL JSON 无法解析：{exc}") from exc
            if not isinstance(data, dict):
                raise ControlError("DVL JSON 报文不是对象")
            if "time" in data and "velocity_valid" in data:
                self.velocity_valid, self.beams_valid = parse_velocity_validity(data)
                self.latest_velocity_metadata = {key: data.get(key) for key in
                    ("time_of_validity", "time_of_transmission")}
                try:
                    velocity = tuple(float(data[key]) for key in ("vx", "vy", "vz"))
                except (KeyError, TypeError, ValueError) as exc:
                    self.velocity_valid = False
                    raise ControlError(f"DVL 速度报告缺少有效 vx/vy/vz：{exc}") from exc
                if not all(math.isfinite(value) for value in velocity):
                    self.velocity_valid = False
                else:
                    self.latest_velocity = (time.monotonic(), mat_vec(self.matrix, velocity))
                continue
            if "ts" not in data:
                continue
            sample = parse_dead_reckoning(data)
            position = mat_vec(self.matrix, sample.position)
            # A50 JSON 的姿态角单位是度；控制计算统一使用弧度。
            yaw = dvl_yaw_rad(sample.attitude[2], self.yaw_offset, self.yaw_sign)
            self.latest = (time.monotonic(), sample, position, yaw)

    def close(self) -> None:
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    def current(self, timeout: float, max_position_std: Optional[float] = None) -> Tuple[float, Tuple[float, float, float], Tuple[float, float, float], float]:
        self.poll()
        if self.latest is None:
            raise ControlError("尚未收到 DVL 航位推算")
        timestamp, sample, position, yaw = self.latest
        if time.monotonic() - timestamp > timeout:
            raise ControlError("DVL 数据过期")
        if sample.status != 0:
            raise ControlError(f"DVL 航位推算状态异常 status={sample.status}")
        if max_position_std is not None and sample.position_std > max_position_std:
            raise ControlError(
                f"DVL 航位推算标准差 {sample.position_std:.3f}m 超过上限 {max_position_std:.3f}m"
            )
        if not self.velocity_valid or not self.beams_valid or self.latest_velocity is None:
            raise ControlError("DVL 速度/波束有效性不足")
        velocity_timestamp, velocity = self.latest_velocity
        if time.monotonic() - velocity_timestamp > timeout:
            raise ControlError("DVL 速度报告过期")
        return timestamp, position, velocity, yaw


def body_error(error_ned: Tuple[float, float, float], velocity_ned: Tuple[float, float, float], yaw: float) -> Tuple[float, float, float, float]:
    c, s = math.cos(yaw), math.sin(yaw)
    forward = c * error_ned[0] + s * error_ned[1]
    right = -s * error_ned[0] + c * error_ned[1]
    forward_v = c * velocity_ned[0] + s * velocity_ned[1]
    right_v = -s * velocity_ned[0] + c * velocity_ned[1]
    return forward, right, forward_v, right_v


def clamp(value: float, limit: int) -> int:
    return int(max(-limit, min(limit, round(value))))


def compute_manual_axes(
    error_ned: Tuple[float, float, float],
    velocity_ned: Tuple[float, float, float],
    yaw: float,
    yaw_target: float,
    config: Dict[str, Any],
) -> Tuple[int, int, int, int]:
    """把固定 NED 误差转换为 MANUAL_CONTROL 归一化轴输入。"""
    # task_frame=local_at_dvl_reset 时，任务轴固定在复位瞬间；必须使用
    # 相对复位航向把任务误差转换到当前机体轴。直接使用绝对 yaw 会让
    # +X 目标混入横移、+Y 目标混入前后运动。
    body_yaw = wrap_pi(yaw - yaw_target)
    forward, right, forward_v, right_v = body_error(error_ned, velocity_ned, body_yaw)
    x = clamp(
        config["horizontal_kp_level_per_m"] * forward
        - config["horizontal_kd_level_per_m_s"] * forward_v,
        config["max_horizontal_level"],
    )
    y = clamp(
        config["horizontal_kp_level_per_m"] * right
        - config["horizontal_kd_level_per_m_s"] * right_v,
        config["max_horizontal_level"],
    )
    # NED +Z 向下，而 MANUAL_CONTROL z 偏移为正表示上浮，故取反。
    z_offset = clamp(
        config["vertical_kp_level_per_m"] * error_ned[2]
        - config["vertical_kd_level_per_m_s"] * velocity_ned[2],
        config["max_vertical_offset"],
    )
    yaw_error = wrap_pi(yaw_target - yaw)
    r = clamp(config["yaw_kp_level_per_rad"] * yaw_error, config["max_horizontal_level"])
    return x, y, 500 - z_offset, r


def run_host_goto(controller: Controller, feed: DvlFeed, config: Dict[str, Any], target: Tuple[float, float, float]) -> None:
    controller.connect()
    try:
        if controller.vehicle.armed:
            raise ControlError("启动时飞控已解锁，拒绝自动接管")
        feed.connect()
        origin_path = Path(config["origin_file"])
        saved_origin = load_origin_state(origin_path, config)
        if saved_origin is None:
            deadline = time.monotonic() + config["origin_sample_duration_s"]
            samples = []
            last_feed_error = "尚未收到 DVL 航位推算"
            while time.monotonic() < deadline:
                try:
                    _, position, velocity, yaw = feed.current(config["dvl_timeout_s"], config["max_position_std_m"])
                except ControlError as exc:
                    # 新 TCP 连接通常需要等待第一组 velocity/position 交替报文。
                    last_feed_error = str(exc)
                    controller.service_io()
                    time.sleep(0.01)
                    continue
                speed = math.sqrt(sum(value * value for value in velocity))
                if speed > config["origin_max_speed_m_s"]:
                    raise ControlError(f"DVL 原点采样时速度 {speed:.3f}m/s 过大")
                samples.append((position, yaw))
                controller.service_io()
                time.sleep(0.01)
            if len(samples) < 3:
                raise ControlError(f"DVL 原点样本不足：{last_feed_error}")
            origin = tuple(sum(item[0][axis] for item in samples) / len(samples) for axis in range(3))
            yaw_target = circular_mean([item[1] for item in samples])
            save_origin_state(origin_path, config, origin, yaw_target)
            origin_source = "created"
        else:
            origin = saved_origin["origin"]
            yaw_target = saved_origin["yaw_target_rad"]
            wait_for_valid_sample(feed, config, config["origin_sample_duration_s"])
            origin_source = "loaded"
        controller.record_event(
            "host_goto_origin",
            origin=origin,
            yaw_target=yaw_target,
            target_relative=target,
            origin_source=origin_source,
            origin_file=str(origin_path),
        )

        # 固定原点模式下，如果当前已经在目标容差内，直接结束，不解锁、不发送非中立输入。
        _, current, current_velocity, _ = wait_for_valid_sample(feed, config, config["origin_sample_duration_s"])
        current_error = tuple(origin[axis] + target[axis] - current[axis] for axis in range(3))
        current_horizontal = math.hypot(current_error[0], current_error[1])
        current_vertical = abs(current_error[2])
        current_speed = math.sqrt(sum(value * value for value in current_velocity))
        if (
            current_horizontal <= config["arrival_horizontal_tolerance_m"]
            and current_vertical <= config["arrival_vertical_tolerance_m"]
            and current_speed <= config["arrival_speed_threshold_m_s"]
        ):
            controller.record_event(
                "host_goto_already_at_target",
                error_ned=current_error,
                horizontal_error_m=current_horizontal,
                vertical_error_m=current_vertical,
                speed_m_s=current_speed,
            )
            return

        controller.request_mode("MANUAL")
        controller.set_state(State.NEUTRAL)
        controller.send_manual((0, 0, 500, 0))
        controller.request_arm()
        controller.action_name = "host-goto"
        controller.set_state(State.MOVING)
        end = time.monotonic() + config["task_timeout_s"]
        hold_since: Optional[float] = None
        previous_position: Optional[Tuple[float, float, float]] = None
        while time.monotonic() < end and not controller.stop_requested:
            _, position, velocity, yaw = feed.current(config["dvl_timeout_s"], config["max_position_std_m"])
            controller.service_io()
            controller.check_runtime_safety()
            if previous_position is not None:
                jump = math.sqrt(sum((position[index] - previous_position[index]) ** 2 for index in range(3)))
                if jump > config["position_jump_threshold_m"]:
                    raise ControlError(f"DVL 航位推算位置突跳 {jump:.3f}m，终止主机外环")
            previous_position = position
            error_ned = tuple(origin[axis] + target[axis] - position[axis] for axis in range(3))
            axes = compute_manual_axes(error_ned, velocity, yaw, yaw_target, config)
            body_yaw = wrap_pi(yaw - yaw_target)
            horizontal = math.hypot(error_ned[0], error_ned[1])
            vertical = abs(error_ned[2])
            speed = math.sqrt(sum(value * value for value in velocity))
            controller.record_event(
                "host_goto_sample",
                position=position,
                velocity=velocity,
                yaw_rad=yaw,
                yaw_target_rad=yaw_target,
                body_yaw_rad=body_yaw,
                error_ned=error_ned,
                horizontal_error_m=horizontal,
                vertical_error_m=vertical,
                speed_m_s=speed,
                axes=axes,
            )
            if horizontal <= config["arrival_horizontal_tolerance_m"] and vertical <= config["arrival_vertical_tolerance_m"] and speed <= config["arrival_speed_threshold_m_s"]:
                if hold_since is None:
                    hold_since = time.monotonic()
                elif time.monotonic() - hold_since >= config["arrival_hold_s"]:
                    controller.record_event("host_goto_arrived")
                    return
            else:
                hold_since = None
            controller.send_manual(axes)
            time.sleep(0.05)
        if controller.stop_requested:
            raise ControlError(controller.termination_reason or "收到终止请求")
        raise ControlError("主机侧 DVL 目标任务超时")
    finally:
        if controller.master is not None and controller.control_responsibility and not controller.disarm_request_attempted:
            controller.stop_and_disarm()
        feed.close()


def run_host_check(feed: DvlFeed, config: Dict[str, Any], duration: float = 5.0) -> int:
    """只读检查 A50 速度、波束有效性和航位推算，不连接飞控、不发送控制。"""
    feed.connect()
    deadline = time.monotonic() + duration
    try:
        while time.monotonic() < deadline:
            try:
                timestamp, position, velocity, yaw = feed.current(config["dvl_timeout_s"], None)
                sample = feed.latest[1] if feed.latest is not None else None
                quality_ok = sample is not None and sample.position_std <= config["max_position_std_m"]
                quality_text = "PASS" if quality_ok else "FAIL"
                print(
                    f"dvl_transport=OK dvl_position_quality={quality_text} "
                    f"position=({position[0]:.3f},{position[1]:.3f},{position[2]:.3f}) "
                    f"velocity=({velocity[0]:.3f},{velocity[1]:.3f},{velocity[2]:.3f}) "
                    f"yaw_rad={yaw:.3f} age={time.monotonic() - timestamp:.3f}s "
                    f"status={sample.status if sample is not None else 'unknown'} "
                    f"position_std_m={sample.position_std if sample is not None else 'unknown'} "
                    f"position_std_limit_m={config['max_position_std_m']:.3f} "
                    f"velocity_valid={feed.velocity_valid} beams_valid={feed.beams_valid}"
                )
                return 0 if quality_ok else 2
            except ControlError:
                time.sleep(0.02)
        raise ControlError("在检查超时内没有收到有效 A50 速度/航位推算组合")
    finally:
        feed.close()


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="旧版 ArduSub 的主机侧 DVL 外环 goto")
    parser.add_argument("--config", type=Path, default=Path("host_goto_config.json"))
    parser.add_argument("--x", type=float)
    parser.add_argument("--y", type=float)
    parser.add_argument("--z", type=float)
    parser.add_argument("--check", action="store_true", help="只读检查 DVL，不连接飞控、不解锁、不发送控制")
    parser.add_argument("--connection")
    args = parser.parse_args(argv)
    try:
        config = load_config(args.config, require_transform=not args.check)
    except ControlError as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 2
    connection = args.connection or config.get("connection")
    if not connection:
        parser.error("缺少 connection")
    logger = SessionLogger(Path(config.get("log_dir", "logs")), action="host-goto")
    controller = Controller(
        connection,
        int(config.get("baud", 115200)),
        int(config.get("source_system", 255)),
        int(config.get("source_component", 190)),
        config.get("target_system"),
        config.get("target_component"),
        logger,
        float(config.get("connect_timeout", 15.0)),
        float(config.get("confirm_timeout", 5.0)),
        float(config.get("heartbeat_timeout", 3.0)),
    )
    feed = DvlFeed(config["dvl_ip"], int(config["dvl_port"]), config["dvl_to_task_ned_rotation"], config["yaw_offset_rad"], config["yaw_sign"])
    install_signal_handlers(controller)
    try:
        if args.check:
            return run_host_check(feed, config)
        if any(value is None for value in (args.x, args.y, args.z)):
            parser.error("执行主机侧 goto 时必须同时提供 --x、--y、--z")
        run_host_goto(controller, feed, config, (finite(args.x, "x"), finite(args.y, "y"), finite(args.z, "z")))
        return 0 if controller.disarm_confirmed else 2
    except (ControlError, OSError, ValueError) as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 1
    finally:
        if not args.check and controller.control_responsibility:
            if controller.disarm_confirmed:
                print("最终状态：已确认上锁。")
            elif controller.disarm_request_sent:
                print("最终状态：已请求上锁但未确认；请按日志和现场状态核实。", file=sys.stderr)
            else:
                print("最终状态：链路断开或清理未完成，最终解锁状态未知。", file=sys.stderr)
        controller.close()
        logger.close()


if __name__ == "__main__":
    raise SystemExit(main())
