#!/usr/bin/env python3
"""BlueROV2/ArduSub 基础通信与低幅度单轴运动测试工具。

程序只发送 MANUAL_CONTROL 给飞控，让 ArduSub 继续完成混控和推进器分配。
它不直接写六个电机输出，也不包含定位、轨迹或动力学算法。
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import enum
import json
import math
import signal
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

try:
    from pymavlink import mavutil
except ImportError as exc:  # pragma: no cover - exercised by installation, not unit tests
    raise SystemExit("未找到 pymavlink，请先执行 python -m pip install -r requirements.txt") from exc


CONTROL_HZ = 20.0
HEARTBEAT_HZ = 1.0
SUMMARY_HZ = 1.0
DEFAULT_CONNECT_TIMEOUT = 15.0
DEFAULT_CONFIRM_TIMEOUT = 5.0
DEFAULT_HEARTBEAT_TIMEOUT = 3.0
DEFAULT_ACTION_DURATION = 1.0
DEFAULT_STOP_DURATION = 1.0
DEFAULT_MAX_DURATION = 5.0
# 这是程序对 MANUAL_CONTROL 归一化轴的默认软件上限，不是 PWM/RPM。
# 仍低于协议的水平/偏航 1000、垂直偏移 500；现场可用命令行进一步降低。
DEFAULT_HORIZONTAL_CAP = 400
DEFAULT_VERTICAL_CAP = 200
MAX_ABSOLUTE_DURATION = 30.0
RECV_BUDGET_MESSAGES = 100
RECV_BUDGET_SECONDS = 0.010

NEUTRAL = (0, 0, 500, 0)
ACTION_NAMES = (
    "forward",
    "backward",
    "left",
    "right",
    "yaw-left",
    "yaw-right",
    "up",
    "down",
)


class State(enum.Enum):
    DISCONNECTED = "DISCONNECTED"
    CONNECTED = "CONNECTED"
    MODE_CONFIRMING = "MODE_CONFIRMING"
    NEUTRAL = "NEUTRAL"
    ARM_CONFIRMING = "ARM_CONFIRMING"
    MOVING = "MOVING"
    STOPPING = "STOPPING"
    DISARM_CONFIRMING = "DISARM_CONFIRMING"
    DONE = "DONE"
    FAULT = "FAULT"


class ControlError(RuntimeError):
    """可显示给操作者的通信、状态或安全错误。"""


class LinkError(ControlError):
    pass


class ActionValidationError(ControlError):
    pass


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def finite_positive(value: float, name: str) -> float:
    if not math.isfinite(value) or value <= 0:
        raise ActionValidationError(f"{name} 必须是有限正数")
    return value


def finite_nonnegative_int(value: str) -> int:
    try:
        number = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("必须是非负整数") from exc
    if not math.isfinite(number) or number < 0 or number != int(number):
        raise argparse.ArgumentTypeError("必须是有限非负整数")
    return int(number)


def bounded_int(minimum: int, maximum: int, label: str) -> Callable[[str], int]:
    def parse(value: str) -> int:
        try:
            result = int(value, 0)
        except ValueError as exc:
            raise argparse.ArgumentTypeError(f"{label} 必须是整数") from exc
        if not minimum <= result <= maximum:
            raise argparse.ArgumentTypeError(f"{label} 必须在 {minimum}..{maximum} 范围内")
        return result

    return parse


def positive_float(value: str) -> float:
    try:
        result = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("必须是数字") from exc
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError("必须是有限正数")
    return result


def nonnegative_float(value: str) -> float:
    try:
        result = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("必须是数字") from exc
    if not math.isfinite(result) or result < 0:
        raise argparse.ArgumentTypeError("必须是有限非负数")
    return result


def source_ids(msg: Any) -> Tuple[Optional[int], Optional[int]]:
    """尽量兼容 pymavlink 消息和单元测试 fake message 的来源字段。"""
    try:
        return msg.get_srcSystem(), msg.get_srcComponent()
    except (AttributeError, TypeError):
        header = getattr(msg, "_header", None)
        return getattr(header, "srcSystem", None), getattr(header, "srcComponent", None)


def message_dict(msg: Any) -> Dict[str, Any]:
    try:
        data = msg.to_dict()
    except (AttributeError, TypeError):
        data = {}
        for key, value in vars(msg).items():
            if not key.startswith("_"):
                data[key] = value
    return json_safe(data)


def json_safe(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    if isinstance(value, (str, int, bool)) or value is None:
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    try:
        return float(value)
    except (TypeError, ValueError):
        return str(value)


def result_name(result: Optional[int]) -> str:
    if result is None:
        return "UNKNOWN"
    try:
        return mavutil.mavlink.enums["MAV_RESULT"][int(result)].name
    except (KeyError, AttributeError, TypeError, ValueError):
        names = {
            0: "ACCEPTED",
            1: "TEMPORARILY_REJECTED",
            2: "DENIED",
            3: "UNSUPPORTED",
            4: "FAILED",
            5: "IN_PROGRESS",
            6: "CANCELLED",
        }
        return names.get(int(result), f"UNKNOWN({result})")


def command_name(command: Optional[int]) -> str:
    if command is None:
        return "UNKNOWN"
    try:
        return mavutil.mavlink.enums["MAV_CMD"][int(command)].name
    except (KeyError, AttributeError, TypeError, ValueError):
        return str(command)


def ack_is_nonfatal(result: Optional[int]) -> bool:
    """兼容缺少新常量的旧 pymavlink；最终仍以状态心跳为主证据。"""
    accepted = getattr(mavutil.mavlink, "MAV_RESULT_ACCEPTED", 0)
    in_progress = getattr(mavutil.mavlink, "MAV_RESULT_IN_PROGRESS", 5)
    return result in (accepted, in_progress)


def map_action(action: str, level: int) -> Tuple[int, int, int, int]:
    """将动作映射到 ArduSub 的 MANUAL_CONTROL 轴。

    x 是前后、y 是横移、z 是升降（500 中立）、r 是偏航。
    y/r 的正负方向需在现场以很小幅度核验，见 README。
    """
    if action not in ACTION_NAMES:
        raise ActionValidationError(f"非法动作 {action!r}，可选值：{', '.join(ACTION_NAMES)}")
    if not isinstance(level, int) or level < 0:
        raise ActionValidationError("level 必须是非负整数")
    if action in ("up", "down"):
        if level > 500:
            raise ActionValidationError("垂直 level 必须在 0..500")
        return (0, 0, 500 + level if action == "up" else 500 - level, 0)
    if level > 1000:
        raise ActionValidationError("水平/偏航 level 必须在 0..1000")
    if action == "forward":
        return (level, 0, 500, 0)
    if action == "backward":
        return (-level, 0, 500, 0)
    if action == "left":
        return (0, -level, 500, 0)
    if action == "right":
        return (0, level, 500, 0)
    if action == "yaw-left":
        return (0, 0, 500, -level)
    return (0, 0, 500, level)


@dataclasses.dataclass
class VehicleState:
    target_system: Optional[int] = None
    target_component: Optional[int] = None
    autopilot: Optional[int] = None
    vehicle_type: Optional[int] = None
    last_heartbeat_mono: Optional[float] = None
    mode: Optional[str] = None
    custom_mode: Optional[int] = None
    armed: Optional[bool] = None
    attitude: Dict[str, Optional[float]] = dataclasses.field(default_factory=dict)
    sys_status: Dict[str, Optional[float]] = dataclasses.field(default_factory=dict)
    battery: Dict[str, Any] = dataclasses.field(default_factory=dict)
    local_position: Dict[str, Optional[float]] = dataclasses.field(default_factory=dict)
    last_local_position_mono: Optional[float] = None
    ekf_status: Dict[str, Any] = dataclasses.field(default_factory=dict)
    last_ekf_status_mono: Optional[float] = None
    parameters: Dict[str, float] = dataclasses.field(default_factory=dict)
    pressure: Dict[str, Any] = dataclasses.field(default_factory=dict)
    autopilot_version: Dict[str, Any] = dataclasses.field(default_factory=dict)
    last_statustext: Optional[str] = None
    last_status_severity: Optional[int] = None
    last_ack: Dict[str, Any] = dataclasses.field(default_factory=dict)
    message_count: int = 0

    def heartbeat_fresh(self, now: float, timeout: float) -> bool:
        return self.last_heartbeat_mono is not None and now - self.last_heartbeat_mono <= timeout


class SessionLogger:
    """JSONL 保存原始记录，CSV 保存便于查看的状态摘要。"""

    CSV_FIELDS = [
        "utc_time",
        "monotonic_s",
        "record_kind",
        "message_type",
        "src_system",
        "src_component",
        "flight_timestamp",
        "state",
        "action",
        "x",
        "y",
        "z",
        "r",
        "mode",
        "armed",
        "roll_rad",
        "pitch_rad",
        "yaw_rad",
        "rollspeed_rad_s",
        "pitchspeed_rad_s",
        "yawspeed_rad_s",
        "voltage_battery_mv",
        "current_battery_ca",
        "battery_remaining_pct",
        "local_x_m",
        "local_y_m",
        "local_z_m_raw",
        "depth_note",
        "event",
        "result",
        "reason",
    ]

    def __init__(self, directory: Path, action: Optional[str] = None):
        directory.mkdir(parents=True, exist_ok=True)
        # 包含微秒，避免同一秒内的诊断/动作覆盖前一份证据。
        stem = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        suffix = f"_{action}" if action else "_session"
        self.jsonl_path = directory / f"{stem}{suffix}.jsonl"
        self.csv_path = directory / f"{stem}{suffix}.csv"
        self.started_mono = time.monotonic()
        self.jsonl = self.jsonl_path.open("w", encoding="utf-8", buffering=1)
        self.csv_file = self.csv_path.open("w", encoding="utf-8", newline="", buffering=1)
        self.csv = csv.DictWriter(self.csv_file, fieldnames=self.CSV_FIELDS, extrasaction="ignore")
        self.csv.writeheader()
        self.closed = False

    def close(self) -> None:
        if not self.closed:
            self.jsonl.close()
            self.csv_file.close()
            self.closed = True

    def _base(self, record_kind: str) -> Dict[str, Any]:
        return {
            "utc_time": utc_now(),
            "monotonic_s": round(time.monotonic() - self.started_mono, 6),
            "record_kind": record_kind,
        }

    def write_log(self, record_kind: str, **data: Any) -> None:
        record = self._base(record_kind)
        record.update(json_safe(data))
        self.jsonl.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")
        row = {field: record.get(field) for field in self.CSV_FIELDS}
        if record_kind == "message":
            row["event"] = record.get("message_type")
        self.csv.writerow(row)

    def message(self, msg: Any, state: State, snapshot: Dict[str, Any]) -> None:
        src_system, src_component = source_ids(msg)
        raw = message_dict(msg)
        flight_timestamp = next(
            (raw.get(key) for key in ("time_boot_ms", "time_usec", "time_ms") if raw.get(key) is not None),
            None,
        )
        self.write_log(
            "message",
            message_type=msg.get_type(),
            src_system=src_system,
            src_component=src_component,
            flight_timestamp=flight_timestamp,
            state=state.value,
            raw=raw,
            **snapshot,
        )


class Controller:
    """单一事件循环：接收、定时发送、状态检查和超时都在这里运行。"""

    def __init__(
        self,
        connection: str,
        baud: int,
        source_system: int,
        source_component: int,
        target_system: Optional[int],
        target_component: Optional[int],
        logger: SessionLogger,
        connect_timeout: float = DEFAULT_CONNECT_TIMEOUT,
        confirm_timeout: float = DEFAULT_CONFIRM_TIMEOUT,
        heartbeat_timeout: float = DEFAULT_HEARTBEAT_TIMEOUT,
    ):
        self.connection_string = connection
        self.baud = baud
        self.source_system = source_system
        self.source_component = source_component
        self.requested_target_system = target_system
        self.requested_target_component = target_component
        self.connect_timeout = finite_positive(connect_timeout, "connect-timeout")
        self.confirm_timeout = finite_positive(confirm_timeout, "confirm-timeout")
        self.heartbeat_timeout = finite_positive(heartbeat_timeout, "heartbeat-timeout")
        self.logger = logger
        self.master: Any = None
        self.state = State.DISCONNECTED
        self.vehicle = VehicleState()
        self.candidates: Dict[Tuple[int, int], Dict[str, Any]] = {}
        self.ack_events: List[Dict[str, Any]] = []
        self.stop_requested = False
        self.termination_reason: Optional[str] = None
        self.arm_request_sent = False
        self.arm_confirmed = False
        self.disarm_request_attempted = False
        self.disarm_request_sent = False
        self.disarm_confirmed = False
        self.control_responsibility = False
        self.action_name: Optional[str] = None
        self.command_axes = NEUTRAL
        self._next_heartbeat = 0.0
        self._next_summary = 0.0
        self._next_control_send = 0.0
        self._last_send_mono: Optional[float] = None
        self._last_message_mono: Optional[float] = None

    @property
    def target_system(self) -> Optional[int]:
        return self.vehicle.target_system

    @property
    def target_component(self) -> Optional[int]:
        return self.vehicle.target_component

    def snapshot(self) -> Dict[str, Any]:
        attitude = self.vehicle.attitude
        sys_status = self.vehicle.sys_status
        battery = self.vehicle.battery
        pos = self.vehicle.local_position
        return {
            "autopilot": self.vehicle.autopilot,
            "vehicle_type": self.vehicle.vehicle_type,
            "action": self.action_name,
            "x": self.command_axes[0],
            "y": self.command_axes[1],
            "z": self.command_axes[2],
            "r": self.command_axes[3],
            "mode": self.vehicle.mode,
            "armed": self.vehicle.armed,
            "roll_rad": attitude.get("roll_rad"),
            "pitch_rad": attitude.get("pitch_rad"),
            "yaw_rad": attitude.get("yaw_rad"),
            "rollspeed_rad_s": attitude.get("rollspeed_rad_s"),
            "pitchspeed_rad_s": attitude.get("pitchspeed_rad_s"),
            "yawspeed_rad_s": attitude.get("yawspeed_rad_s"),
            "voltage_battery_mv": sys_status.get("voltage_battery_mv", battery.get("voltage_battery_mv")),
            "current_battery_ca": sys_status.get("current_battery_ca", battery.get("current_battery_ca")),
            "battery_remaining_pct": sys_status.get("battery_remaining_pct", battery.get("battery_remaining_pct")),
            "local_x_m": pos.get("x_m"),
            "local_y_m": pos.get("y_m"),
            "local_z_m_raw": pos.get("z_m"),
            "depth_note": "LOCAL_POSITION_NED.z 原始值，深度来源/符号/零点待核实" if pos else None,
        }

    def record_event(self, event: str, reason: Optional[str] = None, **data: Any) -> None:
        payload = self.snapshot()
        payload.update(data)
        payload.update({"state": self.state.value, "event": event, "reason": reason})
        self.logger.write_log("event", **payload)

    def set_state(self, state: State, reason: Optional[str] = None) -> None:
        self.state = state
        self.record_event("state", reason=reason, new_state=state.value)

    def _send_heartbeat(self) -> None:
        if self.master is None:
            return
        self.master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )
        self.record_event("send_heartbeat")

    def send_heartbeat(self, now: Optional[float] = None, force: bool = False) -> None:
        now = time.monotonic() if now is None else now
        if force or now >= self._next_heartbeat:
            try:
                self._send_heartbeat()
            except Exception as exc:
                self.record_event("heartbeat_send_error", reason=str(exc))
                raise LinkError(f"发送 GCS 心跳失败：{exc}") from exc
            self._next_heartbeat = now + 1.0 / HEARTBEAT_HZ

    def _emit_summary(self, now: float) -> None:
        if now < self._next_summary:
            return
        self.record_event("summary", heartbeat_age_s=(None if self.vehicle.last_heartbeat_mono is None else now - self.vehicle.last_heartbeat_mono))
        self._next_summary = now + 1.0 / SUMMARY_HZ

    def process_message(self, msg: Any) -> None:
        """更新遥测状态并记录原始消息；不会把缺失字段伪装成 0。"""
        msg_type = msg.get_type()
        src_system, src_component = source_ids(msg)
        now = time.monotonic()
        self._last_message_mono = now
        self.vehicle.message_count += 1
        if msg_type == "HEARTBEAT":
            autopilot = getattr(msg, "autopilot", None)
            is_autopilot = autopilot is not None and autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID
            if is_autopilot and src_system is not None and src_component is not None:
                self.candidates[(int(src_system), int(src_component))] = {
                    "autopilot": autopilot,
                    "type": getattr(msg, "type", None),
                    "last_seen": now,
                    "armed": bool(int(getattr(msg, "base_mode", 0)) & int(mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)),
                    "custom_mode": getattr(msg, "custom_mode", None),
                    "mode": self._decode_mode(msg),
                }
            if self.target_system is not None and (src_system, src_component) == (self.target_system, self.target_component):
                self._apply_heartbeat(msg, now)
        elif self.target_system is not None and (src_system, src_component) == (self.target_system, self.target_component):
            if msg_type == "ATTITUDE":
                self.vehicle.attitude = {
                    "roll_rad": getattr(msg, "roll", None),
                    "pitch_rad": getattr(msg, "pitch", None),
                    "yaw_rad": getattr(msg, "yaw", None),
                    "rollspeed_rad_s": getattr(msg, "rollspeed", None),
                    "pitchspeed_rad_s": getattr(msg, "pitchspeed", None),
                    "yawspeed_rad_s": getattr(msg, "yawspeed", None),
                }
            elif msg_type == "SYS_STATUS":
                self.vehicle.sys_status = {
                    "voltage_battery_mv": self._valid_number(getattr(msg, "voltage_battery", None), 65535),
                    "current_battery_ca": self._valid_number(getattr(msg, "current_battery", None), -1),
                    "battery_remaining_pct": self._valid_number(getattr(msg, "battery_remaining", None), -1),
                    "load_permil": self._valid_number(getattr(msg, "load", None), -1),
                }
            elif msg_type == "BATTERY_STATUS":
                voltages = list(getattr(msg, "voltages", []) or [])
                self.vehicle.battery = {
                    "id": getattr(msg, "id", None),
                    "voltage_battery_mv": next((v for v in voltages if v not in (None, 65535)), None),
                    "voltages_mv": [v if v not in (None, 65535) else None for v in voltages],
                    "current_battery_ca": self._valid_number(getattr(msg, "current_battery", None), -1),
                    "battery_remaining_pct": self._valid_number(getattr(msg, "battery_remaining", None), -1),
                }
            elif msg_type == "LOCAL_POSITION_NED":
                self.vehicle.local_position = {
                    "time_boot_ms": getattr(msg, "time_boot_ms", None),
                    "x_m": getattr(msg, "x", None),
                    "y_m": getattr(msg, "y", None),
                    "z_m": getattr(msg, "z", None),
                    "vx_m_s": getattr(msg, "vx", None),
                    "vy_m_s": getattr(msg, "vy", None),
                    "vz_m_s": getattr(msg, "vz", None),
                }
                self.vehicle.last_local_position_mono = now
            elif msg_type == "EKF_STATUS_REPORT":
                self.vehicle.ekf_status = message_dict(msg)
                self.vehicle.last_ekf_status_mono = now
            elif msg_type == "PARAM_VALUE":
                name = getattr(msg, "param_id", "")
                if isinstance(name, bytes):
                    name = name.decode("utf-8", errors="replace")
                name = str(name).rstrip("\x00")
                value = getattr(msg, "param_value", None)
                if name and value is not None and math.isfinite(float(value)):
                    self.vehicle.parameters[name] = float(value)
            elif msg_type in ("SCALED_PRESSURE", "SCALED_PRESSURE2", "SCALED_PRESSURE3", "RAW_PRESSURE", "DEPTH"):
                self.vehicle.pressure = message_dict(msg)
            elif msg_type == "AUTOPILOT_VERSION":
                self.vehicle.autopilot_version = message_dict(msg)
            elif msg_type == "STATUSTEXT":
                text = getattr(msg, "text", "")
                if isinstance(text, bytes):
                    text = text.rstrip(b"\x00").decode("utf-8", errors="replace")
                self.vehicle.last_statustext = str(text).rstrip("\x00")
                self.vehicle.last_status_severity = getattr(msg, "severity", None)
                self.record_event("statustext", reason=self.vehicle.last_statustext, severity=self.vehicle.last_status_severity)
            elif msg_type == "COMMAND_ACK":
                ack = {
                    "command": getattr(msg, "command", None),
                    "command_name": command_name(getattr(msg, "command", None)),
                    "result": getattr(msg, "result", None),
                    "result_name": result_name(getattr(msg, "result", None)),
                    "progress": getattr(msg, "progress", None),
                    "result_param2": getattr(msg, "result_param2", None),
                    "target_system": getattr(msg, "target_system", None),
                    "target_component": getattr(msg, "target_component", None),
                    "mono": now,
                }
                self.vehicle.last_ack = ack
                self.ack_events.append(ack)
                self.record_event("command_ack", **ack)
                if ack["command"] == getattr(mavutil.mavlink, "MAV_CMD_SET_MESSAGE_INTERVAL", None) and not ack_is_nonfatal(ack["result"]):
                    self.record_event("message_interval_rejected", reason=ack["result_name"], ack=ack)
                if ack["command"] == getattr(mavutil.mavlink, "MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES", None) and not ack_is_nonfatal(ack["result"]):
                    self.record_event("autopilot_version_request_rejected", reason=ack["result_name"], ack=ack)
        self.logger.message(msg, self.state, self.snapshot())

    @staticmethod
    def _valid_number(value: Any, invalid: Any) -> Any:
        if value is None or value == invalid:
            return None
        return value

    @staticmethod
    def _decode_mode(msg: Any) -> Optional[str]:
        try:
            mode = mavutil.mode_string_v10(msg)
        except Exception:
            mode = None
        if mode is not None and not str(mode).startswith("Mode("):
            return str(mode)
        custom_mode = getattr(msg, "custom_mode", None)
        return str(custom_mode) if custom_mode is not None else mode

    def _apply_heartbeat(self, msg: Any, now: Optional[float] = None) -> None:
        now = time.monotonic() if now is None else now
        self.vehicle.last_heartbeat_mono = now
        self.vehicle.autopilot = getattr(msg, "autopilot", None)
        self.vehicle.vehicle_type = getattr(msg, "type", None)
        self.vehicle.armed = bool(int(getattr(msg, "base_mode", 0)) & int(mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED))
        self.vehicle.custom_mode = getattr(msg, "custom_mode", None)
        self.vehicle.mode = self._decode_mode(msg)

    def service_io(self, now: Optional[float] = None) -> int:
        """每轮只处理有限数量/时间的消息，避免遥测洪水饿死控制发送。"""
        now = time.monotonic() if now is None else now
        self.send_heartbeat(now)
        processed = 0
        started = time.monotonic()
        while processed < RECV_BUDGET_MESSAGES and time.monotonic() - started < RECV_BUDGET_SECONDS:
            try:
                msg = self.master.recv_match(blocking=False) if self.master is not None else None
            except Exception as exc:
                raise LinkError(f"接收 MAVLink 消息失败：{exc}") from exc
            if msg is None:
                break
            self.process_message(msg)
            processed += 1
        self._emit_summary(now)
        return processed

    def _select_target(self) -> None:
        candidates = list(self.candidates)
        if self.requested_target_system is not None:
            candidates = [
                item
                for item in candidates
                if item[0] == self.requested_target_system
                and (self.requested_target_component is None or item[1] == self.requested_target_component)
            ]
        elif self.requested_target_component is not None:
            candidates = [item for item in candidates if item[1] == self.requested_target_component]
        if len(candidates) != 1:
            if not candidates:
                raise LinkError("尚未识别到符合条件的飞控 HEARTBEAT；请检查端点或 target-system/component")
            formatted = ", ".join(f"{system}/{component}" for system, component in candidates)
            raise LinkError(f"发现多个候选飞控 ({formatted})，请显式指定 --target-system/--target-component")
        self.vehicle.target_system, self.vehicle.target_component = candidates[0]
        self.master.target_system = candidates[0][0]
        self.master.target_component = candidates[0][1]
        # 选择前已经消费的那一帧心跳也用于启动安全检查，避免把“已解锁”
        # 误当成未知状态而继续请求接管。
        candidate = self.candidates[candidates[0]]
        self.vehicle.last_heartbeat_mono = candidate.get("last_seen")
        self.vehicle.autopilot = candidate.get("autopilot")
        self.vehicle.vehicle_type = candidate.get("type")
        self.vehicle.armed = candidate.get("armed")
        self.vehicle.custom_mode = candidate.get("custom_mode")
        self.vehicle.mode = candidate.get("mode")
        self.record_event("target_selected", target_system=candidates[0][0], target_component=candidates[0][1])

    def connect(self) -> None:
        if self.state != State.DISCONNECTED:
            return
        try:
            self.master = mavutil.mavlink_connection(
                self.connection_string,
                baud=self.baud,
                source_system=self.source_system,
                source_component=self.source_component,
            )
        except Exception as exc:
            raise LinkError(f"创建 MAVLink 连接失败：{exc}") from exc
        self.set_state(State.CONNECTED)
        deadline = time.monotonic() + self.connect_timeout
        while time.monotonic() < deadline and not self.stop_requested:
            try:
                self.service_io()
            except LinkError:
                raise
            if self.candidates:
                self._select_target()
                # 重新处理在选择前到达的心跳，确保 last_heartbeat 不为空。
                if self.vehicle.last_heartbeat_mono is None:
                    self.service_io()
                if self.vehicle.last_heartbeat_mono is not None:
                    self.record_event("connected", target_system=self.target_system, target_component=self.target_component)
                    return
            time.sleep(0.005)
        raise LinkError("等待飞控 HEARTBEAT 超时")

    def close(self) -> None:
        if self.master is not None:
            try:
                self.master.close()
            except Exception as exc:
                self.record_event("close_error", reason=str(exc))
            self.master = None
        if self.state not in (State.DONE, State.FAULT, State.DISCONNECTED):
            self.state = State.DISCONNECTED

    def _ensure_heartbeat(self) -> None:
        if not self.vehicle.heartbeat_fresh(time.monotonic(), self.heartbeat_timeout):
            raise LinkError("飞控 HEARTBEAT 已过期，停止非中立控制")

    def _ack_since(self, command: int, since: float) -> Optional[Dict[str, Any]]:
        for ack in reversed(self.ack_events):
            if ack.get("command") == command and ack.get("mono", 0) >= since:
                return ack
        return None

    def wait_until(self, predicate: Callable[[], bool], timeout: float, tick: Optional[Callable[[], None]] = None) -> None:
        deadline = time.monotonic() + finite_positive(timeout, "timeout")
        while time.monotonic() < deadline and not self.stop_requested:
            self.service_io()
            if tick:
                tick()
            if predicate():
                return
            time.sleep(0.005)
        if self.stop_requested:
            raise ControlError(self.termination_reason or "收到终止请求")
        raise ControlError("等待状态确认超时")

    def request_mode(self, mode: str = "MANUAL") -> None:
        self._ensure_heartbeat()
        mapping = self.master.mode_mapping() or {}
        if mode not in mapping:
            raise ControlError(f"飞控未报告 {mode} 模式；可用模式：{', '.join(sorted(mapping))}")
        mode_id = mapping[mode]
        self.set_state(State.MODE_CONFIRMING)
        since = time.monotonic()
        self.master.mav.set_mode_send(
            self.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
        self.record_event("request_mode", requested_mode=mode, mode_id=mode_id)

        def tick() -> None:
            # 等待确认期间保持中立，并继续提供 GCS 心跳。
            # 如果目标已经由本轮心跳确认，GUIDED 等模式不再发送 MANUAL_CONTROL。
            if self.vehicle.mode != mode:
                self.send_neutral_if_due()

        def mode_confirmed() -> bool:
            return self.vehicle.mode == mode

        try:
            self.wait_until(mode_confirmed, self.confirm_timeout, tick=tick)
        except ControlError as exc:
            ack = self._ack_since(mavutil.mavlink.MAV_CMD_DO_SET_MODE, since)
            self.record_event("mode_confirm_failed", reason=str(exc), ack=ack)
            raise ControlError(f"MANUAL 模式未由后续 HEARTBEAT 确认：{exc}") from exc
        ack = self._ack_since(mavutil.mavlink.MAV_CMD_DO_SET_MODE, since)
        if ack and not ack_is_nonfatal(ack.get("result")):
            raise ControlError(f"模式 COMMAND_ACK 被拒绝：{ack.get('result_name')}")
        self.record_event("mode_confirmed", mode=mode, ack=ack)

    def send_manual(self, axes: Tuple[int, int, int, int]) -> None:
        if self.master is None or self.target_system is None:
            raise LinkError("尚未连接到目标飞控")
        x, y, z, r = axes
        if not (-1000 <= x <= 1000 and -1000 <= y <= 1000 and 0 <= z <= 1000 and -1000 <= r <= 1000):
            raise ActionValidationError("MANUAL_CONTROL 轴值超出协议范围")
        self.master.mav.manual_control_send(self.target_system, x, y, z, r, 0)
        self.command_axes = axes
        self._last_send_mono = time.monotonic()
        self._next_control_send = self._last_send_mono + 1.0 / CONTROL_HZ
        self.logger.write_log("tx_manual_control", state=self.state.value, **self.snapshot())

    def send_neutral_if_due(self) -> None:
        """确认等待/清理期间也按约 20 Hz 发中立，避免 5 ms 轮询变成 200 Hz。"""
        now = time.monotonic()
        if now >= self._next_control_send:
            self.send_manual(NEUTRAL)

    def request_arm(self, send_neutral: bool = True) -> None:
        self._ensure_heartbeat()
        if self.vehicle.armed:
            raise ControlError("启动时飞控已经解锁，默认拒绝自动接管")
        self.set_state(State.ARM_CONFIRMING)
        self.arm_request_sent = True
        self.control_responsibility = True
        since = time.monotonic()
        self.master.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        self.record_event("request_arm")

        def tick() -> None:
            self.send_neutral_if_due()

        try:
            self.wait_until(lambda: self.vehicle.armed is True, self.confirm_timeout, tick=tick if send_neutral else None)
        except ControlError as exc:
            ack = self._ack_since(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, since)
            self.record_event("arm_confirm_failed", reason=str(exc), ack=ack)
            raise ControlError(f"解锁未由 HEARTBEAT 确认；请求可能已生效：{exc}") from exc
        ack = self._ack_since(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, since)
        if ack and not ack_is_nonfatal(ack.get("result")):
            raise ControlError(f"解锁 COMMAND_ACK 被拒绝：{ack.get('result_name')}")
        self.arm_confirmed = True
        self.record_event("arm_confirmed", ack=ack)

    def check_runtime_safety(self) -> None:
        self._ensure_heartbeat()
        if self.vehicle.mode != "MANUAL":
            raise ControlError(f"飞控模式意外变化为 {self.vehicle.mode!r}")
        if self.vehicle.armed is not True:
            raise ControlError("飞控自行上锁，停止非中立控制")

    def run_timed_action(self, action: str, level: int, duration: float, max_duration: float, horizontal_cap: int, vertical_cap: int) -> None:
        if action not in ACTION_NAMES:
            raise ActionValidationError(f"非法动作：{action}")
        duration = finite_positive(duration, "duration")
        if duration > min(max_duration, MAX_ABSOLUTE_DURATION):
            raise ActionValidationError(f"duration 超过当前软件上限 {min(max_duration, MAX_ABSOLUTE_DURATION):g} 秒")
        if action in ("up", "down"):
            if level > min(500, vertical_cap):
                raise ActionValidationError(f"垂直 level 超过软件上限 {min(500, vertical_cap)}")
        elif level > min(1000, horizontal_cap):
            raise ActionValidationError(f"水平/偏航 level 超过软件上限 {min(1000, horizontal_cap)}")
        self.action_name = action
        self.command_axes = map_action(action, level)
        self.check_runtime_safety()
        self.set_state(State.MOVING)
        end = time.monotonic() + duration
        period = 1.0 / CONTROL_HZ
        next_send = time.monotonic()
        self.record_event("motion_start", duration_s=duration, level=level)
        try:
            while time.monotonic() < end and not self.stop_requested:
                now = time.monotonic()
                self.service_io(now)
                if self.stop_requested:
                    raise ControlError(self.termination_reason or "收到终止请求")
                self.check_runtime_safety()
                if now >= next_send:
                    self.send_manual(self.command_axes)
                    next_send = now + period
                time.sleep(min(0.003, max(0.0, next_send - time.monotonic())))
            if self.stop_requested:
                raise ControlError(self.termination_reason or "收到终止请求")
            self.record_event("motion_complete")
        finally:
            # 即使异常也必须先回中；上锁由 stop_and_disarm 统一完成。
            try:
                self.stop_and_disarm()
            except Exception as cleanup_exc:
                # 清理异常不能覆盖动作/链路的原始异常；留下证据后由外层报告未知状态。
                self.state = State.FAULT
                self.record_event("cleanup_exception", reason=str(cleanup_exc))

    def stop_and_disarm(self, send_neutral: bool = True) -> bool:
        """有界清理：持续回中，再请求上锁，返回是否收到已上锁心跳。"""
        if self.master is None or self.target_system is None:
            self.record_event("cleanup_skipped", reason="无可用连接")
            return False
        self.set_state(State.STOPPING)
        if send_neutral:
            try:
                # MANUAL 控制清理时立即回中一次，再以约 20 Hz 维持一小段时间。
                self.send_manual(NEUTRAL)
            except Exception as exc:
                self.record_event("neutral_send_failed", reason=str(exc))
        stop_deadline = time.monotonic() + DEFAULT_STOP_DURATION
        while time.monotonic() < stop_deadline:
            try:
                self.service_io()
                if send_neutral:
                    self.send_neutral_if_due()
            except Exception as exc:
                self.record_event("neutral_send_failed", reason=str(exc))
                break
            time.sleep(0.005)
        self.command_axes = NEUTRAL
        # 只有本程序曾经请求/确认解锁时才请求上锁；启动时已解锁的拒绝路径不会擅自接管。
        if not (self.arm_request_sent or self.arm_confirmed or self.control_responsibility):
            self.record_event("cleanup_no_disarm", reason="本程序未请求解锁")
            self.set_state(State.DONE)
            return bool(self.vehicle.armed is False)
        self.set_state(State.DISARM_CONFIRMING)
        self.disarm_request_attempted = True
        since = time.monotonic()
        try:
            self.master.mav.command_long_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
            )
            self.disarm_request_sent = True
            self.record_event("request_disarm")
            deadline = time.monotonic() + self.confirm_timeout
            # 清理阶段即使原循环因 Ctrl+C 终止也继续短时运行；这是最后一次
            # 回中/上锁机会，但仍受 confirm-timeout 限制。
            while time.monotonic() < deadline:
                self.service_io()
                if send_neutral:
                    try:
                        self.send_neutral_if_due()
                    except Exception as exc:
                        self.record_event("neutral_send_failed", reason=str(exc))
                        break
                if self.vehicle.armed is False:
                    self.disarm_confirmed = True
                    self.set_state(State.DONE)
                    self.record_event("disarm_confirmed", ack=self._ack_since(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, since))
                    return True
                time.sleep(0.005)
            ack = self._ack_since(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, since)
            self.record_event("disarm_unconfirmed", reason="已请求上锁但未由 HEARTBEAT 确认", ack=ack)
            self.set_state(State.FAULT)
            return False
        except Exception as exc:
            self.record_event("disarm_error", reason=str(exc))
            self.set_state(State.FAULT)
            return False

    def diagnose(self, duration: float, request_version: bool = False, intervals: Optional[Sequence[Tuple[int, float]]] = None) -> None:
        duration = finite_positive(duration, "duration")
        self.connect()
        self.record_event("diagnose_start", duration_s=duration, sends_actuator_control=False)
        if request_version:
            try:
                self.master.mav.command_long_send(
                    self.target_system,
                    self.target_component,
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
                self.record_event("request_autopilot_version")
            except Exception as exc:
                self.record_event("request_version_failed", reason=str(exc))
        for message_id, frequency in intervals or []:
            try:
                self.master.mav.command_long_send(
                    self.target_system,
                    self.target_component,
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
                self.record_event("request_message_interval", message_id=message_id, frequency_hz=frequency)
            except Exception as exc:
                self.record_event("request_message_interval_failed", reason=str(exc), message_id=message_id)
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline and not self.stop_requested:
            self.service_io()
            time.sleep(0.005)
        if self.stop_requested:
            raise ControlError(self.termination_reason or "收到终止请求")
        self.record_event("diagnose_complete", sends_actuator_control=False)
        self.set_state(State.DONE)


def parse_interval(value: str) -> Tuple[int, float]:
    try:
        name, hz_text = value.split("=", 1)
        frequency = float(hz_text)
        if not math.isfinite(frequency) or frequency <= 0:
            raise ValueError
        try:
            message_id = int(name, 0)
        except ValueError:
            message_id = int(getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{name.upper()}"))
        if message_id < 0:
            raise ValueError
        return message_id, frequency
    except (ValueError, AttributeError) as exc:
        raise argparse.ArgumentTypeError("消息频率格式应为 MESSAGE_NAME=HZ 或 ID=HZ") from exc


def add_common_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--connection", required=True, help="pymavlink 端点，例如 udpin:0.0.0.0:14550 或串口路径")
    parser.add_argument("--baud", type=bounded_int(1200, 4_000_000, "baud"), default=115200)
    parser.add_argument("--target-system", type=bounded_int(1, 255, "target-system"))
    parser.add_argument("--target-component", type=bounded_int(0, 255, "target-component"))
    parser.add_argument("--source-system", type=bounded_int(1, 255, "source-system"), default=255)
    parser.add_argument("--source-component", type=bounded_int(0, 255, "source-component"), default=190)
    parser.add_argument("--log-dir", type=Path, default=Path("logs"))
    parser.add_argument("--connect-timeout", type=positive_float, default=DEFAULT_CONNECT_TIMEOUT)
    parser.add_argument("--confirm-timeout", type=positive_float, default=DEFAULT_CONFIRM_TIMEOUT)
    parser.add_argument("--heartbeat-timeout", type=positive_float, default=DEFAULT_HEARTBEAT_TIMEOUT)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="BlueROV2 ArduSub 基础通信与单轴运动测试")
    subparsers = parser.add_subparsers(dest="command", required=True)

    diagnose = subparsers.add_parser("diagnose", help="连接并收集有限时长遥测；不解锁、不切模式、不发 MANUAL_CONTROL")
    add_common_arguments(diagnose)
    diagnose.add_argument("--duration", type=positive_float, default=15.0)
    diagnose.add_argument("--request-version", action="store_true", help="请求 AUTOPILOT_VERSION（旧固件可能拒绝）")
    diagnose.add_argument("--message-interval", action="append", type=parse_interval, default=[], metavar="MSG=HZ")

    status = subparsers.add_parser("status", help="诊断别名，默认收集 5 秒且不请求额外消息")
    add_common_arguments(status)
    status.add_argument("--duration", type=positive_float, default=5.0)
    status.add_argument("--request-version", action="store_true")
    status.add_argument("--message-interval", action="append", type=parse_interval, default=[])

    move = subparsers.add_parser("move", help="MANUAL 模式下执行一次短时单轴运动，然后回中并请求上锁")
    add_common_arguments(move)
    move.add_argument("--action", required=True, choices=ACTION_NAMES)
    move.add_argument("--level", required=True, type=finite_nonnegative_int)
    move.add_argument("--duration", type=positive_float, default=DEFAULT_ACTION_DURATION)
    move.add_argument("--max-duration", type=positive_float, default=DEFAULT_MAX_DURATION)
    move.add_argument("--max-horizontal-level", type=bounded_int(0, 1000, "max-horizontal-level"), default=DEFAULT_HORIZONTAL_CAP)
    move.add_argument("--max-vertical-level", type=bounded_int(0, 500, "max-vertical-level"), default=DEFAULT_VERTICAL_CAP)
    return parser


def make_controller(args: argparse.Namespace, action: Optional[str] = None) -> Controller:
    logger = SessionLogger(args.log_dir, action=action)
    return Controller(
        connection=args.connection,
        baud=args.baud,
        source_system=args.source_system,
        source_component=args.source_component,
        target_system=args.target_system,
        target_component=args.target_component,
        logger=logger,
        connect_timeout=args.connect_timeout,
        confirm_timeout=args.confirm_timeout,
        heartbeat_timeout=args.heartbeat_timeout,
    )


def validate_move_parameters(args: argparse.Namespace) -> None:
    """在连接/模式/解锁前拒绝全部非法运动参数。"""
    duration = finite_positive(args.duration, "duration")
    max_duration = finite_positive(args.max_duration, "max-duration")
    if duration > min(max_duration, MAX_ABSOLUTE_DURATION):
        raise ActionValidationError(f"duration 超过当前软件上限 {min(max_duration, MAX_ABSOLUTE_DURATION):g} 秒")
    if args.action in ("up", "down"):
        if args.level > min(500, args.max_vertical_level):
            raise ActionValidationError(f"垂直 level 超过软件上限 {min(500, args.max_vertical_level)}")
    elif args.level > min(1000, args.max_horizontal_level):
        raise ActionValidationError(f"水平/偏航 level 超过软件上限 {min(1000, args.max_horizontal_level)}")
    map_action(args.action, args.level)


def install_signal_handlers(controller: Controller) -> None:
    def handler(signum: int, _frame: Any) -> None:
        controller.stop_requested = True
        controller.termination_reason = f"收到信号 {signum}"
        controller.record_event("termination_requested", reason=controller.termination_reason)

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, handler)
        except (ValueError, OSError):
            pass


def run(args: argparse.Namespace) -> int:
    if args.command == "move":
        # 先校验，避免输入错误却已经请求模式或解锁。
        validate_move_parameters(args)
    action = args.action if args.command == "move" else None
    controller = make_controller(args, action=action)
    install_signal_handlers(controller)
    exit_code = 0
    original_error: Optional[BaseException] = None
    try:
        if args.command in ("diagnose", "status"):
            controller.diagnose(args.duration, args.request_version, args.message_interval)
            print(
                f"诊断完成：target={controller.target_system}/{controller.target_component}, "
                f"autopilot={controller.vehicle.autopilot}, type={controller.vehicle.vehicle_type}, "
                f"mode={controller.vehicle.mode or '未知'}, armed={controller.vehicle.armed}, "
                f"messages={controller.vehicle.message_count}, "
                f"autopilot_version={'收到' if controller.vehicle.autopilot_version else '未收到'}\n"
                f"姿态/角速度(rad, rad/s)：{json.dumps(controller.vehicle.attitude, ensure_ascii=False)}\n"
                f"电池：{json.dumps({**controller.vehicle.sys_status, **controller.vehicle.battery}, ensure_ascii=False)}\n"
                "未发送执行器控制。"
            )
        else:
            controller.connect()
            if controller.vehicle.armed:
                raise ControlError("启动时飞控已解锁，拒绝自动接管；请人工确认后重新运行")
            controller.request_mode("MANUAL")
            controller.set_state(State.NEUTRAL)
            controller.send_manual(NEUTRAL)
            controller.request_arm()
            controller.run_timed_action(
                args.action,
                args.level,
                args.duration,
                args.max_duration,
                args.max_horizontal_level,
                args.max_vertical_level,
            )
            if controller.state == State.FAULT:
                exit_code = 2
    except KeyboardInterrupt:
        controller.stop_requested = True
        controller.termination_reason = "Ctrl+C"
        original_error = ControlError("Ctrl+C")
        exit_code = 130
    except (ControlError, OSError, ValueError) as exc:
        original_error = exc
        controller.termination_reason = str(exc)
        if controller.state not in (State.DONE, State.FAULT):
            controller.state = State.FAULT
        print(f"错误：{exc}", file=sys.stderr)
        exit_code = 1
    finally:
        # 仅在本程序可能请求过解锁时清理；启动时已解锁的拒绝路径不会自动上锁。
        if args.command == "move" and controller.master is not None and controller.control_responsibility and not controller.disarm_request_attempted and controller.state not in (State.DONE, State.DISARM_CONFIRMING):
            try:
                confirmed = controller.stop_and_disarm()
            except Exception as cleanup_exc:
                controller.state = State.FAULT
                controller.record_event("cleanup_exception", reason=str(cleanup_exc))
                confirmed = False
            if not confirmed and exit_code == 0:
                exit_code = 2
        if args.command == "move" and controller.control_responsibility:
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
        if original_error:
            controller.record_event("exit", reason=str(original_error), final_status=("confirmed_disarmed" if controller.disarm_confirmed else "unknown_or_unconfirmed"))
        controller.close()
        controller.logger.close()
    return exit_code


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return run(args)
    except ActionValidationError as exc:
        parser.error(str(exc))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
