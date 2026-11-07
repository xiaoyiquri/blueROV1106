#!/usr/bin/env python3
"""Water Linked A50 TCP JSON -> ArduPilot VISION_POSITION_DELTA 桥接。

这个桥只做外部导航输入，不发送解锁、模式或推进器控制。ArduSub 官方旧版
DVL 集成要求支持 VISION_POSITION_DELTA 的固件；检测到 3.5.x 等旧固件时会
明确退出，避免把未支持消息误当成已经融合。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import socket
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional, Sequence, Tuple

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil


MIN_FIRMWARE = (4, 1)


class BridgeError(RuntimeError):
    pass


def finite(value: Any, name: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise BridgeError(f"{name} 不是数字") from exc
    if not math.isfinite(number):
        raise BridgeError(f"{name} 不是有限数")
    return number


def version_tuple(flight_sw_version: int) -> Tuple[int, int, int]:
    return ((flight_sw_version >> 24) & 0xFF, (flight_sw_version >> 16) & 0xFF, (flight_sw_version >> 8) & 0xFF)


def mat_vec(matrix: Sequence[float], vector: Sequence[float]) -> Tuple[float, float, float]:
    if len(matrix) != 9:
        raise BridgeError("dvl_to_ned_rotation 必须包含 9 个元素")
    return (
        matrix[0] * vector[0] + matrix[1] * vector[1] + matrix[2] * vector[2],
        matrix[3] * vector[0] + matrix[4] * vector[1] + matrix[5] * vector[2],
        matrix[6] * vector[0] + matrix[7] * vector[1] + matrix[8] * vector[2],
    )


def wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class DvlSample:
    position: Tuple[float, float, float]
    attitude: Tuple[float, float, float]
    velocity_valid: bool
    beams_valid: bool
    raw_time: Optional[float]
    status: int
    position_std: float = 0.0


def parse_dead_reckoning(data: Dict[str, Any]) -> DvlSample:
    required = ("x", "y", "z", "roll", "pitch", "yaw", "std", "status", "format")
    missing = [key for key in required if key not in data]
    if missing:
        raise BridgeError("DVL 航位推算消息缺少字段：" + ",".join(missing))
    finite(data["std"], "std")
    try:
        status = int(data["status"])
    except (TypeError, ValueError) as exc:
        raise BridgeError("DVL 航位推算 status 无效") from exc
    beams = data.get("transducers", [])
    beams_valid = bool(beams) and all(bool(beam.get("beam_valid", False)) for beam in beams)
    return DvlSample(
        position=tuple(finite(data[key], key) for key in ("x", "y", "z")),  # type: ignore[return-value]
        attitude=tuple(finite(data[key], key) for key in ("roll", "pitch", "yaw")),  # type: ignore[return-value]
        velocity_valid=bool(data.get("velocity_valid", False)),
        beams_valid=beams_valid,
        raw_time=finite(data["ts"], "ts") if "ts" in data else None,
        status=status,
        position_std=finite(data["std"], "std"),
    )


def dvl_attitude_delta_rad(current: DvlSample, previous: DvlSample) -> Tuple[float, float, float]:
    """A50 的 roll/pitch/yaw 是度；VISION_POSITION_DELTA 要求弧度。"""
    return tuple(
        math.radians((current.attitude[index] - previous.attitude[index] + 180.0) % 360.0 - 180.0)
        for index in range(3)
    )  # type: ignore[return-value]


def parse_velocity_validity(data: Dict[str, Any]) -> Tuple[bool, bool]:
    """读取 A50 velocity report 的有效性，供后续死 reckoning 增量使用。"""
    beams = data.get("transducers", [])
    try:
        fom = finite(data.get("fom"), "fom")
    except BridgeError:
        return False, False
    try:
        status_ok = int(data.get("status", 1)) == 0
    except (TypeError, ValueError):
        status_ok = False
    return status_ok and bool(data.get("velocity_valid", False)) and fom >= 0.0, bool(beams) and all(bool(beam.get("beam_valid", False)) for beam in beams)


class DvlTcpReader:
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self.sock: Optional[socket.socket] = None
        self.buffer = ""

    def connect(self) -> None:
        self.sock = socket.create_connection((self.host, self.port), timeout=5.0)
        self.sock.settimeout(1.0)

    def lines(self):
        if self.sock is None:
            self.connect()
        assert self.sock is not None
        while True:
            try:
                chunk = self.sock.recv(4096)
            except socket.timeout:
                continue
            if not chunk:
                raise BridgeError("DVL TCP 连接断开")
            self.buffer += chunk.decode("utf-8", errors="replace")
            while "\n" in self.buffer:
                line, _, self.buffer = self.buffer.partition("\n")
                if line.strip():
                    yield json.loads(line)

    def close(self) -> None:
        if self.sock is not None:
            self.sock.close()
            self.sock = None


def identify(
    master: Any,
    requested_target_system: Optional[int] = None,
    requested_target_component: Optional[int] = None,
    timeout: float = 10.0,
) -> Tuple[int, int, Tuple[int, int, int]]:
    deadline = time.monotonic() + timeout
    candidates = set()
    target = None
    flight_version = None
    version_requested = False
    while time.monotonic() < deadline:
        master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)
        msg = master.recv_match(blocking=False)
        if msg is not None:
            if msg.get_type() == "HEARTBEAT" and getattr(msg, "autopilot", None) != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                candidate = (msg.get_srcSystem(), msg.get_srcComponent())
                if requested_target_system is not None and candidate[0] != requested_target_system:
                    continue
                if requested_target_component is not None and candidate[1] != requested_target_component:
                    continue
                candidates.add(candidate)
                if len(candidates) > 1:
                    formatted = ", ".join(f"{system}/{component}" for system, component in sorted(candidates))
                    raise BridgeError(f"发现多个候选飞控 ({formatted})，请先隔离端点或增加目标选择配置")
                target = next(iter(candidates))
                master.target_system, master.target_component = target
                if not version_requested:
                    master.mav.command_long_send(
                        target[0], target[1], mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES,
                        0, 1, 0, 0, 0, 0, 0, 0,
                    )
                    version_requested = True
            elif msg.get_type() == "AUTOPILOT_VERSION" and target is not None and (msg.get_srcSystem(), msg.get_srcComponent()) == target:
                flight_version = int(msg.flight_sw_version)
            if target and flight_version is not None:
                return target[0], target[1], version_tuple(flight_version)
        time.sleep(0.02)
    raise BridgeError("未在超时内识别飞控及 AUTOPILOT_VERSION")


def run(args: argparse.Namespace) -> int:
    matrix = [finite(value, "dvl_to_ned_rotation") for value in args.rotation]
    if len(matrix) != 9:
        raise BridgeError("必须使用 9 个 dvl_to_ned_rotation 元素")
    master = mavutil.mavlink_connection(
        args.connection,
        baud=args.baud,
        source_system=args.source_system,
        source_component=args.source_component,
        dialect="ardupilotmega",
    )
    reader = DvlTcpReader(args.dvl_ip, args.dvl_port)
    previous: Optional[DvlSample] = None
    previous_mono: Optional[float] = None
    try:
        system, component, version = identify(master, args.target_system, args.target_component)
        print(f"target={system}/{component} firmware={version[0]}.{version[1]}.{version[2]}")
        if version[:2] < MIN_FIRMWARE:
            raise BridgeError(
                f"当前固件 {version[0]}.{version[1]}.{version[2]} 不支持本桥使用的 VISION_POSITION_DELTA；"
                "需要先核实并升级到官方支持该 DVL 接入的固件，程序不会发送不兼容消息"
            )
        reader.connect()
        velocity_valid = False
        beams_valid = False
        for data in reader.lines():
            if "time" in data and "velocity_valid" in data:
                velocity_valid, beams_valid = parse_velocity_validity(data)
                continue
            if "ts" not in data:
                continue
            sample = parse_dead_reckoning(data)
            now = time.monotonic()
            if previous is None:
                previous = sample
                previous_mono = now
                continue
            if sample.status != 0 or previous.status != 0 or not velocity_valid or not beams_valid:
                previous = sample
                previous_mono = now
                continue
            assert previous_mono is not None
            delta_usec = int(max(1.0, (now - previous_mono) * 1_000_000.0))
            position_delta = mat_vec(matrix, tuple(sample.position[i] - previous.position[i] for i in range(3)))
            angle_delta = mat_vec(matrix, dvl_attitude_delta_rad(sample, previous))
            time_usec = int(now * 1_000_000.0)
            master.mav.vision_position_delta_send(
                time_usec,
                delta_usec,
                angle_delta,
                position_delta,
                100.0,
            )
            previous = sample
            previous_mono = now
    finally:
        reader.close()
        master.close()
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Water Linked A50 到 ArduPilot VISION_POSITION_DELTA 桥")
    parser.add_argument("--connection", required=True, help="发送到飞控/Companion 的 pymavlink 端点")
    parser.add_argument("--dvl-ip", default="192.168.2.95")
    parser.add_argument("--dvl-port", type=int, default=16171)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--source-system", type=int, default=254)
    parser.add_argument("--source-component", type=int, default=191)
    parser.add_argument("--target-system", type=int)
    parser.add_argument("--target-component", type=int)
    parser.add_argument(
        "--rotation",
        nargs=9,
        type=float,
        required=True,
        metavar="R",
        help="已核验的 DVL 坐标到 NED 坐标 3x3 矩阵，按行输入",
    )
    return parser


if __name__ == "__main__":
    try:
        raise SystemExit(run(build_parser().parse_args()))
    except BridgeError as exc:
        raise SystemExit(f"错误：{exc}") from exc
