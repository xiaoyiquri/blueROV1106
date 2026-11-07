#!/usr/bin/env python3
"""BlueROV2 分段轨迹执行器。

这里的“轨迹”是定时 MANUAL_CONTROL 段的开环计划。ArduSub 负责每个时刻的
RC 输入转换和六推进器混控；本文件不直接写推进器 PWM，也不提供位置闭环。
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

from rov_control import (
    CONTROL_HZ,
    DEFAULT_HORIZONTAL_CAP,
    DEFAULT_VERTICAL_CAP,
    MAX_ABSOLUTE_DURATION,
    ActionValidationError,
    ControlError,
    Controller,
    NEUTRAL,
    SessionLogger,
    State,
    add_common_arguments,
    finite_positive,
    install_signal_handlers,
    map_action,
)


MAX_TRAJECTORY_DURATION = 300.0
DEFAULT_TRANSITION_NEUTRAL = 0.2
DEFAULT_MAX_TOTAL_DURATION = 60.0


@dataclasses.dataclass(frozen=True)
class Segment:
    """一个持续 duration 秒的 MANUAL_CONTROL 轴值。"""

    label: str
    axes: Tuple[int, int, int, int]
    duration: float
    kind: str = "action"
    metadata: Dict[str, Any] = dataclasses.field(default_factory=dict)


def _finite_number(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise ActionValidationError(f"{name} 不能是布尔值")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ActionValidationError(f"{name} 必须是数字") from exc
    if not math.isfinite(number):
        raise ActionValidationError(f"{name} 必须是有限数")
    return number


def _int_field(value: Any, name: str) -> int:
    number = _finite_number(value, name)
    if number != int(number):
        raise ActionValidationError(f"{name} 必须是整数")
    return int(number)


def _duration(value: Any, name: str = "duration") -> float:
    duration = _finite_number(value, name)
    return finite_positive(duration, name)


def _axes_from_json(value: Any) -> Tuple[int, int, int, int]:
    if not isinstance(value, dict):
        raise ActionValidationError("axes 必须是包含 x/y/z/r 的对象")
    required = ("x", "y", "z", "r")
    if any(key not in value for key in required):
        raise ActionValidationError("axes 必须同时包含 x、y、z、r")
    axes = tuple(_int_field(value[key], f"axes.{key}") for key in required)
    x, y, z, r = axes
    if not (-1000 <= x <= 1000 and -1000 <= y <= 1000 and 0 <= z <= 1000 and -1000 <= r <= 1000):
        raise ActionValidationError("axes 超出 MANUAL_CONTROL 协议范围")
    return axes  # type: ignore[return-value]


def _circle_segment(item: Dict[str, Any], index: int) -> Segment:
    radius = _finite_number(item.get("radius_m"), f"segments[{index}].radius_m")
    if radius <= 0:
        raise ActionValidationError(f"segments[{index}].radius_m 必须为正数")
    duration = _duration(item.get("duration"), f"segments[{index}].duration")
    forward_level = _int_field(item.get("forward_level", 0), f"segments[{index}].forward_level")
    yaw_level = _int_field(item.get("yaw_level", 0), f"segments[{index}].yaw_level")
    direction = str(item.get("direction", "left"))
    if direction not in ("left", "right"):
        raise ActionValidationError(f"segments[{index}].direction 必须是 left 或 right")
    if not 0 <= forward_level <= 1000 or not 0 <= yaw_level <= 1000:
        raise ActionValidationError(f"segments[{index}] 的 forward_level/yaw_level 必须在 0..1000")
    turns = item.get("turns")
    if turns is not None and _finite_number(turns, f"segments[{index}].turns") <= 0:
        raise ActionValidationError(f"segments[{index}].turns 必须为正数")
    yaw = -yaw_level if direction == "left" else yaw_level
    label = str(item.get("label", f"circle-{index}"))
    return Segment(
        label=label,
        axes=(forward_level, 0, 500, yaw),
        duration=duration,
        kind="circle_open_loop",
        metadata={
            "radius_m": radius,
            "direction": direction,
            "turns": turns,
            "forward_level": forward_level,
            "yaw_level": yaw_level,
            "warning": "radius_m 是计划标注；未使用位置反馈，实际半径不保证",
        },
    )


def parse_segments(document: Any) -> List[Segment]:
    if not isinstance(document, dict):
        raise ActionValidationError("轨迹文件根对象必须是 JSON 对象")
    items = document.get("segments")
    if not isinstance(items, list) or not items:
        raise ActionValidationError("轨迹文件必须包含非空 segments 数组")
    segments: List[Segment] = []
    for index, item in enumerate(items):
        if not isinstance(item, dict):
            raise ActionValidationError(f"segments[{index}] 必须是对象")
        kind = str(item.get("type", "action"))
        if kind == "circle":
            segment = _circle_segment(item, index)
        elif kind in ("action", "neutral"):
            duration = _duration(item.get("duration"), f"segments[{index}].duration")
            if kind == "neutral":
                axes = NEUTRAL
                label = str(item.get("label", f"neutral-{index}"))
            elif "action" in item:
                action = str(item["action"])
                level = _int_field(item.get("level"), f"segments[{index}].level")
                axes = map_action(action, level)
                label = str(item.get("label", f"{action}-{index}"))
            elif "axes" in item:
                axes = _axes_from_json(item["axes"])
                label = str(item.get("label", f"axes-{index}"))
            else:
                raise ActionValidationError(f"segments[{index}] 需要 action、axes 或 type=neutral")
            segment = Segment(label=label, axes=axes, duration=duration, kind=kind, metadata={"source_index": index})
        else:
            raise ActionValidationError(f"segments[{index}].type 不支持：{kind}")
        segments.append(segment)
    return segments


def load_segments(path: Path) -> Tuple[Dict[str, Any], List[Segment]]:
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ActionValidationError(f"读取轨迹文件失败：{exc}") from exc
    return document, parse_segments(document)


def validate_plan(
    segments: Sequence[Segment],
    transition_neutral: float,
    max_total_duration: float,
    max_horizontal_level: int,
    max_vertical_level: int,
) -> float:
    transition_neutral = _duration(transition_neutral, "transition-neutral") if transition_neutral else 0.0
    max_total_duration = finite_positive(max_total_duration, "max-total-duration")
    if max_total_duration > MAX_TRAJECTORY_DURATION:
        raise ActionValidationError(f"max-total-duration 不能超过 {MAX_TRAJECTORY_DURATION:g} 秒")
    if not segments:
        raise ActionValidationError("轨迹不能为空")
    total = 0.0
    for index, segment in enumerate(segments):
        if segment.duration > MAX_ABSOLUTE_DURATION:
            raise ActionValidationError(f"segments[{index}] duration 不能超过 {MAX_ABSOLUTE_DURATION:g} 秒")
        x, y, z, r = segment.axes
        if max(abs(x), abs(y), abs(r)) > max_horizontal_level:
            raise ActionValidationError(f"segments[{index}] 水平/偏航输入超过软件上限 {max_horizontal_level}")
        if abs(z - 500) > max_vertical_level:
            raise ActionValidationError(f"segments[{index}] 垂直偏移超过软件上限 {max_vertical_level}")
        total += segment.duration
        if index:
            total += transition_neutral
    if total > max_total_duration:
        raise ActionValidationError(f"轨迹总时长 {total:g} 秒超过软件上限 {max_total_duration:g} 秒")
    return total


def describe_plan(segments: Sequence[Segment], total: float) -> str:
    rows = [f"segments={len(segments)}, total_duration={total:g}s"]
    for index, segment in enumerate(segments):
        rows.append(f"[{index}] {segment.label}: axes={segment.axes}, duration={segment.duration:g}s, kind={segment.kind}")
        if segment.metadata.get("warning"):
            rows.append(f"    warning: {segment.metadata['warning']}")
    return "\n".join(rows)


class TrajectoryRunner:
    def __init__(
        self,
        controller: Controller,
        segments: Sequence[Segment],
        transition_neutral: float,
        total_duration: float,
    ):
        self.controller = controller
        self.segments = list(segments)
        self.transition_neutral = transition_neutral
        self.total_duration = total_duration

    def _run_axes(self, segment: Segment, index: int) -> None:
        controller = self.controller
        controller.action_name = segment.label
        controller.command_axes = segment.axes
        controller.set_state(State.MOVING)
        controller.record_event(
            "trajectory_segment_start",
            segment_index=index,
            segment_kind=segment.kind,
            segment_duration_s=segment.duration,
            segment_axes=segment.axes,
            segment_metadata=segment.metadata,
        )
        end = time.monotonic() + segment.duration
        period = 1.0 / CONTROL_HZ
        next_send = time.monotonic()
        while time.monotonic() < end and not controller.stop_requested:
            now = time.monotonic()
            controller.service_io(now)
            if controller.stop_requested:
                raise ControlError(controller.termination_reason or "收到终止请求")
            controller.check_runtime_safety()
            if now >= next_send:
                controller.send_manual(segment.axes)
                next_send = now + period
            time.sleep(min(0.003, max(0.0, next_send - time.monotonic())))
        if controller.stop_requested:
            raise ControlError(controller.termination_reason or "收到终止请求")
        controller.record_event("trajectory_segment_complete", segment_index=index, segment_label=segment.label)

    def _neutral_transition(self, index: int) -> None:
        if self.transition_neutral <= 0:
            return
        controller = self.controller
        controller.set_state(State.NEUTRAL)
        controller.action_name = f"transition-neutral-{index}"
        end = time.monotonic() + self.transition_neutral
        next_send = time.monotonic()
        period = 1.0 / CONTROL_HZ
        controller.record_event("trajectory_transition_neutral", segment_index=index, duration_s=self.transition_neutral)
        while time.monotonic() < end and not controller.stop_requested:
            now = time.monotonic()
            controller.service_io(now)
            controller.check_runtime_safety()
            if now >= next_send:
                controller.send_manual(NEUTRAL)
                next_send = now + period
            time.sleep(min(0.003, max(0.0, next_send - time.monotonic())))

    def run(self) -> None:
        controller = self.controller
        controller.connect()
        try:
            if controller.vehicle.armed:
                raise ControlError("启动时飞控已解锁，拒绝自动接管")
            controller.request_mode("MANUAL")
            controller.set_state(State.NEUTRAL)
            controller.send_manual(NEUTRAL)
            controller.request_arm()
            controller.record_event("trajectory_start", segment_count=len(self.segments), total_duration_s=self.total_duration)
            for index, segment in enumerate(self.segments):
                if index:
                    self._neutral_transition(index)
                self._run_axes(segment, index)
            controller.record_event("trajectory_complete")
        finally:
            if controller.master is not None and controller.control_responsibility and not controller.disarm_request_attempted:
                try:
                    controller.stop_and_disarm()
                except Exception as exc:
                    controller.state = State.FAULT
                    controller.record_event("trajectory_cleanup_exception", reason=str(exc))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="BlueROV2 ArduSub 分段轨迹执行器")
    sub = parser.add_subparsers(dest="command", required=True)
    trajectory = sub.add_parser("run", help="验证并执行 JSON 轨迹")
    add_common_arguments(trajectory)
    trajectory.add_argument("--file", required=True, type=Path, help="JSON 轨迹文件")
    trajectory.add_argument("--transition-neutral", type=float, default=DEFAULT_TRANSITION_NEUTRAL)
    trajectory.add_argument("--max-total-duration", type=float, default=DEFAULT_MAX_TOTAL_DURATION)
    trajectory.add_argument("--max-horizontal-level", type=int, default=DEFAULT_HORIZONTAL_CAP)
    trajectory.add_argument("--max-vertical-level", type=int, default=DEFAULT_VERTICAL_CAP)
    trajectory.add_argument("--dry-run", action="store_true", help="只解析/校验/打印轨迹，不连接飞控")
    return parser


def make_trajectory_controller(args: argparse.Namespace, logger: SessionLogger) -> Controller:
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


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.command != "run":
        parser.error("未知子命令")
    try:
        if not 0 <= args.max_horizontal_level <= 1000:
            raise ActionValidationError("max-horizontal-level 必须在 0..1000")
        if not 0 <= args.max_vertical_level <= 500:
            raise ActionValidationError("max-vertical-level 必须在 0..500")
        document, segments = load_segments(args.file)
        total = validate_plan(
            segments,
            args.transition_neutral,
            args.max_total_duration,
            args.max_horizontal_level,
            args.max_vertical_level,
        )
        print(describe_plan(segments, total))
        if args.dry_run:
            print("dry-run：未连接飞控，未发送 MAVLink 消息。")
            return 0
    except ActionValidationError as exc:
        parser.error(str(exc))
        return 2

    logger = SessionLogger(args.log_dir, action="trajectory")
    controller = make_trajectory_controller(args, logger)
    install_signal_handlers(controller)
    exit_code = 0
    original_error: Optional[BaseException] = None
    try:
        controller.record_event("trajectory_plan", plan=document, segment_count=len(segments), total_duration_s=total)
        TrajectoryRunner(controller, segments, args.transition_neutral, total).run()
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
        if original_error:
            controller.record_event("trajectory_exit", reason=str(original_error), final_status=("confirmed_disarmed" if controller.disarm_confirmed else "unknown_or_unconfirmed"))
        if controller.disarm_confirmed:
            print("轨迹结束：已确认上锁。")
        elif controller.control_responsibility:
            print("轨迹结束：最终上锁状态未确认，请按日志和现场状态核实。", file=sys.stderr)
            if exit_code == 0:
                exit_code = 2
        controller.close()
        logger.close()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
