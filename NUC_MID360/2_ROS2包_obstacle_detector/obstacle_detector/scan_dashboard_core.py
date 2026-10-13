"""Validation helpers for the scan dashboard."""

import json
from typing import Dict, Optional, Tuple

from .nav_protocol import (
    SCAN_CONTEXT_BED1,
    SCAN_CONTEXT_BED3,
    SCAN_CONTEXT_ORDER,
    START_WAIT_TASK_STATES,
    TASK_ACTIVE_STATES,
    TASK_COMPLETE,
    TASK_NAV_ERROR,
    TASK_NAV_HOME,
    TASK_WAIT_BED1_START,
    TASK_WAIT_BED3_START,
)


VALID_CONTEXTS = frozenset(
    (SCAN_CONTEXT_ORDER, SCAN_CONTEXT_BED1, SCAN_CONTEXT_BED3)
)
TELE_SCAN_STATES = frozenset(("standby", "scanning", "success", "failed"))

START_WAITING = "waiting"
START_READY = "ready"
START_RUNNING = "running"
START_SUCCESS = "success"
START_ERROR = "error"

# Single status word shown above the camera thumbnail.  ``ready`` keeps the
# original 发车 gate from the bridge; every other value is derived from the
# STM32 task state alone, so the label keeps working if the gate is absent.
START_STATUS_TEXT = {
    START_WAITING: "等待",
    START_READY: "发车",
    START_RUNNING: "运行",
    START_SUCCESS: "成功",
    START_ERROR: "失败",
}

START_STATUS_COLOR = {
    START_WAITING: "#e53935",
    START_READY: "#138a36",
    START_RUNNING: "#138a36",
    START_SUCCESS: "#138a36",
    START_ERROR: "#e53935",
}


def parse_scan_result(payload: str) -> Optional[Tuple[int, str]]:
    try:
        data = json.loads(payload)
        context = int(data["context"])
        value = str(data["value"]).strip()
    except (KeyError, TypeError, ValueError, json.JSONDecodeError):
        return None
    if context not in VALID_CONTEXTS or not value:
        return None
    return context, value


def parse_scan_status(payload: str) -> Optional[Dict[int, str]]:
    try:
        values = json.loads(payload)["scan_values"]
        parsed = {
            context: str(values.get(str(context), values.get(context, ""))).strip()
            for context in VALID_CONTEXTS
        }
    except (AttributeError, KeyError, TypeError, ValueError, json.JSONDecodeError):
        return None
    return parsed


def parse_tele_scan_state(payload: str) -> Optional[str]:
    try:
        state = str(json.loads(payload)["tele_scan_state"])
    except (KeyError, TypeError, ValueError, json.JSONDecodeError):
        return None
    return state if state in TELE_SCAN_STATES else None


def parse_bridge_status(payload: str) -> Optional[dict]:
    """Extract the STM32 start gate for the dashboard."""
    try:
        data = json.loads(payload)
        stm32 = data.get("stm32") or {}
        gate = data.get("start_gate") or {}
        arm_debug = data.get("arm_debug") or {}
        tts = data.get("tts") or {}
        debug_bed = tts.get("debug_bed")
        if debug_bed not in (1, 3):
            debug_bed = None
        task_state = int(stm32.get("task_state", -1))
        nav_status = int(stm32.get("nav_status", 0))
        return {
            "serial": bool(data.get("serial")),
            "pose": bool(data.get("pose")),
            "task_state": task_state,
            "nav_status": nav_status,
            "waiting_for_button": bool(gate.get("waiting_for_button"))
            or task_state in START_WAIT_TASK_STATES,
            "path_ready": bool(gate.get("path_ready")),
            "can_start": bool(gate.get("can_start")),
            "motion_authorized": bool(data.get("motion_authorized")),
            "reason": str(gate.get("reason", "")),
            "arm_debug": {
                "allowed": bool(arm_debug.get("allowed")),
                "deployed": bool(arm_debug.get("deployed")),
                "pending": bool(arm_debug.get("pending")),
                "requested_deployed": bool(
                    arm_debug.get("requested_deployed")
                ),
                "result": str(arm_debug.get("result", "idle")),
            },
            "tts_debug": {
                "allowed": bool(tts.get("debug_allowed")),
                "playing": bool(tts.get("debug_playing")),
                "bed": debug_bed,
                "result": str(tts.get("debug_result", "idle")),
            },
        }
    except (AttributeError, KeyError, TypeError, ValueError, json.JSONDecodeError):
        return None


def describe_start_status(
    status: Optional[dict], tele_scan_state: str = "standby"
) -> Tuple[str, str, str]:
    """Map the STM32 task state onto the one status word on the dashboard.

    Returns ``(state, text, colour)``.  The wait states are split by the
    bridge's 发车 gate: red 等待 until Nav2 has a path ready while the chassis
    is still locked, green 发车 once button C would actually start the run.
    Arriving home is the success state and a navigation failure the error
    state; TASK_NAV_HOME deliberately stays on 运行 so the label only turns
    green-success after the drive home has finished.  An unknown or missing
    task state stays on 等待 rather than claiming the robot is moving.
    """
    if status is None:
        state = START_WAITING
    else:
        task_state = int(status["task_state"])
        if task_state == TASK_COMPLETE:
            state = START_SUCCESS
        elif task_state == TASK_NAV_ERROR:
            state = START_ERROR
        elif task_state in START_WAIT_TASK_STATES:
            state = START_READY if status["can_start"] else START_WAITING
        elif task_state in TASK_ACTIVE_STATES:
            # TASK_NAV_HOME counts as running: the robot is still driving.
            state = START_RUNNING
        else:
            state = START_WAITING
    text = START_STATUS_TEXT[state]
    qr_confirmed = (
        status is not None
        and int(status["task_state"])
        in (TASK_WAIT_BED1_START, TASK_WAIT_BED3_START)
    )
    if state == START_READY and (
        tele_scan_state == "success" or qr_confirmed
    ):
        text = "可发车"
    return state, text, START_STATUS_COLOR[state]


def describe_start_reason(status: Optional[dict]) -> str:
    """Keep the cause visible when cold startup cannot open the button gate."""
    if status is None:
        return "等待导航桥接状态"
    reason = status.get("reason", "")
    messages = {
        "serial_disconnected": "STM32 串口未连接",
        "waiting_pose": "等待 STM32 位姿遥测",
        "waiting_goal_request": "等待 STM32 目标请求",
        "waiting_navigator": "等待导航器响应当前目标",
        "waiting_stm32_nav_ack": "等待 STM32 确认路径就绪",
        "stm32:waiting_telemetry": "等待 STM32 位姿/轮速遥测",
        "stm32:rebooting": "STM32 遥测中断",
        "stm32:waiting_reset_state": "等待 STM32 进入发车等待状态",
        "stm32:waiting_stationary": "等待底盘静止",
        "stm32:recovering": "STM32 静止遥测确认中",
    }
    if reason in ("ready", "not_waiting_for_start"):
        return ""
    if reason.startswith("navigator:waiting_for_nav2_active"):
        nodes = reason.partition("waiting_for_nav2_active")[2].lstrip(":")
        return "等待导航节点激活" + ("：" + nodes if nodes else "")
    if reason.startswith("navigator:prestart_plan"):
        return "预规划暂未成功，正在重试"
    if reason.startswith("navigator:"):
        return "等待路径：" + reason.partition(":")[2]
    return messages.get(reason, "等待首条路径就绪")
