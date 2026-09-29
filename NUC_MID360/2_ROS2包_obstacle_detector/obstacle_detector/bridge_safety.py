"""Pure motion authorization rules for the STM32 bridge."""

import math

from .nav_protocol import (
    GOAL_NONE,
    NAV_FOLLOWING,
    NAV_IDLE,
    NAV_WAIT_PATH,
    TASK_WAIT_START,
)


NAVIGATION_TASK_STATES = frozenset((1, 3, 6, 9))


def medical_mission_restarted(
    previous_task_state,
    previous_nav_status,
    task_state: int,
    nav_status: int,
    pose_gap_s: float,
    pose_timeout_s: float,
) -> bool:
    """Recognize an STM32 reboot even when task state remains WAIT_START."""

    if task_state != TASK_WAIT_START:
        return False
    if previous_task_state != TASK_WAIT_START:
        return True
    if pose_gap_s > pose_timeout_s:
        return True
    return previous_nav_status == NAV_FOLLOWING and nav_status in (
        NAV_IDLE,
        NAV_WAIT_PATH,
    )


class SettledStopDetector:
    def __init__(
        self,
        linear_threshold_m_s: float,
        angular_threshold_rad_s: float,
        settle_s: float,
    ) -> None:
        self.linear_threshold_m_s = max(0.0, float(linear_threshold_m_s))
        self.angular_threshold_rad_s = max(0.0, float(angular_threshold_rad_s))
        self.settle_s = max(0.0, float(settle_s))
        self.stopped_since_s = None

    def reset(self) -> None:
        self.stopped_since_s = None

    def update(self, vx: float, vy: float, wz: float, now_s: float) -> bool:
        stopped = (
            math.hypot(vx, vy) <= self.linear_threshold_m_s
            and abs(wz) <= self.angular_threshold_rad_s
        )
        if not stopped:
            self.stopped_since_s = None
            return False
        if self.stopped_since_s is None:
            self.stopped_since_s = float(now_s)
        return float(now_s) - self.stopped_since_s >= self.settle_s


def navigation_motion_is_authorized(
    pose_fresh: bool,
    navigator_fresh: bool,
    handoff_ready: bool,
    navigator_following_ready: bool,
    task_state: int,
    pose_nav_status: int,
    requested_request_id: int,
    requested_goal_id: int,
    navigator_request_id: int,
    navigator_goal_id: int,
    navigator_state: int,
) -> bool:
    return (
        pose_fresh
        and navigator_fresh
        and handoff_ready
        and navigator_following_ready
        and task_state in NAVIGATION_TASK_STATES
        and pose_nav_status == NAV_FOLLOWING
        and requested_request_id != 0
        and requested_goal_id != GOAL_NONE
        and navigator_request_id == requested_request_id
        and navigator_goal_id == requested_goal_id
        and navigator_state == NAV_FOLLOWING
    )
