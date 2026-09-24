"""Pure motion authorization rules for the STM32 bridge."""

from .nav_protocol import GOAL_NONE, NAV_FOLLOWING


NAVIGATION_TASK_STATES = frozenset((1, 3, 6, 9))


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
