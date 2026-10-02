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
XDRIVE_TRANSLATION_SCALE = 1.0 / math.sqrt(2.0)


def omni_wheel_speeds(vx: float, vy: float, wz: float, yaw_radius_m: float):
    """Return physical wheel-surface speeds for a 45-degree X-drive.

    ``vx`` and ``vy`` are ROS body velocities.  The four ordinary omni wheels
    are mounted at 45 degrees, so the translational projection onto each
    wheel's drive direction carries a 1/sqrt(2) factor.  ``yaw_radius_m`` is
    the effective perpendicular lever arm of a wheel for rotation.
    """

    vx = float(vx)
    vy = float(vy)
    yaw_speed = float(yaw_radius_m) * float(wz)
    return (
        (vx - vy) * XDRIVE_TRANSLATION_SCALE - yaw_speed,
        (vx + vy) * XDRIVE_TRANSLATION_SCALE - yaw_speed,
        (-vx + vy) * XDRIVE_TRANSLATION_SCALE - yaw_speed,
        (-vx - vy) * XDRIVE_TRANSLATION_SCALE - yaw_speed,
    )


def limit_planar_velocity(vx: float, vy: float, max_planar_speed_m_s: float):
    """Apply a circular body-speed cap while preserving travel direction."""

    vx = float(vx)
    vy = float(vy)
    limit = max(0.0, float(max_planar_speed_m_s))
    speed = math.hypot(vx, vy)
    if limit <= 0.0 or speed <= limit or speed <= 1.0e-9:
        return vx, vy, 1.0
    scale = limit / speed
    return vx * scale, vy * scale, scale


def limit_yaw_rate_by_motion(
    vx: float,
    vy: float,
    wz: float,
    heading_correction_max_wz_rad_s: float,
    active_rotation_max_wz_rad_s: float,
    active_rotation_linear_threshold_m_s: float,
):
    """Use separate angular limits for translating and near-stationary motion.

    Nav2's GoalAngleCritic supplies the closed-loop yaw correction.  While the
    chassis is translating that command is treated as heading hold.  Only
    near-stationary commands may use the larger active-rotation limit.
    """

    translating = math.hypot(float(vx), float(vy)) > max(
        0.0, float(active_rotation_linear_threshold_m_s)
    )
    mode = "heading_correction" if translating else "active_rotation"
    limit = (
        heading_correction_max_wz_rad_s
        if translating
        else active_rotation_max_wz_rad_s
    )
    limit = max(0.0, float(limit))
    wz = float(wz)
    return max(-limit, min(limit, wz)), mode


def limit_xdrive_command(
    vx: float,
    vy: float,
    wz: float,
    max_planar_speed_m_s: float,
    max_wheel_speed_m_s: float,
    yaw_radius_m: float,
    heading_correction_max_wz_rad_s: float,
    active_rotation_max_wz_rad_s: float,
    active_rotation_linear_threshold_m_s: float,
):
    """Project a body command into the physical X-drive feasible set.

    The planar direction is preserved.  Yaw is limited according to the
    current motion mode, then given wheel-speed priority so a heading error is
    corrected instead of being carried down the bed corridor.  Translation is
    reduced only when the requested yaw leaves insufficient wheel headroom.
    """

    vx, vy, planar_scale = limit_planar_velocity(
        vx, vy, max_planar_speed_m_s
    )
    wz, yaw_mode = limit_yaw_rate_by_motion(
        vx,
        vy,
        wz,
        heading_correction_max_wz_rad_s,
        active_rotation_max_wz_rad_s,
        active_rotation_linear_threshold_m_s,
    )

    wheel_limit = max(0.0, float(max_wheel_speed_m_s))
    radius = max(0.0, float(yaw_radius_m))
    if wheel_limit <= 0.0:
        requested_peak = max(
            abs(speed) for speed in omni_wheel_speeds(vx, vy, wz, radius)
        )
        return (
            vx,
            vy,
            wz,
            planar_scale,
            1.0,
            requested_peak,
            yaw_mode,
        )

    yaw_load = radius * abs(wz)
    if yaw_load > wheel_limit and radius > 0.0:
        wz = math.copysign(wheel_limit / radius, wz)
        yaw_load = wheel_limit

    requested_peak = max(
        abs(speed) for speed in omni_wheel_speeds(vx, vy, wz, radius)
    )
    translation_peak = max(
        abs(speed) for speed in omni_wheel_speeds(vx, vy, 0.0, radius)
    )
    translation_headroom = max(0.0, wheel_limit - yaw_load)
    wheel_scale = 1.0
    if translation_peak > translation_headroom and translation_peak > 1.0e-9:
        wheel_scale = translation_headroom / translation_peak
        vx *= wheel_scale
        vy *= wheel_scale

    return (
        vx,
        vy,
        wz,
        planar_scale,
        wheel_scale,
        requested_peak,
        yaw_mode,
    )


def normalize_omni_command(
    vx: float,
    vy: float,
    wz: float,
    max_wheel_speed_m_s: float,
    yaw_radius_m: float,
):
    requested_wheels = omni_wheel_speeds(vx, vy, wz, yaw_radius_m)
    requested_peak = max(abs(speed) for speed in requested_wheels)
    wheel_limit = max(0.0, float(max_wheel_speed_m_s))
    if wheel_limit <= 0.0 or requested_peak <= wheel_limit:
        return float(vx), float(vy), float(wz), 1.0, requested_peak
    scale = wheel_limit / requested_peak
    return (
        float(vx) * scale,
        float(vy) * scale,
        float(wz) * scale,
        scale,
        requested_peak,
    )


class GateReleaseLimiter:
    """Continuously limit wheel-space acceleration after safety processing.

    The class name and ``rearm_drop_m_s`` argument are retained for launch and
    diagnostic compatibility.  The limiter is now permanently armed whenever
    ``max_wheel_accel_m_s2`` is positive: stops and reductions pass
    immediately, while every increase in the peak wheel-surface speed is
    ramped with one common scale so the requested chassis direction is kept.
    """

    def __init__(
        self,
        max_wheel_accel_m_s2: float,
        yaw_radius_m: float,
        rearm_drop_m_s: float = 0.0,
    ) -> None:
        self.max_wheel_accel_m_s2 = max(0.0, float(max_wheel_accel_m_s2))
        self.yaw_radius_m = max(0.0, float(yaw_radius_m))
        self.rearm_drop_m_s = max(0.0, float(rearm_drop_m_s))
        self.output = (0.0, 0.0, 0.0)
        self.last_update_s = None
        self.active = self.max_wheel_accel_m_s2 > 0.0
        self.limiting = False

    def reset(self, now_s=None) -> None:
        self.output = (0.0, 0.0, 0.0)
        self.last_update_s = None if now_s is None else float(now_s)
        self.active = self.max_wheel_accel_m_s2 > 0.0
        self.limiting = False

    def _wheel_speeds(self, command):
        vx, vy, wz = command
        return omni_wheel_speeds(vx, vy, wz, self.yaw_radius_m)

    def update(self, vx: float, vy: float, wz: float, now_s: float):
        desired = (float(vx), float(vy), float(wz))
        now_s = float(now_s)
        desired_wheels = self._wheel_speeds(desired)
        desired_peak = max(abs(speed) for speed in desired_wheels)

        if self.max_wheel_accel_m_s2 <= 0.0:
            self.output = desired
            self.last_update_s = now_s
            self.active = False
            self.limiting = False
            return self.output

        self.active = True
        current_wheels = self._wheel_speeds(self.output)
        current_peak = max(abs(speed) for speed in current_wheels)

        # Stops and safety-commanded reductions must never wait for a ramp.
        if desired_peak <= current_peak:
            self.output = desired
            self.last_update_s = now_s
            self.limiting = False
            return self.output

        if self.last_update_s is None:
            self.last_update_s = now_s
            self.limiting = True
            return self.output

        dt = max(0.0, now_s - self.last_update_s)
        self.last_update_s = now_s
        wheel_delta = tuple(
            desired_speed - current_speed
            for desired_speed, current_speed in zip(desired_wheels, current_wheels)
        )
        peak_delta = max(abs(delta) for delta in wheel_delta)
        if peak_delta == 0.0:
            self.output = desired
            self.limiting = False
            return self.output

        allowed_delta = self.max_wheel_accel_m_s2 * dt
        alpha = min(1.0, allowed_delta / peak_delta)
        self.output = tuple(
            current + alpha * (target - current)
            for current, target in zip(self.output, desired)
        )
        self.limiting = alpha < 1.0
        return self.output


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
