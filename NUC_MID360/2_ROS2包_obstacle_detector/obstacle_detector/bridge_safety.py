"""Pure motion authorization rules for the STM32 bridge."""

import math

from .nav_protocol import (
    GOAL_NONE,
    NAV_FOLLOWING,
    NAV_IDLE,
    NAV_WAIT_PATH,
    TASK_NAV_BED1,
    TASK_NAV_BED3,
    TASK_NAV_HOME,
    TASK_WAIT_START,
)


NAVIGATION_TASK_STATES = frozenset((1, 3, 6, 9))
DEFAULT_HEADING_HOLD_TASK_STATES = frozenset(
    (TASK_NAV_BED1, TASK_NAV_BED3, TASK_NAV_HOME)
)
XDRIVE_TRANSLATION_SCALE = 1.0 / math.sqrt(2.0)


class Stm32ReadinessGate:
    """Debounce STM32 telemetry loss and recovery around an intentional reset.

    A hard reset removes both pose and wheel telemetry for several seconds.
    The NUC must block motion and perception immediately, but it must not reopen
    the lidar pipeline on the first boot frame.  Requiring a continuous healthy,
    stationary interval prevents a partially booted controller from starting a
    new mission or publishing a discontinuous pose into an uncleared costmap.
    """

    def __init__(
        self,
        recovery_hold_s: float = 0.75,
        stationary_linear_m_s: float = 0.03,
        stationary_angular_rad_s: float = 0.05,
    ) -> None:
        self.recovery_hold_s = max(0.0, float(recovery_hold_s))
        self.stationary_linear_m_s = max(
            0.0, float(stationary_linear_m_s)
        )
        self.stationary_angular_rad_s = max(
            0.0, float(stationary_angular_rad_s)
        )
        self.ready = False
        self.state = "waiting_telemetry"
        self.recovery_started_s = None
        self.transitions = 0

    def update(
        self,
        now_s: float,
        pose_fresh: bool,
        wheel_telemetry_fresh: bool,
        linear_speed_m_s: float,
        angular_speed_rad_s: float,
        recovery_allowed: bool = True,
    ) -> bool:
        telemetry_healthy = bool(pose_fresh and wheel_telemetry_fresh)
        stationary = (
            abs(float(linear_speed_m_s))
            <= self.stationary_linear_m_s
            and abs(float(angular_speed_rad_s))
            <= self.stationary_angular_rad_s
        )

        if not telemetry_healthy:
            self.recovery_started_s = None
            self.state = "rebooting" if self.ready else "waiting_telemetry"
            if self.ready:
                self.ready = False
                self.transitions += 1
            return self.ready

        if self.ready:
            self.state = "ready"
            return True

        if not recovery_allowed:
            self.recovery_started_s = None
            self.state = "waiting_reset_state"
            return False

        if not stationary:
            self.recovery_started_s = None
            self.state = "waiting_stationary"
            return False

        if self.recovery_started_s is None:
            self.recovery_started_s = float(now_s)
            self.state = "recovering"
            return False

        if float(now_s) - self.recovery_started_s < self.recovery_hold_s:
            self.state = "recovering"
            return False

        self.ready = True
        self.state = "ready"
        self.transitions += 1
        return True


def normalize_angle_rad(angle: float) -> float:
    """Wrap an angle to the shortest signed displacement in radians."""

    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


class HeadingHoldController:
    """Independent HWT-yaw PID used while Nav2 supplies planar velocity.

    The controller follows the same target-minus-measurement and shortest-angle
    convention as STM32 ``ChassisCtrl_MoveTarget``.  Its output is ROS ``wz``:
    positive is counter-clockwise.  Task states outside ``task_states`` retain
    MPPI's angular command, which keeps nurse-station scan viewpoints free to
    rotate.
    """

    def __init__(
        self,
        enabled: bool = True,
        task_states=DEFAULT_HEADING_HOLD_TASK_STATES,
        target_yaw_deg: float = 0.0,
        kp: float = 1.5,
        ki: float = 0.0,
        kd: float = 0.0,
        deadband_deg: float = 1.0,
        max_wz_rad_s: float = 0.30,
        integral_limit_rad_s: float = 0.05,
        reverse_min_speed_m_s: float = 1.0,
        reverse_full_speed_m_s: float = 2.5,
        reverse_kp: float = 2.2,
        reverse_kd: float = 0.45,
        reverse_max_wz_rad_s: float = 0.40,
        reverse_hold_settle_s: float = 0.20,
        yaw_rate_deadband_rad_s: float = 0.02,
    ) -> None:
        self.enabled = bool(enabled)
        self.task_states = frozenset(int(state) for state in task_states)
        self.target_yaw_rad = math.radians(float(target_yaw_deg))
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.deadband_rad = math.radians(max(0.0, float(deadband_deg)))
        self.max_wz_rad_s = max(0.0, float(max_wz_rad_s))
        self.integral_limit_rad_s = max(
            0.0, float(integral_limit_rad_s)
        )
        self.reverse_min_speed_m_s = max(
            0.0, float(reverse_min_speed_m_s)
        )
        self.reverse_full_speed_m_s = max(
            self.reverse_min_speed_m_s + 1.0e-6,
            float(reverse_full_speed_m_s),
        )
        self.reverse_kp = float(reverse_kp)
        self.reverse_kd = max(0.0, float(reverse_kd))
        self.reverse_max_wz_rad_s = max(
            self.max_wz_rad_s, float(reverse_max_wz_rad_s)
        )
        self.reverse_hold_settle_s = max(
            0.0, float(reverse_hold_settle_s)
        )
        self.yaw_rate_deadband_rad_s = max(
            0.0, float(yaw_rate_deadband_rad_s)
        )
        self.task_state = None
        self.integral_output = 0.0
        self.previous_error_rad = None
        self.last_update_s = None
        self.last_profile = "normal"
        self.last_reverse_blend = 0.0
        self.last_effective_kp = self.kp
        self.last_effective_kd = self.kd
        self.last_effective_max_wz_rad_s = self.max_wz_rad_s
        self.last_yaw_rate_rad_s = None
        self.reverse_hold_active = False
        self.reverse_hold_settle_started_s = None

    @property
    def target_yaw_deg(self) -> float:
        return math.degrees(self.target_yaw_rad)

    def reset(self) -> None:
        self.reverse_hold_active = False
        self.reverse_hold_settle_started_s = None
        self._reset_pid_state()

    def _reset_pid_state(self) -> None:
        self.integral_output = 0.0
        self.previous_error_rad = None
        self.last_update_s = None

    def _effective_control(
        self,
        forward_velocity_m_s: float,
        error_rad: float,
        measured_yaw_rate,
        now_s: float,
    ):
        reverse_speed = max(0.0, -float(forward_velocity_m_s))
        reverse_blend = max(
            0.0,
            min(
                1.0,
                (reverse_speed - self.reverse_min_speed_m_s)
                / (self.reverse_full_speed_m_s - self.reverse_min_speed_m_s),
            ),
        )
        if reverse_speed >= self.reverse_full_speed_m_s:
            self.reverse_hold_active = True
            self.reverse_hold_settle_started_s = None
        elif self.reverse_hold_active:
            yaw_rate_stable = (
                measured_yaw_rate is not None
                and abs(measured_yaw_rate) <= self.yaw_rate_deadband_rad_s
            )
            heading_stable = abs(error_rad) <= self.deadband_rad
            reverse_motion_ended = reverse_speed < self.reverse_min_speed_m_s
            if reverse_motion_ended and heading_stable and yaw_rate_stable:
                if self.reverse_hold_settle_started_s is None:
                    self.reverse_hold_settle_started_s = now_s
                elif (
                    now_s - self.reverse_hold_settle_started_s
                    >= self.reverse_hold_settle_s
                ):
                    self.reverse_hold_active = False
                    self.reverse_hold_settle_started_s = None
            else:
                self.reverse_hold_settle_started_s = None
        if self.reverse_hold_active:
            reverse_blend = 1.0
        effective_kp = self.kp + reverse_blend * (self.reverse_kp - self.kp)
        effective_kd = self.kd + reverse_blend * (self.reverse_kd - self.kd)
        effective_max_wz = self.max_wz_rad_s + reverse_blend * (
            self.reverse_max_wz_rad_s - self.max_wz_rad_s
        )
        if self.reverse_hold_active and reverse_speed < self.reverse_full_speed_m_s:
            self.last_profile = "high_speed_reverse_hold"
        else:
            self.last_profile = "high_speed_reverse" if reverse_blend > 0.0 else "normal"
        self.last_reverse_blend = reverse_blend
        self.last_effective_kp = effective_kp
        self.last_effective_kd = effective_kd
        self.last_effective_max_wz_rad_s = effective_max_wz
        return effective_kp, effective_kd, effective_max_wz

    def update(
        self,
        task_state: int,
        mppi_wz_rad_s: float,
        current_yaw_rad,
        pose_fresh: bool,
        now_s: float,
        allow_override: bool = True,
        forward_velocity_m_s: float = 0.0,
        yaw_rate_rad_s=None,
    ):
        """Return ``(wz, mode, error)`` without touching MPPI ``vx``/``vy``."""

        task_state = int(task_state)
        mppi_wz_rad_s = float(mppi_wz_rad_s)
        now_s = float(now_s)
        if task_state != self.task_state:
            self.reset()
            self.task_state = task_state

        if (
            not allow_override
            or not self.enabled
            or task_state not in self.task_states
        ):
            self.reset()
            return mppi_wz_rad_s, "mppi", None

        if (
            not pose_fresh
            or current_yaw_rad is None
            or not math.isfinite(float(current_yaw_rad))
        ):
            self.reset()
            return 0.0, "heading_hold_no_pose", None

        error_rad = normalize_angle_rad(
            self.target_yaw_rad - float(current_yaw_rad)
        )
        measured_yaw_rate = (
            float(yaw_rate_rad_s)
            if yaw_rate_rad_s is not None
            and math.isfinite(float(yaw_rate_rad_s))
            else None
        )
        self.last_yaw_rate_rad_s = measured_yaw_rate
        effective_kp, effective_kd, effective_max_wz = self._effective_control(
            forward_velocity_m_s,
            error_rad,
            measured_yaw_rate,
            now_s,
        )
        damping_output = 0.0
        if (
            measured_yaw_rate is not None
            and abs(measured_yaw_rate) > self.yaw_rate_deadband_rad_s
        ):
            damping_output = -effective_kd * measured_yaw_rate

        inside_deadband = abs(error_rad) <= self.deadband_rad
        if inside_deadband and damping_output == 0.0:
            self._reset_pid_state()
            self.previous_error_rad = error_rad
            self.last_update_s = now_s
            mode = (
                "heading_hold_reverse_settling"
                if self.reverse_hold_active
                else "heading_hold_deadband"
            )
            return 0.0, mode, error_rad

        control_error_rad = 0.0 if inside_deadband else error_rad

        dt = None
        if self.last_update_s is not None:
            candidate_dt = now_s - self.last_update_s
            if 0.0 < candidate_dt <= 0.5:
                dt = candidate_dt

        derivative_output = damping_output
        if (
            measured_yaw_rate is None
            and dt is not None
            and self.previous_error_rad is not None
        ):
            derivative_output = (
                effective_kd
                * normalize_angle_rad(error_rad - self.previous_error_rad)
                / dt
            )

        candidate_integral = self.integral_output
        if dt is not None and self.ki != 0.0:
            candidate_integral += self.ki * control_error_rad * dt
            candidate_integral = max(
                -self.integral_limit_rad_s,
                min(self.integral_limit_rad_s, candidate_integral),
            )
        else:
            candidate_integral = 0.0 if self.ki == 0.0 else candidate_integral

        unsaturated = (
            effective_kp * control_error_rad
            + candidate_integral
            + derivative_output
        )
        output = max(
            -effective_max_wz,
            min(effective_max_wz, unsaturated),
        )
        # Keep the integral only if it is not pushing farther into saturation.
        if (
            abs(unsaturated) <= effective_max_wz
            or control_error_rad * unsaturated <= 0.0
        ):
            self.integral_output = candidate_integral
        self.previous_error_rad = error_rad
        self.last_update_s = now_s
        mode = (
            "heading_hold_reverse_hold"
            if self.reverse_hold_active
            and max(0.0, -float(forward_velocity_m_s))
            < self.reverse_full_speed_m_s
            else "heading_hold_reverse"
            if self.last_reverse_blend > 0.0
            else "heading_hold"
        )
        return output, mode, error_rad


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
    Independent heading hold may set ``priority_yaw``: translation keeps this
    current-protection ramp, while the bounded yaw correction passes directly
    instead of arriving only after the 20 kg chassis has accumulated error.
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

    def update(
        self,
        vx: float,
        vy: float,
        wz: float,
        now_s: float,
        priority_yaw: bool = False,
    ):
        desired = (float(vx), float(vy), float(wz))
        now_s = float(now_s)
        priority_yaw = bool(priority_yaw)
        ramp_desired = (
            (desired[0], desired[1], 0.0) if priority_yaw else desired
        )
        desired_wheels = self._wheel_speeds(ramp_desired)
        desired_peak = max(abs(speed) for speed in desired_wheels)

        if self.max_wheel_accel_m_s2 <= 0.0:
            self.output = desired
            self.last_update_s = now_s
            self.active = False
            self.limiting = False
            return self.output

        self.active = True
        ramp_current = (
            (self.output[0], self.output[1], 0.0)
            if priority_yaw
            else self.output
        )
        current_wheels = self._wheel_speeds(ramp_current)
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
        ramped_output = tuple(
            current + alpha * (target - current)
            for current, target in zip(ramp_current, ramp_desired)
        )
        self.output = (
            ramped_output[0],
            ramped_output[1],
            desired[2] if priority_yaw else ramped_output[2],
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
