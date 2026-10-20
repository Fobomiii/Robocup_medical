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
    START_WAIT_TASK_STATES,
)


NAVIGATION_TASK_STATES = frozenset((1, 3, 6, 9))
# Nurse scan is translation-only for the current validation run.  Its DWB
# route may contain a non-zero angular sample, but the bridge must not pass
# that sample to the chassis.  Heading hold remains available for bed/home
# travel where keeping the calibrated chassis heading is still required.
DEFAULT_HEADING_HOLD_TASK_STATES = frozenset(
    (TASK_NAV_BED1, TASK_NAV_BED3, TASK_NAV_HOME)
)
class CollisionStopGate:
    """Identify a monitor stop without mistaking normal zero input for one."""

    def __init__(self, input_timeout_s: float = 0.3, hold_s: float = 0.5):
        self.input_timeout_s = max(0.0, float(input_timeout_s))
        self.hold_s = max(0.0, float(hold_s))
        self.input_cmd = (0.0, 0.0, 0.0)
        self.input_time_s = None
        self.stop_time_s = None

    @staticmethod
    def is_zero(command):
        return all(abs(float(value)) < 1.0e-4 for value in command)

    def update_input(self, command, now_s):
        self.input_cmd = tuple(float(value) for value in command)
        self.input_time_s = float(now_s)

    def is_collision_stop(self, safe_command, now_s):
        now_s = float(now_s)
        if not self.is_zero(safe_command):
            self.stop_time_s = None
            return False
        fresh_input = (
            self.input_time_s is not None
            and 0.0 <= now_s - self.input_time_s <= self.input_timeout_s
        )
        if fresh_input and not self.is_zero(self.input_cmd):
            self.stop_time_s = now_s
        if fresh_input and self.is_zero(self.input_cmd):
            self.stop_time_s = None
        return (
            self.stop_time_s is not None
            and now_s - self.stop_time_s <= self.hold_s
        )


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
    positive is counter-clockwise. Task states outside ``task_states`` retain
    the navigation angular command. Nurse scan rotation is disabled by the
    bridge before this controller is called.
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
        nav_wz_rad_s: float,
        current_yaw_rad,
        pose_fresh: bool,
        now_s: float,
        allow_override: bool = True,
        forward_velocity_m_s: float = 0.0,
        yaw_rate_rad_s=None,
    ):
        """Return ``(wz, mode, error)`` without changing navigation X/Y."""

        task_state = int(task_state)
        nav_wz_rad_s = float(nav_wz_rad_s)
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
            return nav_wz_rad_s, "navigation", None

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


def crossdrive_wheel_speeds(
    vx: float, vy: float, wz: float, yaw_radius_m: float
):
    """Return wheel-surface speeds for the physical cross layout.

    ``vx`` and ``vy`` are ROS body forward and left velocities.  CAN IDs
    1/3 are the left/right pair that drives ``vx``; IDs 2/4 are the rear/front
    pair that drives ``vy``.  The two translation axes are independent, so a
    diagonal command is not normalized by its planar magnitude.
    """

    vx = float(vx)
    vy = float(vy)
    yaw_speed = float(yaw_radius_m) * float(wz)
    return (
        vx - yaw_speed,
        vy - yaw_speed,
        -vx - yaw_speed,
        -vy - yaw_speed,
    )


def limit_axis_velocity(vx: float, vy: float, max_axis_speed_m_s: float):
    """Clamp body X/Y independently without planar-vector normalization."""

    vx = float(vx)
    vy = float(vy)
    limit = max(0.0, float(max_axis_speed_m_s))
    if limit <= 0.0:
        return vx, vy, 1.0
    limited_vx = max(-limit, min(limit, vx))
    limited_vy = max(-limit, min(limit, vy))
    scales = [1.0]
    if abs(vx) > 1.0e-9:
        scales.append(abs(limited_vx / vx))
    if abs(vy) > 1.0e-9:
        scales.append(abs(limited_vy / vy))
    return limited_vx, limited_vy, min(scales)


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


def limit_crossdrive_command(
    vx: float,
    vy: float,
    wz: float,
    max_axis_speed_m_s: float,
    yaw_radius_m: float,
    heading_correction_max_wz_rad_s: float,
    active_rotation_max_wz_rad_s: float,
    active_rotation_linear_threshold_m_s: float,
):
    """Project a command into the cross-drive feasible set.

    X and Y are capped independently. Yaw is limited according to the current
    motion mode and receives wheel-speed headroom without scaling one
    translation axis merely because the other axis reached its limit.
    """

    vx, vy, axis_scale = limit_axis_velocity(
        vx, vy, max_axis_speed_m_s
    )
    wz, yaw_mode = limit_yaw_rate_by_motion(
        vx,
        vy,
        wz,
        heading_correction_max_wz_rad_s,
        active_rotation_max_wz_rad_s,
        active_rotation_linear_threshold_m_s,
    )

    wheel_limit = max(0.0, float(max_axis_speed_m_s))
    radius = max(0.0, float(yaw_radius_m))
    if wheel_limit <= 0.0:
        requested_peak = max(
            abs(speed)
            for speed in crossdrive_wheel_speeds(vx, vy, wz, radius)
        )
        return (
            vx,
            vy,
            wz,
            axis_scale,
            1.0,
            requested_peak,
            yaw_mode,
        )

    yaw_load = radius * abs(wz)
    if yaw_load > wheel_limit and radius > 0.0:
        wz = math.copysign(wheel_limit / radius, wz)
        yaw_load = wheel_limit

    requested_peak = max(
        abs(speed) for speed in crossdrive_wheel_speeds(vx, vy, wz, radius)
    )
    translation_headroom = max(0.0, wheel_limit - yaw_load)
    limited_vx = max(-translation_headroom, min(translation_headroom, vx))
    limited_vy = max(-translation_headroom, min(translation_headroom, vy))
    headroom_scales = [1.0]
    if abs(vx) > 1.0e-9:
        headroom_scales.append(abs(limited_vx / vx))
    if abs(vy) > 1.0e-9:
        headroom_scales.append(abs(limited_vy / vy))
    headroom_scale = min(headroom_scales)

    return (
        limited_vx,
        limited_vy,
        wz,
        axis_scale,
        headroom_scale,
        requested_peak,
        yaw_mode,
    )


class GateReleaseLimiter:
    """Continuously limit wheel-space acceleration after safety processing.

    The class name and ``rearm_drop_m_s`` argument are retained for launch and
    diagnostic compatibility.  The limiter is permanently armed whenever
    ``max_wheel_accel_m_s2`` is positive: each axis reduction passes
    immediately, while increases are bounded by wheel-space delta (including
    equal-peak reversals).  A reversed axis drops its old direction first.
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
        return crossdrive_wheel_speeds(vx, vy, wz, self.yaw_radius_m)

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
        # Apply tightened per-axis bounds before ramping any remaining increase.
        # In particular, never interpolate a forbidden old direction back in.
        base = tuple(
            0.0 if current * target < 0.0 else
            target if abs(target) < abs(current) else current
            for current, target in zip(ramp_current, ramp_desired)
        )
        desired_wheels = self._wheel_speeds(ramp_desired)
        base_wheels = self._wheel_speeds(base)

        if self.last_update_s is None:
            self.last_update_s = now_s
            self.output = (base[0], base[1], desired[2] if priority_yaw else base[2])
            self.limiting = base != ramp_desired
            return self.output

        dt = max(0.0, now_s - self.last_update_s)
        self.last_update_s = now_s
        wheel_delta = tuple(
            desired_speed - current_speed
            for desired_speed, current_speed in zip(desired_wheels, base_wheels)
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
            for current, target in zip(base, ramp_desired)
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
    mcu_time_restarted: bool = False,
) -> bool:
    """Recognize a mission reset without treating a telemetry gap as reboot."""

    del pose_gap_s, pose_timeout_s

    if task_state != TASK_WAIT_START:
        return False
    if previous_task_state != TASK_WAIT_START:
        return True
    return bool(mcu_time_restarted)


def mcu_timestamp_restarted(previous_ms, current_ms) -> bool:
    """Return true for a real backward MCU clock jump, while allowing wrap."""

    if previous_ms in (None, 0) or current_ms == 0:
        return False
    delta = (int(current_ms) - int(previous_ms)) & 0xFFFFFFFF
    return delta >= 0x80000000


def start_gate_status(task_state, pose_nav_status, pose_fresh, stm32_ready,
                      stm32_gate_state, serial_connected, goal_gate,
                      navigator_result=""):
    """Report a fresh, request-matched gate, not a stale STM32 lamp value."""
    waiting = task_state in START_WAIT_TASK_STATES
    matched = (goal_gate["requested_goal_id"] != GOAL_NONE
               and goal_gate["requested_goal_id"] == goal_gate["navigator_goal_id"]
               and goal_gate["requested_request_id"] == goal_gate["navigator_request_id"])
    if not serial_connected:
        reason = "serial_disconnected"
    elif not pose_fresh:
        reason = "waiting_pose"
    elif not stm32_ready:
        reason = "stm32:" + str(stm32_gate_state)
    elif not waiting:
        reason = "not_waiting_for_start"
    elif goal_gate["requested_goal_id"] == GOAL_NONE:
        reason = "waiting_goal_request"
    elif not matched or not goal_gate["navigator_fresh"]:
        reason = "waiting_navigator"
    elif not goal_gate["plan_ready"] or goal_gate["navigator_state"] != NAV_FOLLOWING:
        reason = "navigator:" + (navigator_result or "waiting_path")
    elif pose_nav_status != NAV_FOLLOWING:
        reason = "waiting_stm32_nav_ack"
    else:
        reason = "ready"
    return {"waiting_for_button": waiting, "path_ready": reason == "ready",
            "can_start": reason == "ready", "reason": reason}


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
