import math
import unittest

from obstacle_detector.bridge_safety import (
    GateReleaseLimiter,
    HeadingHoldController,
    CollisionStopGate,
    SettledStopDetector,
    Stm32ReadinessGate,
    limit_axis_velocity,
    limit_crossdrive_command,
    limit_yaw_rate_by_motion,
    medical_mission_restarted,
    mcu_timestamp_restarted,
    start_gate_status,
    navigation_motion_is_authorized,
    crossdrive_wheel_speeds,
)
from obstacle_detector.nav_protocol import (
    FrameParser,
    GOAL_BED1,
    GOAL_BED3,
    NAV_FOLLOWING,
    NAV_IDLE,
    NAV_WAIT_PATH,
    MSG_EMERGENCY_STOP,
    MSG_EMERGENCY_CLEAR,
    TASK_NAV_BED1,
    TASK_NAV_BED3,
    TASK_NAV_HOME,
    TASK_WAIT_START,
    encode_emergency_stop,
    encode_emergency_clear,
    encode_frame,
)


class BridgeSafetyTest(unittest.TestCase):
    def test_emergency_stop_frame_carries_timestamp_and_uses_distinct_type(self):
        frames = FrameParser().feed(
            encode_frame(MSG_EMERGENCY_STOP, 7, encode_emergency_stop(0x1234))
        )
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].msg_type, MSG_EMERGENCY_STOP)
        self.assertEqual(frames[0].payload, b"\x12\x34")

    def test_emergency_clear_frame_carries_timestamp_and_uses_distinct_type(self):
        frames = FrameParser().feed(
            encode_frame(MSG_EMERGENCY_CLEAR, 8, encode_emergency_clear(0x5678))
        )
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].msg_type, MSG_EMERGENCY_CLEAR)
        self.assertEqual(frames[0].payload, b"\x56\x78")

    def test_collision_stop_requires_recent_moving_input_and_zero_safe_output(self):
        gate = CollisionStopGate(input_timeout_s=0.3, hold_s=0.5)
        gate.update_input((1.0, 0.0, 0.0), 1.0)
        self.assertTrue(gate.is_collision_stop((0.0, 0.0, 0.0), 1.1))
        self.assertTrue(gate.is_collision_stop((0.0, 0.0, 0.0), 1.5))
        self.assertFalse(gate.is_collision_stop((0.0, 0.0, 0.0), 1.61))
        self.assertFalse(gate.is_collision_stop((0.0, 0.0, 0.0), 2.0))

    def test_normal_zero_input_and_partial_slowdown_are_not_collision_stop(self):
        gate = CollisionStopGate()
        gate.update_input((0.0, 0.0, 0.0), 1.0)
        self.assertFalse(gate.is_collision_stop((0.0, 0.0, 0.0), 1.01))
        gate.update_input((0.0, 0.0, 0.6), 1.1)
        self.assertFalse(gate.is_collision_stop((0.0, 0.0, 0.3), 1.11))
        self.assertTrue(gate.is_collision_stop((0.0, 0.0, 0.0), 1.12))
        gate.update_input((0.0, 0.0, 0.0), 1.2)
        self.assertFalse(gate.is_collision_stop((0.0, 0.0, 0.0), 1.21))

    def test_stm32_readiness_requires_stable_stationary_recovery(self):
        gate = Stm32ReadinessGate(recovery_hold_s=0.75)

        self.assertFalse(gate.update(1.0, False, False, 0.0, 0.0))
        self.assertEqual(gate.state, "waiting_telemetry")
        self.assertFalse(gate.update(2.0, True, True, 0.20, 0.0))
        self.assertEqual(gate.state, "waiting_stationary")
        self.assertFalse(gate.update(3.0, True, True, 0.0, 0.0))
        self.assertFalse(gate.update(3.74, True, True, 0.0, 0.0))
        self.assertTrue(gate.update(3.75, True, True, 0.0, 0.0))
        self.assertEqual(gate.state, "ready")

    def test_stm32_readiness_closes_immediately_on_telemetry_loss(self):
        gate = Stm32ReadinessGate(recovery_hold_s=0.1)

        gate.update(1.0, True, True, 0.0, 0.0)
        self.assertTrue(gate.update(1.1, True, True, 0.0, 0.0))
        self.assertFalse(gate.update(1.11, False, False, 0.0, 0.0))
        self.assertEqual(gate.state, "rebooting")

    def test_stm32_recovery_waits_for_reset_wait_state(self):
        gate = Stm32ReadinessGate(recovery_hold_s=0.1)

        self.assertFalse(
            gate.update(1.0, True, True, 0.0, 0.0, recovery_allowed=False)
        )
        self.assertEqual(gate.state, "waiting_reset_state")
        self.assertFalse(
            gate.update(2.0, True, True, 0.0, 0.0, recovery_allowed=True)
        )
        self.assertTrue(
            gate.update(2.1, True, True, 0.0, 0.0, recovery_allowed=True)
        )

    def test_heading_hold_zero_and_deadband_return_zero(self):
        controller = HeadingHoldController(deadband_deg=1.0)

        zero = controller.update(
            TASK_NAV_BED1, 0.6, 0.0, True, 1.0
        )
        inside_deadband = controller.update(
            TASK_NAV_BED1, -0.6, math.radians(-0.8), True, 1.1
        )

        self.assertEqual(zero[:2], (0.0, "heading_hold_deadband"))
        self.assertEqual(
            inside_deadband[:2], (0.0, "heading_hold_deadband")
        )

    def test_clockwise_field_yaw_produces_counter_clockwise_ros_wz(self):
        controller = HeadingHoldController(deadband_deg=0.0)

        positive_field_yaw = controller.update(
            TASK_NAV_BED1,
            -0.7,
            # HWT field yaw is clockwise-positive, so ROS yaw is negative.
            math.radians(-10.0),
            True,
            1.0,
        )
        negative_field_yaw = controller.update(
            TASK_NAV_BED1,
            0.7,
            math.radians(10.0),
            True,
            1.1,
        )

        self.assertGreater(positive_field_yaw[0], 0.0)
        self.assertAlmostEqual(
            positive_field_yaw[0], 1.5 * math.radians(10.0)
        )
        self.assertLess(negative_field_yaw[0], 0.0)
        self.assertAlmostEqual(
            negative_field_yaw[0], -1.5 * math.radians(10.0)
        )

    def test_heading_hold_wraps_shortest_path_across_180_degrees(self):
        controller = HeadingHoldController(
            target_yaw_deg=179.0,
            deadband_deg=0.0,
            max_wz_rad_s=1.0,
        )

        output, mode, error = controller.update(
            TASK_NAV_BED3, 0.0, math.radians(-179.0), True, 1.0
        )

        self.assertEqual(mode, "heading_hold")
        self.assertAlmostEqual(math.degrees(error), -2.0)
        self.assertAlmostEqual(math.degrees(output), -3.0)

    def test_heading_hold_saturates_at_configured_yaw_rate(self):
        controller = HeadingHoldController(
            kp=2.0, deadband_deg=0.0, max_wz_rad_s=0.30
        )

        output, mode, _ = controller.update(
            TASK_NAV_HOME, 0.0, math.radians(-45.0), True, 1.0
        )

        self.assertEqual(mode, "heading_hold")
        self.assertAlmostEqual(output, 0.30)

    def test_high_speed_reverse_blends_stronger_pd_heading_control(self):
        controller = HeadingHoldController(
            deadband_deg=0.0,
            reverse_min_speed_m_s=1.0,
            reverse_full_speed_m_s=2.5,
            reverse_kp=2.2,
            reverse_kd=0.30,
            reverse_max_wz_rad_s=0.40,
        )

        reverse = controller.update(
            TASK_NAV_HOME,
            0.0,
            math.radians(-4.0),
            True,
            1.0,
            forward_velocity_m_s=-2.5,
            yaw_rate_rad_s=-0.20,
        )

        expected = 2.2 * math.radians(4.0) + 0.30 * 0.20
        self.assertEqual(reverse[1], "heading_hold_reverse")
        self.assertAlmostEqual(reverse[0], expected)
        self.assertEqual(controller.last_profile, "high_speed_reverse")
        self.assertAlmostEqual(controller.last_reverse_blend, 1.0)
        self.assertAlmostEqual(controller.last_effective_max_wz_rad_s, 0.40)

    def test_forward_motion_keeps_base_heading_profile(self):
        controller = HeadingHoldController(
            deadband_deg=0.0,
            reverse_kp=2.2,
            reverse_kd=0.30,
        )

        forward = controller.update(
            TASK_NAV_HOME,
            0.0,
            math.radians(-4.0),
            True,
            1.0,
            forward_velocity_m_s=2.5,
            yaw_rate_rad_s=-0.20,
        )

        self.assertEqual(forward[1], "heading_hold")
        self.assertAlmostEqual(forward[0], 1.5 * math.radians(4.0))
        self.assertEqual(controller.last_profile, "normal")

    def test_reverse_yaw_damping_acts_inside_angle_deadband(self):
        controller = HeadingHoldController(
            deadband_deg=1.0,
            reverse_min_speed_m_s=1.0,
            reverse_full_speed_m_s=2.5,
            reverse_kd=0.30,
            yaw_rate_deadband_rad_s=0.02,
        )

        output, mode, error = controller.update(
            TASK_NAV_HOME,
            0.0,
            math.radians(-0.5),
            True,
            1.0,
            forward_velocity_m_s=-2.5,
            yaw_rate_rad_s=-0.20,
        )

        self.assertEqual(mode, "heading_hold_reverse")
        self.assertAlmostEqual(math.degrees(error), 0.5)
        self.assertAlmostEqual(output, 0.30 * 0.20)

    def test_high_speed_reverse_uses_raised_yaw_limit(self):
        controller = HeadingHoldController(
            deadband_deg=0.0,
            reverse_min_speed_m_s=1.0,
            reverse_full_speed_m_s=2.5,
            reverse_kp=2.2,
            reverse_max_wz_rad_s=0.40,
        )

        output, mode, _ = controller.update(
            TASK_NAV_HOME,
            0.0,
            math.radians(-20.0),
            True,
            1.0,
            forward_velocity_m_s=-2.5,
            yaw_rate_rad_s=-0.20,
        )

        self.assertEqual(mode, "heading_hold_reverse")
        self.assertAlmostEqual(output, 0.40)

    def test_reverse_pd_remains_latched_through_braking(self):
        controller = HeadingHoldController(
            deadband_deg=1.0,
            reverse_min_speed_m_s=1.0,
            reverse_full_speed_m_s=2.5,
            reverse_kp=2.2,
            reverse_kd=0.45,
            reverse_hold_settle_s=0.20,
        )

        controller.update(
            TASK_NAV_HOME,
            0.0,
            math.radians(-4.0),
            True,
            1.0,
            forward_velocity_m_s=-2.5,
            yaw_rate_rad_s=-0.20,
        )
        output, mode, _ = controller.update(
            TASK_NAV_HOME,
            0.0,
            math.radians(-4.0),
            True,
            1.1,
            forward_velocity_m_s=0.0,
            yaw_rate_rad_s=-0.20,
        )

        expected = 2.2 * math.radians(4.0) + 0.45 * 0.20
        self.assertEqual(mode, "heading_hold_reverse_hold")
        self.assertAlmostEqual(output, expected)
        self.assertTrue(controller.reverse_hold_active)
        self.assertEqual(controller.last_profile, "high_speed_reverse_hold")
        self.assertAlmostEqual(controller.last_reverse_blend, 1.0)

    def test_reverse_pd_releases_only_after_stable_settle_time(self):
        controller = HeadingHoldController(
            deadband_deg=1.0,
            reverse_min_speed_m_s=1.0,
            reverse_full_speed_m_s=2.5,
            reverse_hold_settle_s=0.20,
            yaw_rate_deadband_rad_s=0.02,
        )
        controller.update(
            TASK_NAV_HOME,
            0.0,
            0.0,
            True,
            1.0,
            forward_velocity_m_s=-2.5,
            yaw_rate_rad_s=0.0,
        )

        first = controller.update(
            TASK_NAV_HOME, 0.0, 0.0, True, 1.1,
            forward_velocity_m_s=0.0, yaw_rate_rad_s=0.0,
        )
        settling = controller.update(
            TASK_NAV_HOME, 0.0, 0.0, True, 1.25,
            forward_velocity_m_s=0.0, yaw_rate_rad_s=0.0,
        )
        released = controller.update(
            TASK_NAV_HOME, 0.0, 0.0, True, 1.31,
            forward_velocity_m_s=0.0, yaw_rate_rad_s=0.0,
        )

        self.assertEqual(first[1], "heading_hold_reverse_settling")
        self.assertEqual(settling[1], "heading_hold_reverse_settling")
        self.assertEqual(released[1], "heading_hold_deadband")
        self.assertFalse(controller.reverse_hold_active)

    def test_reverse_pd_keeps_damping_inside_heading_deadband(self):
        controller = HeadingHoldController(
            deadband_deg=1.0,
            reverse_min_speed_m_s=1.0,
            reverse_full_speed_m_s=2.5,
            reverse_kd=0.45,
            reverse_hold_settle_s=0.20,
            yaw_rate_deadband_rad_s=0.02,
        )
        controller.update(
            TASK_NAV_HOME,
            0.0,
            0.0,
            True,
            1.0,
            forward_velocity_m_s=-2.5,
            yaw_rate_rad_s=0.0,
        )

        output, mode, _ = controller.update(
            TASK_NAV_HOME,
            0.0,
            math.radians(0.5),
            True,
            1.1,
            forward_velocity_m_s=0.0,
            yaw_rate_rad_s=0.10,
        )

        self.assertEqual(mode, "heading_hold_reverse_hold")
        self.assertAlmostEqual(output, -0.045)
        self.assertTrue(controller.reverse_hold_active)
        self.assertIsNone(controller.reverse_hold_settle_started_s)

    def test_heading_hold_states_override_only_wz_and_require_fresh_pose(self):
        controller = HeadingHoldController(deadband_deg=0.0)
        vx = 1.2
        vy = -0.7

        held_wz, held_mode, _ = controller.update(
            TASK_NAV_BED1, -0.4, math.radians(-8.0), True, 1.0
        )
        command = (vx, vy, held_wz)
        stale_wz, stale_mode, _ = controller.update(
            TASK_NAV_BED1, 0.4, None, False, 1.1
        )

        self.assertEqual(command[:2], (vx, vy))
        self.assertEqual(held_mode, "heading_hold")
        self.assertNotAlmostEqual(held_wz, -0.4)
        self.assertEqual((stale_wz, stale_mode), (0.0, "heading_hold_no_pose"))

    def test_nurse_scan_suppresses_heading_correction(self):
        controller = HeadingHoldController(deadband_deg=0.0)

        nurse = controller.update(1, 0.55, math.radians(20.0), True, 1.0)
        wait = controller.update(
            TASK_WAIT_START, -0.35, math.radians(20.0), True, 1.1
        )

        self.assertEqual(nurse[1], "navigation")
        self.assertAlmostEqual(nurse[0], 0.55)
        self.assertEqual(wait[:2], (-0.35, "navigation"))

    def test_stop_sources_bypass_heading_override(self):
        controller = HeadingHoldController(deadband_deg=0.0)

        output = controller.update(
            TASK_NAV_BED1,
            0.0,
            math.radians(-20.0),
            True,
            1.0,
            allow_override=False,
        )

        self.assertEqual(output[:2], (0.0, "navigation"))

    def test_crossdrive_assigns_translation_to_independent_wheel_pairs(self):
        wheels = crossdrive_wheel_speeds(2.0, 1.5, 0.0, 0.25)

        self.assertEqual(wheels, (2.0, 1.5, -2.0, -1.5))

    def test_axis_limit_does_not_normalize_diagonal_motion(self):
        vx, vy, scale = limit_axis_velocity(2.0, 2.0, 2.0)

        self.assertEqual((vx, vy, scale), (2.0, 2.0, 1.0))
        self.assertAlmostEqual(math.hypot(vx, vy), 2.0 * math.sqrt(2.0))

    def test_axis_limit_clamps_each_component_independently(self):
        vx, vy, scale = limit_axis_velocity(4.0, -1.5, 3.2)

        self.assertEqual((vx, vy), (3.2, -1.5))
        self.assertAlmostEqual(scale, 0.8)

    def test_yaw_limits_separate_heading_hold_from_active_rotation(self):
        heading_wz, heading_mode = limit_yaw_rate_by_motion(
            0.0, 1.0, 0.8, 0.2, 0.8, 0.05
        )
        rotation_wz, rotation_mode = limit_yaw_rate_by_motion(
            0.0, 0.0, 0.8, 0.2, 0.8, 0.05
        )

        self.assertEqual(heading_mode, "heading_correction")
        self.assertAlmostEqual(heading_wz, 0.2)
        self.assertEqual(rotation_mode, "active_rotation")
        self.assertAlmostEqual(rotation_wz, 0.8)

    def test_crossdrive_command_keeps_two_m_per_s_on_both_axes(self):
        output = limit_crossdrive_command(
            2.0, 2.0, 0.0, 2.0, 0.25, 0.2, 0.8, 0.05
        )
        vx, vy, wz, axis_scale, headroom_scale, requested_peak, yaw_mode = output

        self.assertEqual((vx, vy), (2.0, 2.0))
        self.assertEqual(wz, 0.0)
        self.assertEqual(axis_scale, 1.0)
        self.assertEqual(headroom_scale, 1.0)
        self.assertAlmostEqual(requested_peak, 2.0)
        self.assertEqual(yaw_mode, "heading_correction")

    def test_three_m_per_s_cap_is_independent_and_wheel_feasible(self):
        output = limit_crossdrive_command(
            3.0, 3.0, 0.0, 3.0, 0.25, 0.3, 0.8, 0.05
        )
        vx, vy, wz, axis_scale, headroom_scale, requested_peak, _ = output

        self.assertEqual((vx, vy), (3.0, 3.0))
        self.assertAlmostEqual(math.hypot(vx, vy), 3.0 * math.sqrt(2.0))
        self.assertEqual(axis_scale, 1.0)
        self.assertEqual(headroom_scale, 1.0)
        self.assertAlmostEqual(requested_peak, 3.0)
        self.assertEqual(wz, 0.0)

    def test_heading_correction_keeps_both_axes_when_headroom_is_available(self):
        output = limit_crossdrive_command(
            2.0 ** 0.5,
            2.0 ** 0.5,
            0.2,
            2.0,
            0.25,
            0.2,
            0.8,
            0.05,
        )
        vx, vy, wz, _, headroom_scale, _, yaw_mode = output

        self.assertAlmostEqual(wz, 0.2)
        self.assertEqual(headroom_scale, 1.0)
        self.assertAlmostEqual(vx, 2.0 ** 0.5)
        self.assertAlmostEqual(vy, 2.0 ** 0.5)
        self.assertEqual(yaw_mode, "heading_correction")
        self.assertAlmostEqual(
            max(
                abs(value)
                for value in crossdrive_wheel_speeds(vx, vy, wz, 0.25)
            ),
            2.0 ** 0.5 + 0.05,
        )

    def test_yaw_headroom_clamps_only_the_axis_that_needs_it(self):
        output = limit_crossdrive_command(
            2.0, 0.5, 0.2, 2.0, 0.25, 0.2, 0.8, 0.05
        )
        vx, vy, wz, axis_scale, headroom_scale, _, _ = output

        self.assertAlmostEqual(vx, 1.95)
        self.assertAlmostEqual(vy, 0.5)
        self.assertAlmostEqual(wz, 0.2)
        self.assertEqual(axis_scale, 1.0)
        self.assertAlmostEqual(headroom_scale, 0.975)
        self.assertAlmostEqual(
            max(
                abs(value)
                for value in crossdrive_wheel_speeds(vx, vy, wz, 0.25)
            ),
            2.0,
        )

    def test_gate_release_limiter_uses_common_wheel_space_scale(self):
        limiter = GateReleaseLimiter(2.0, 0.25)
        limiter.reset(1.0)

        output = limiter.update(1.0, 0.5, 0.4, 1.1)

        self.assertAlmostEqual(output[0] / output[1], 2.0)
        self.assertAlmostEqual(output[0] / output[2], 2.5)
        self.assertAlmostEqual(
            max(abs(value) for value in limiter._wheel_speeds(output)), 0.2
        )
        self.assertTrue(limiter.active)
        self.assertTrue(limiter.limiting)

    def test_heading_yaw_bypasses_translation_acceleration_ramp(self):
        limiter = GateReleaseLimiter(2.8, 0.25)
        limiter.reset(1.0)

        output = limiter.update(
            -2.5,
            0.0,
            -0.4,
            1.02,
            priority_yaw=True,
        )

        self.assertGreater(output[0], -2.5)
        self.assertAlmostEqual(output[2], -0.4)
        self.assertTrue(limiter.limiting)

        reversed_yaw = limiter.update(
            -2.5,
            0.0,
            0.3,
            1.04,
            priority_yaw=True,
        )
        self.assertAlmostEqual(reversed_yaw[2], 0.3)

    def test_gate_release_limiter_stays_armed_after_catchup(self):
        limiter = GateReleaseLimiter(2.0, 0.25)
        limiter.reset(1.0)

        self.assertEqual(limiter.update(1.0, 0.0, 0.0, 1.5), (1.0, 0.0, 0.0))
        self.assertTrue(limiter.active)
        output = limiter.update(2.0, 0.0, 0.0, 1.51)
        self.assertGreater(output[0], 1.0)
        self.assertLess(output[0], 2.0)
        self.assertTrue(limiter.limiting)

    def test_gate_release_limiter_never_delays_stop_or_reduction(self):
        limiter = GateReleaseLimiter(2.0, 0.25, 0.25)
        limiter.reset(1.0)
        limiter.update(1.0, 0.5, 0.4, 1.1)

        self.assertEqual(limiter.update(0.05, 0.0, 0.0, 1.11), (0.05, 0.0, 0.0))
        self.assertTrue(limiter.active)
        self.assertEqual(limiter.update(0.0, 0.0, 0.0, 1.12), (0.0, 0.0, 0.0))

    def test_gate_release_limiter_drops_forbidden_axis_during_diagonal_release(self):
        limiter = GateReleaseLimiter(2.0, 0.25)
        limiter.reset(1.0)
        self.assertEqual(limiter.update(1.0, 0.0, 0.0, 2.0), (1.0, 0.0, 0.0))

        output = limiter.update(0.0, 1.0, 0.0, 2.05)
        self.assertEqual(output[0], 0.0)
        self.assertGreater(output[1], 0.0)
        self.assertLess(output[1], 1.0)
        self.assertLessEqual(
            max(abs(value) for value in limiter._wheel_speeds(output)), 0.1 + 1e-9
        )

    def test_gate_release_limiter_respects_tighter_axis_during_other_axis_accel(self):
        limiter = GateReleaseLimiter(2.0, 0.25)
        limiter.reset(1.0)
        limiter.update(1.0, 0.0, 0.0, 2.0)

        output = limiter.update(0.4, 1.0, 0.0, 2.05)
        self.assertEqual(output[0], 0.4)
        self.assertGreater(output[1], 0.0)
        self.assertLess(output[1], 1.0)

    def test_gate_release_limiter_bounds_equal_peak_reversal(self):
        limiter = GateReleaseLimiter(2.0, 0.25)
        limiter.reset(1.0)
        limiter.update(1.0, 0.0, 0.0, 2.0)

        output = limiter.update(-1.0, 0.0, 0.0, 2.05)
        self.assertLess(output[0], 0.0)
        self.assertGreater(output[0], -1.0)
        self.assertLessEqual(
            max(abs(value) for value in limiter._wheel_speeds(output)), 0.1 + 1e-9
        )

    def test_gate_release_limiter_zero_command_is_immediate(self):
        limiter = GateReleaseLimiter(2.0, 0.25)
        limiter.reset(1.0)
        limiter.update(1.0, 0.7, 0.2, 2.0)
        self.assertEqual(limiter.update(0.0, 0.0, 0.0, 2.01), (0.0, 0.0, 0.0))

    def test_gate_release_limiter_ramps_after_safety_slowdown(self):
        limiter = GateReleaseLimiter(2.0, 0.25, 0.25)
        limiter.reset(1.0)
        output = limiter.update(1.0, 0.0, 0.0, 1.5)
        self.assertAlmostEqual(output[0], 1.0)
        self.assertEqual(output[1:], (0.0, 0.0))
        self.assertTrue(limiter.active)

        self.assertEqual(limiter.update(0.4, 0.0, 0.0, 1.6), (0.4, 0.0, 0.0))
        self.assertTrue(limiter.active)
        output = limiter.update(1.0, 0.0, 0.0, 1.8)
        self.assertGreater(output[0], 0.4)
        self.assertLess(output[0], 1.0)
        self.assertEqual(output[1:], (0.0, 0.0))
        self.assertTrue(limiter.limiting)

    def test_gate_release_limiter_catches_recovery_after_gradual_slowdown(self):
        limiter = GateReleaseLimiter(2.5, 0.25, 0.25)
        limiter.reset(1.0)
        self.assertEqual(limiter.update(1.0, 0.0, 0.0, 1.5), (1.0, 0.0, 0.0))
        self.assertTrue(limiter.active)

        self.assertEqual(limiter.update(0.85, 0.0, 0.0, 1.6), (0.85, 0.0, 0.0))
        self.assertEqual(limiter.update(0.70, 0.0, 0.0, 1.7), (0.70, 0.0, 0.0))
        self.assertTrue(limiter.active)
        output = limiter.update(1.0, 0.0, 0.0, 1.71)
        self.assertGreater(output[0], 0.70)
        self.assertLess(output[0], 1.0)
        self.assertTrue(limiter.limiting)

    def test_gate_release_limiter_can_be_disabled_transparently(self):
        limiter = GateReleaseLimiter(0.0, 0.25)
        limiter.reset(1.0)

        self.assertEqual(
            limiter.update(1.5, -0.4, 0.3, 1.001),
            (1.5, -0.4, 0.3),
        )
        # With the downstream ramp disabled, reversal and a safety stop also
        # reach their new targets immediately, without waiting for elapsed dt.
        self.assertEqual(
            limiter.update(-1.5, 0.4, -0.3, 1.001),
            (-1.5, 0.4, -0.3),
        )
        self.assertEqual(limiter.update(0.0, 0.0, 0.0, 1.001), (0.0, 0.0, 0.0))
        self.assertFalse(limiter.active)
        self.assertFalse(limiter.limiting)

    def test_gate_release_limiter_restarts_after_authorization_is_revoked(self):
        limiter = GateReleaseLimiter(2.0, 0.25)
        limiter.reset(1.0)
        limiter.update(1.0, 0.0, 0.0, 1.5)
        limiter.update(2.0, 0.0, 0.0, 1.51)

        limiter.reset(2.0)

        output = limiter.update(1.0, 0.0, 0.0, 2.1)
        self.assertAlmostEqual(
            max(abs(value) for value in limiter._wheel_speeds(output)), 0.2
        )
        self.assertEqual(output[1:], (0.0, 0.0))
        self.assertTrue(limiter.limiting)

    def test_detects_stm32_restart_while_wait_state_is_unchanged(self):
        # A planning-state regression with uninterrupted pose telemetry is
        # not a reboot and must not erase the live goal request.
        self.assertFalse(
            medical_mission_restarted(
                TASK_WAIT_START,
                NAV_FOLLOWING,
                TASK_WAIT_START,
                NAV_WAIT_PATH,
                0.02,
                0.5,
            )
        )
        self.assertFalse(
            medical_mission_restarted(
                TASK_WAIT_START,
                NAV_FOLLOWING,
                TASK_WAIT_START,
                NAV_FOLLOWING,
                0.8,
                0.5,
            )
        )
        self.assertTrue(
            medical_mission_restarted(
                TASK_WAIT_START,
                NAV_FOLLOWING,
                TASK_WAIT_START,
                NAV_FOLLOWING,
                0.02,
                0.5,
                True,
            )
        )
        self.assertFalse(
            medical_mission_restarted(
                TASK_WAIT_START,
                NAV_FOLLOWING,
                TASK_WAIT_START,
                NAV_FOLLOWING,
                0.02,
                0.5,
            )
        )

    def test_mcu_timestamp_restart_detection_allows_uint32_wrap(self):
        self.assertTrue(mcu_timestamp_restarted(5000, 100))
        self.assertFalse(mcu_timestamp_restarted(0xFFFFFFF0, 20))
        self.assertFalse(mcu_timestamp_restarted(None, 20))
        self.assertFalse(
            medical_mission_restarted(
                TASK_WAIT_START,
                NAV_FOLLOWING,
                1,
                NAV_IDLE,
                1.0,
                0.5,
            )
        )

    def test_settled_stop_detector_waits_and_resets(self):
        detector = SettledStopDetector(0.03, 0.05, 0.15)

        self.assertFalse(detector.update(0.4, 0.0, 0.0, 1.0))
        self.assertFalse(detector.update(0.02, 0.0, 0.04, 1.1))
        self.assertFalse(detector.update(0.02, 0.0, 0.04, 1.24))
        self.assertTrue(detector.update(0.02, 0.0, 0.04, 1.251))
        self.assertFalse(detector.update(0.04, 0.0, 0.0, 1.3))
        self.assertFalse(detector.update(0.0, 0.0, 0.0, 1.4))

    def test_matching_active_goal_allows_navigation(self):
        self.assertTrue(
            navigation_motion_is_authorized(
                True, True, True, True, 3, NAV_FOLLOWING, 12, GOAL_BED1,
                12, GOAL_BED1, NAV_FOLLOWING,
            )
        )

    def test_old_goal_status_is_rejected_during_handoff(self):
        self.assertFalse(
            navigation_motion_is_authorized(
                True, True, True, True, 6, NAV_FOLLOWING, 13, GOAL_BED3,
                12, GOAL_BED1, NAV_FOLLOWING,
            )
        )

    def test_waiting_path_and_stale_pose_are_rejected(self):
        self.assertFalse(
            navigation_motion_is_authorized(
                True, True, True, True, 3, NAV_FOLLOWING, 12, GOAL_BED1,
                12, GOAL_BED1, NAV_WAIT_PATH,
            )
        )
        self.assertFalse(
            navigation_motion_is_authorized(
                False, True, True, True, 3, NAV_FOLLOWING, 12, GOAL_BED1,
                12, GOAL_BED1, NAV_FOLLOWING,
            )
        )

    def test_handoff_hold_rejects_first_new_goal_commands(self):
        self.assertFalse(
            navigation_motion_is_authorized(
                True, True, False, True, 3, NAV_FOLLOWING, 12, GOAL_BED1,
                12, GOAL_BED1, NAV_FOLLOWING,
            )
        )

    def test_stale_navigator_status_revokes_motion(self):
        self.assertFalse(
            navigation_motion_is_authorized(
                True, False, True, True, 3, NAV_FOLLOWING, 12, GOAL_BED1,
                12, GOAL_BED1, NAV_FOLLOWING,
            )
        )

    def test_new_goal_following_hold_rejects_residual_command(self):
        self.assertFalse(
            navigation_motion_is_authorized(
                True, True, True, False, 3, NAV_FOLLOWING, 12, GOAL_BED1,
                12, GOAL_BED1, NAV_FOLLOWING,
            )
        )


class StartGateTest(unittest.TestCase):
    def setUp(self):
        self.goal_gate = {
            "requested_request_id": 1, "requested_goal_id": GOAL_BED1,
            "navigator_request_id": 1, "navigator_goal_id": GOAL_BED1,
            "navigator_fresh": True, "plan_ready": True,
            "navigator_state": NAV_FOLLOWING,
        }

    def gate(self, **kwargs):
        values = dict(task_state=15, pose_nav_status=NAV_FOLLOWING,
                      pose_fresh=True, stm32_ready=True, stm32_gate_state="ready",
                      serial_connected=True, goal_gate=self.goal_gate)
        values.update(kwargs)
        return start_gate_status(**values)

    def test_ready_requires_matching_fresh_plan_and_mcu_ack(self):
        self.assertTrue(self.gate()["can_start"])
        self.assertFalse(self.gate(pose_fresh=False)["can_start"])
        self.assertFalse(self.gate(stm32_ready=False)["can_start"])
        self.assertFalse(self.gate(pose_nav_status=NAV_WAIT_PATH)["can_start"])
        for key, value in (("navigator_request_id", 2),
                           ("navigator_goal_id", GOAL_BED3),
                           ("navigator_fresh", False), ("plan_ready", False)):
            with self.subTest(key=key):
                gate = dict(self.goal_gate, **{key: value})
                self.assertFalse(self.gate(goal_gate=gate)["can_start"])

    def test_wait_states_can_be_ready_but_never_authorize_motion(self):
        for task in (14, 15, 16):
            self.assertTrue(self.gate(task_state=task)["can_start"])
            self.assertFalse(navigation_motion_is_authorized(
                True, True, True, True, task, NAV_FOLLOWING,
                1, GOAL_BED1, 1, GOAL_BED1, NAV_FOLLOWING))

    def test_reason_exposes_blocking_stage(self):
        self.assertEqual(self.gate(serial_connected=False)["reason"], "serial_disconnected")
        self.assertEqual(self.gate(pose_fresh=False)["reason"], "waiting_pose")
        self.assertEqual(self.gate(pose_nav_status=NAV_WAIT_PATH)["reason"],
                         "waiting_stm32_nav_ack")
        gate = dict(self.goal_gate, plan_ready=False)
        self.assertEqual(self.gate(goal_gate=gate, navigator_result="waiting_for_nav2_active")
                         ["reason"], "navigator:waiting_for_nav2_active")


if __name__ == "__main__":
    unittest.main()
