import math
import unittest

from obstacle_detector.bridge_safety import (
    GateReleaseLimiter,
    HeadingHoldController,
    SettledStopDetector,
    Stm32ReadinessGate,
    limit_planar_velocity,
    limit_xdrive_command,
    limit_yaw_rate_by_motion,
    medical_mission_restarted,
    navigation_motion_is_authorized,
    normalize_omni_command,
    omni_wheel_speeds,
)
from obstacle_detector.nav_protocol import (
    GOAL_BED1,
    GOAL_BED3,
    NAV_FOLLOWING,
    NAV_IDLE,
    NAV_WAIT_PATH,
    TASK_NAV_BED1,
    TASK_NAV_BED3,
    TASK_NAV_HOME,
    TASK_WAIT_START,
)


class BridgeSafetyTest(unittest.TestCase):
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

    def test_nurse_and_non_hold_states_retain_mppi_yaw(self):
        controller = HeadingHoldController(deadband_deg=0.0)

        nurse = controller.update(1, 0.55, math.radians(20.0), True, 1.0)
        wait = controller.update(
            TASK_WAIT_START, -0.35, math.radians(20.0), True, 1.1
        )

        self.assertEqual(nurse[:2], (0.55, "mppi"))
        self.assertEqual(wait[:2], (-0.35, "mppi"))

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

        self.assertEqual(output[:2], (0.0, "mppi"))

    def test_xdrive_uses_physical_45_degree_wheel_projection(self):
        axial = omni_wheel_speeds(2.0, 0.0, 0.0, 0.25)
        diagonal = omni_wheel_speeds(2.0 ** 0.5, 2.0 ** 0.5, 0.0, 0.25)

        self.assertAlmostEqual(max(abs(value) for value in axial), 2.0 ** 0.5)
        self.assertAlmostEqual(max(abs(value) for value in diagonal), 2.0)

    def test_wheel_normalization_leaves_single_axis_motion_unchanged(self):
        output = normalize_omni_command(2.0, 0.0, 0.0, 2.0, 0.25)
        self.assertEqual(output[:4], (2.0, 0.0, 0.0, 1.0))
        self.assertAlmostEqual(
            max(
                abs(value)
                for value in omni_wheel_speeds(*output[:3], 0.25)
            ),
            2.0 ** 0.5,
        )

    def test_wheel_normalization_scales_diagonal_motion_together(self):
        vx, vy, wz, scale, requested_peak = normalize_omni_command(
            2.0, 1.5, 0.0, 2.0, 0.25
        )
        expected_peak = 3.5 / (2.0 ** 0.5)
        self.assertAlmostEqual(requested_peak, expected_peak)
        self.assertAlmostEqual(scale, 2.0 / expected_peak)
        self.assertAlmostEqual(vx / vy, 2.0 / 1.5)
        self.assertAlmostEqual(
            max(abs(value) for value in omni_wheel_speeds(vx, vy, wz, 0.25)),
            2.0,
        )

    def test_planar_limit_projects_square_command_to_two_m_per_s_circle(self):
        vx, vy, scale = limit_planar_velocity(2.0, 2.0, 2.0)

        self.assertAlmostEqual(vx, 2.0 ** 0.5)
        self.assertAlmostEqual(vy, 2.0 ** 0.5)
        self.assertAlmostEqual(scale, 1.0 / (2.0 ** 0.5))
        self.assertAlmostEqual((vx * vx + vy * vy) ** 0.5, 2.0)

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

    def test_xdrive_command_reaches_two_m_per_s_in_diagonal_direction(self):
        output = limit_xdrive_command(
            2.0, 2.0, 0.0, 2.0, 2.0, 0.25, 0.2, 0.8, 0.05
        )
        vx, vy, wz, planar_scale, wheel_scale, requested_peak, yaw_mode = output

        self.assertAlmostEqual(vx, 2.0 ** 0.5)
        self.assertAlmostEqual(vy, 2.0 ** 0.5)
        self.assertEqual(wz, 0.0)
        self.assertAlmostEqual(planar_scale, 1.0 / (2.0 ** 0.5))
        self.assertEqual(wheel_scale, 1.0)
        self.assertAlmostEqual(requested_peak, 2.0)
        self.assertEqual(yaw_mode, "heading_correction")

    def test_three_m_per_s_cap_is_circular_and_wheel_feasible(self):
        output = limit_xdrive_command(
            3.0, 3.0, 0.0, 3.0, 3.0, 0.25, 0.3, 0.8, 0.05
        )
        vx, vy, wz, planar_scale, wheel_scale, requested_peak, _ = output

        self.assertAlmostEqual(math.hypot(vx, vy), 3.0)
        self.assertAlmostEqual(vx, 3.0 / math.sqrt(2.0))
        self.assertAlmostEqual(vy, 3.0 / math.sqrt(2.0))
        self.assertAlmostEqual(planar_scale, 1.0 / math.sqrt(2.0))
        self.assertAlmostEqual(wheel_scale, 1.0)
        self.assertAlmostEqual(requested_peak, 3.0)
        self.assertEqual(wz, 0.0)

    def test_heading_correction_keeps_yaw_and_reduces_translation_for_headroom(self):
        output = limit_xdrive_command(
            2.0 ** 0.5,
            2.0 ** 0.5,
            0.2,
            2.0,
            2.0,
            0.25,
            0.2,
            0.8,
            0.05,
        )
        vx, vy, wz, _, wheel_scale, _, yaw_mode = output

        self.assertAlmostEqual(wz, 0.2)
        self.assertAlmostEqual(wheel_scale, 0.975)
        self.assertAlmostEqual((vx * vx + vy * vy) ** 0.5, 1.95)
        self.assertEqual(yaw_mode, "heading_correction")
        self.assertAlmostEqual(
            max(abs(value) for value in omni_wheel_speeds(vx, vy, wz, 0.25)),
            2.0,
        )

    def test_wheel_normalization_includes_yaw_load(self):
        vx, vy, wz, scale, _ = normalize_omni_command(
            1.5, 0.5, 3.0, 2.0, 0.25
        )
        self.assertLess(scale, 1.0)
        self.assertAlmostEqual(
            max(abs(value) for value in omni_wheel_speeds(vx, vy, wz, 0.25)),
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
        self.assertTrue(
            medical_mission_restarted(
                TASK_WAIT_START,
                NAV_FOLLOWING,
                TASK_WAIT_START,
                NAV_WAIT_PATH,
                0.02,
                0.5,
            )
        )
        self.assertTrue(
            medical_mission_restarted(
                TASK_WAIT_START,
                NAV_FOLLOWING,
                TASK_WAIT_START,
                NAV_FOLLOWING,
                0.8,
                0.5,
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


if __name__ == "__main__":
    unittest.main()
