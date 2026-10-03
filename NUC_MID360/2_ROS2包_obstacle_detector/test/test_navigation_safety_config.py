"""Static checks for the live-replanning and collision-monitor profile."""

import ast
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
PARAMS = PACKAGE / "config" / "nav2_params.yaml"
SCANNER_PARAMS = PACKAGE / "config" / "scanner.yaml"
BT_XML = PACKAGE / "config" / "navigate_to_pose_1hz.xml"
NURSE_BT_XML = PACKAGE / "config" / "navigate_to_pose_nurse_fast.xml"


class NavigationSafetyConfigTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = yaml.safe_load(PARAMS.read_text(encoding="utf-8"))

    def test_mppi_leaves_capacity_for_lidar_safety(self):
        params = self.config["controller_server"]["ros__parameters"]
        controller_names = [
            "FollowPathNurse",
            "FollowPathHeadingHold",
            "FollowPathLateralHold",
        ]
        self.assertEqual(params["controller_plugins"], controller_names)
        for controller_name in controller_names:
            follow_path = params[controller_name]
            self.assertEqual(
                follow_path["plugin"],
                "nav2_mppi_controller::MPPIController",
            )
            self.assertEqual(follow_path["model_dt"], 0.05)
            self.assertEqual(follow_path["batch_size"], 1200)
            for unsupported in ("ax_max", "ax_min", "ay_max", "az_max"):
                self.assertNotIn(unsupported, follow_path)
            self.assertEqual(follow_path["prune_distance"], 3.5)
            self.assertEqual(follow_path["temperature"], 0.25)
            self.assertEqual(
                follow_path["PathFollowCritic"]["offset_from_furthest"], 8
            )
            self.assertEqual(
                follow_path["PathAlignCritic"]["cost_weight"], 10.0
            )
            self.assertFalse(
                follow_path["CostCritic"]["consider_footprint"]
            )
            self.assertTrue(follow_path["visualize"])

        nurse = params["FollowPathNurse"]
        heading = params["FollowPathHeadingHold"]
        lateral = params["FollowPathLateralHold"]
        self.assertEqual(nurse["time_steps"], 40)
        self.assertEqual(lateral["time_steps"], 40)
        self.assertEqual(heading["time_steps"], 30)
        self.assertAlmostEqual(
            heading["time_steps"] * heading["model_dt"], 1.5
        )
        self.assertAlmostEqual(
            nurse["time_steps"] * nurse["model_dt"], 2.0
        )
        self.assertAlmostEqual(
            lateral["time_steps"] * lateral["model_dt"], 2.0
        )
        self.assertEqual(nurse["vx_std"], 0.30)
        self.assertEqual(heading["vx_std"], 0.80)
        self.assertEqual(lateral["vx_std"], 0.30)
        self.assertEqual(
            heading["PathFollowCritic"]["cost_weight"], 16.0
        )
        self.assertEqual(
            heading["PathFollowCritic"]["threshold_to_consider"], 0.50
        )
        self.assertEqual(
            nurse["PathFollowCritic"]["cost_weight"], 10.0
        )
        self.assertEqual(
            lateral["PathFollowCritic"]["cost_weight"], 10.0
        )
        self.assertEqual((nurse["vy_std"], nurse["wz_std"]), (0.28, 0.15))
        self.assertEqual(nurse["wz_max"], 0.80)
        self.assertEqual(
            (heading["vy_std"], heading["wz_std"], heading["wz_max"]),
            (0.28, 0.02, 0.05),
        )
        self.assertEqual(
            (lateral["vy_std"], lateral["wz_std"], lateral["wz_max"]),
            (0.40, 0.02, 0.05),
        )
        self.assertGreater(lateral["vy_std"], heading["vy_std"])

    def test_intentional_stm32_reset_gates_motion_and_lidar(self):
        launch_source = (
            PACKAGE / "launch" / "obstacle.launch.py"
        ).read_text(encoding="utf-8")
        bridge = (
            PACKAGE / "obstacle_detector" / "stm32_bridge.py"
        ).read_text(encoding="utf-8")
        lidar_filter = (
            PACKAGE / "obstacle_detector" / "lidar_transform.py"
        ).read_text(encoding="utf-8")
        navigator = (
            PACKAGE / "obstacle_detector" / "medical_navigator.py"
        ).read_text(encoding="utf-8")

        self.assertIn('"gate_on_stm32_ready": True', launch_source)
        self.assertIn('"stm32_telemetry_timeout_s": 0.25', launch_source)
        self.assertIn('"stm32_recovery_hold_s": 0.75', launch_source)
        self.assertIn('"/medical_nav/stm32_ready"', bridge)
        self.assertIn("Stm32ReadinessGate", bridge)
        self.assertIn("stm32_gate_drops", lidar_filter)
        self.assertIn("waiting_for_stm32_recovery", navigator)

    def test_omni_controller_holds_task_yaw_during_translation(self):
        params = self.config["controller_server"]["ros__parameters"]
        for controller_name in params["controller_plugins"]:
            follow_path = params[controller_name]
            self.assertEqual(follow_path["motion_model"], "Omni")
            self.assertEqual(follow_path["vx_max"], 3.00)
            self.assertEqual(follow_path["vx_min"], -3.00)
            self.assertEqual(follow_path["vy_max"], 3.00)
            self.assertFalse(follow_path["PathAngleCritic"]["enabled"])
            self.assertEqual(follow_path["PathAngleCritic"]["mode"], 1)
            self.assertTrue(follow_path["GoalAngleCritic"]["enabled"])
            self.assertEqual(
                follow_path["GoalAngleCritic"]["cost_weight"], 10.0
            )
            self.assertIn("TwirlingCritic", follow_path["critics"])

    def test_collision_monitor_restores_field_tested_single_stage_profile(self):
        normal_smoother = self.config["velocity_smoother"]["ros__parameters"]
        monitor = self.config["collision_monitor"]["ros__parameters"]
        local_costmap = self.config["local_costmap"]["local_costmap"][
            "ros__parameters"
        ]

        # Ordinary motion is ramp-limited before entering the safety chain.
        self.assertEqual(normal_smoother["max_accel"], [2.5, 2.2, 1.8])
        self.assertEqual(normal_smoother["max_decel"], [-2.0, -2.6, -2.0])
        self.assertEqual(normal_smoother["max_velocity"], [3.0, 3.0, 0.8])
        self.assertEqual(normal_smoother["min_velocity"], [-3.0, -3.0, -0.8])
        self.assertEqual((local_costmap["width"], local_costmap["height"]), (14, 14))
        local_livox = local_costmap["obstacle_layer"]["livox"]
        self.assertEqual(local_livox["obstacle_max_range"], 5.5)
        self.assertEqual(local_livox["raytrace_max_range"], 6.0)
        self.assertNotIn("collision_monitor_predictive", self.config)
        self.assertEqual(monitor["polygons"], ["ApproachPolygon", "StopPolygon"])
        self.assertEqual(monitor["cmd_vel_in_topic"], "/cmd_vel_home_limited")
        self.assertEqual(monitor["cmd_vel_out_topic"], "/cmd_vel_safe")
        self.assertEqual(monitor["source_timeout"], 0.3)
        self.assertNotIn("safety_velocity_smoother", self.config)

        approach = monitor["ApproachPolygon"]
        self.assertEqual(approach["action_type"], "approach")
        self.assertEqual(approach["time_before_collision"], 1.0)
        self.assertEqual(approach["simulation_time_step"], 0.1)
        self.assertEqual(approach["max_points"], 5)

        stop = monitor["StopPolygon"]
        self.assertEqual(stop["action_type"], "stop")
        self.assertEqual(stop["max_points"], 3)
        self.assertEqual(len(approach["points"]) % 2, 0)
        self.assertEqual(len(stop["points"]) % 2, 0)
        self.assertLess(max(abs(value) for value in stop["points"]), 0.30)

        launch_source = (PACKAGE / "launch" / "obstacle.launch.py").read_text(
            encoding="utf-8"
        )
        self.assertEqual(launch_source.count('package="nav2_collision_monitor"'), 1)
        self.assertNotIn("nav2_velocity_smoother", launch_source)
        self.assertNotIn('name="collision_monitor_predictive"', launch_source)
        self.assertIn('executable="home_approach_limiter"', launch_source)
        self.assertIn('name="home_approach_limiter"', launch_source)
        self.assertIn('executable="corner_speed_limiter"', launch_source)
        self.assertIn('name="corner_speed_limiter"', launch_source)
        self.assertIn('"raw_path_topic": "/plan"', launch_source)
        self.assertIn(
            '"smoothed_path_topic": "/transformed_global_plan"',
            launch_source,
        )
        self.assertIn('"output_topic": "/speed_limit"', launch_source)
        self.assertIn(
            '"controller_selector_topic": "/controller_selector"',
            launch_source,
        )
        self.assertIn(
            '"heading_hold_controller_id": "FollowPathHeadingHold"',
            launch_source,
        )
        self.assertIn('"lookahead_distance_m": 3.50', launch_source)
        self.assertIn('"braking_decel_m_s2": 1.80', launch_source)
        self.assertIn('"min_corner_speed_m_s": 1.0', launch_source)
        self.assertIn(
            '"heading_hold_min_corner_speed_m_s": 1.10', launch_source
        )
        self.assertIn(
            '"heading_hold_lateral_accel_m_s2": 1.60', launch_source
        )
        self.assertIn('"tangent_span_m": 0.75', launch_source)
        self.assertIn('"min_turn_angle_deg": 35.0', launch_source)
        self.assertIn('"input_topic": "/cmd_vel"', launch_source)
        self.assertIn('"output_topic": "/cmd_vel_home_limited"', launch_source)
        self.assertIn('"home_task_state": 9', launch_source)
        self.assertIn('"bed1_task_state": 3', launch_source)
        self.assertIn('"bed3_task_state": 6', launch_source)
        self.assertIn('"max_speed_m_s": 3.0', launch_source)
        self.assertIn('"soft_decel_m_s2": 4.9875', launch_source)
        self.assertIn('"terminal_speed_m_s": 0.10', launch_source)
        self.assertIn('"terminal_distance_m": 0.10', launch_source)
        self.assertIn('"decel_start_distance_m": 1.00', launch_source)
        self.assertIn('"bed_max_speed_m_s": 3.0', launch_source)
        self.assertIn('"wheel_odom_topic": "/medical_nav/wheel_odom"', launch_source)
        self.assertIn('"status_topic": "/medical_nav/bed_approach_status"', launch_source)
        self.assertIn('"bed_forward_decel_m_s2": 2.0', launch_source)
        self.assertIn('"bed_side_decel_m_s2": 2.2', launch_source)
        self.assertIn('"bed_reaction_time_s": 0.22', launch_source)
        self.assertIn('"bed_braking_margin_m": 0.03', launch_source)
        self.assertIn('"pose_timeout_s": 0.5', launch_source)
        self.assertIn('"gate_release_wheel_accel_m_s2": 2.8', launch_source)
        self.assertIn('"gate_release_yaw_radius_m": 0.25', launch_source)
        self.assertIn('"max_planar_speed_m_s": 3.0', launch_source)
        self.assertIn('"max_wheel_speed_m_s": 3.0', launch_source)
        self.assertIn('"max_speed_mm_s": 3000.0', launch_source)
        self.assertIn(
            '"heading_correction_max_wz_rad_s": 0.30', launch_source
        )
        self.assertIn(
            '"active_rotation_max_wz_rad_s": 0.80', launch_source
        )
        self.assertIn(
            '"active_rotation_linear_threshold_m_s": 0.05', launch_source
        )
        self.assertIn('"heading_hold_enabled": True', launch_source)
        self.assertIn(
            '"heading_hold_task_states": [3, 6, 9]', launch_source
        )
        self.assertIn('"heading_hold_target_deg": 0.0', launch_source)
        self.assertIn('"heading_hold_kp": 1.5', launch_source)
        self.assertIn('"heading_hold_ki": 0.0', launch_source)
        self.assertIn('"heading_hold_kd": 0.0', launch_source)
        self.assertIn('"heading_hold_deadband_deg": 1.0', launch_source)
        self.assertIn(
            '"heading_hold_max_wz_rad_s": 0.30', launch_source
        )
        self.assertIn(
            '"heading_hold_reverse_min_speed_m_s": 1.0', launch_source
        )
        self.assertIn(
            '"heading_hold_reverse_full_speed_m_s": 2.5', launch_source
        )
        self.assertIn('"heading_hold_reverse_kp": 2.2', launch_source)
        self.assertIn('"heading_hold_reverse_kd": 0.45', launch_source)
        self.assertIn(
            '"heading_hold_reverse_max_wz_rad_s": 0.40', launch_source
        )
        self.assertIn(
            '"heading_hold_reverse_hold_settle_s": 0.20', launch_source
        )
        self.assertIn(
            '"heading_hold_yaw_rate_deadband_rad_s": 0.02', launch_source
        )
        self.assertIn(
            '"gate_release_rearm_drop_m_s": 0.25', launch_source
        )
        self.assertNotIn('"collision_monitor_predictive",', launch_source)

        setup_source = (PACKAGE / "setup.py").read_text(encoding="utf-8")
        approach_source = (
            PACKAGE / "obstacle_detector" / "home_approach_limiter.py"
        ).read_text(encoding="utf-8")
        self.assertIn("scale_planar_velocity", approach_source)
        self.assertIn(
            "message.linear.x, message.linear.y, self.max_speed_m_s",
            approach_source,
        )
        self.assertIn(
            "home_approach_limiter = obstacle_detector.home_approach_limiter:main",
            setup_source,
        )
        self.assertIn(
            "corner_speed_limiter = obstacle_detector.corner_speed_limiter:main",
            setup_source,
        )

    def test_behavior_trees_use_bounded_replanning_rates(self):
        root = ET.parse(BT_XML).getroot()
        rate_controllers = root.findall(".//RateController")
        self.assertEqual(len(rate_controllers), 1)
        self.assertEqual(float(rate_controllers[0].attrib["hz"]), 1.0)
        smoothers = root.findall(".//SmoothPath")
        self.assertEqual(len(smoothers), 1)
        self.assertEqual(smoothers[0].attrib["smoother_id"], "simple_smoother")
        self.assertEqual(
            float(smoothers[0].attrib["max_smoothing_duration"]), 0.35
        )
        smooth_fallback = root.find(".//Fallback[@name='SmoothOrKeepOriginal']")
        self.assertIsNotNone(smooth_fallback)
        self.assertIsNotNone(smooth_fallback.find("SmoothPath"))
        self.assertIsNotNone(smooth_fallback.find("AlwaysSuccess"))
        plugin_names = self.config["bt_navigator"]["ros__parameters"][
            "plugin_lib_names"
        ]
        self.assertIn("nav2_smooth_path_action_bt_node", plugin_names)
        smoother = self.config["smoother_server"]["ros__parameters"][
            "simple_smoother"
        ]
        self.assertEqual(smoother["w_data"], 0.20)
        self.assertEqual(smoother["w_smooth"], 0.35)
        self.assertNotIn("error_code_id", BT_XML.read_text(encoding="utf-8"))

        recovery = root.find(".//ReactiveFallback[@name='RecoveryFallback']")
        self.assertIsNotNone(recovery)
        refresh = recovery.find("Sequence[@name='ClearCostmapsAndRefresh']")
        self.assertIsNotNone(refresh)
        self.assertEqual(len(refresh.findall("ClearEntireCostmap")), 2)
        self.assertEqual(refresh.findall("Wait"), [])
        self.assertEqual(root.findall(".//Spin"), [])
        self.assertEqual(root.findall(".//BackUp"), [])
        self.assertEqual(root.findall(".//RoundRobin"), [])

        nurse_root = ET.parse(NURSE_BT_XML).getroot()
        nurse_rate_controllers = nurse_root.findall(".//RateController")
        self.assertEqual(len(nurse_rate_controllers), 1)
        self.assertEqual(float(nurse_rate_controllers[0].attrib["hz"]), 2.0)
        self.assertEqual(
            root.find(".//ControllerSelector").attrib["default_controller"],
            "FollowPathHeadingHold",
        )
        self.assertEqual(
            nurse_root.find(".//ControllerSelector").attrib[
                "default_controller"
            ],
            "FollowPathNurse",
        )

        bt_params = self.config["bt_navigator"]["ros__parameters"]
        self.assertEqual(
            bt_params["default_nav_to_pose_bt_xml"], BT_XML.name
        )
        self.assertNotIn("default_bt_xml_filename", bt_params)

        launch_source = (PACKAGE / "launch" / "obstacle.launch.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("TimerAction(", launch_source)
        self.assertIn("period=3.0", launch_source)

    def test_nurse_scan_behavior_tree_has_no_motion_recovery(self):
        root = ET.parse(NURSE_BT_XML).getroot()
        self.assertEqual(root.findall(".//Spin"), [])
        self.assertEqual(root.findall(".//Wait"), [])
        self.assertEqual(root.findall(".//BackUp"), [])
        self.assertEqual(len(root.findall(".//ClearEntireCostmap")), 2)
        self.assertNotIn(
            "error_code_id", NURSE_BT_XML.read_text(encoding="utf-8")
        )

        navigator = (
            PACKAGE / "obstacle_detector" / "medical_navigator.py"
        ).read_text(encoding="utf-8")
        self.assertIn(
            "elif goal_id == self.nurse_goal_id:\n"
            "            self._send_next_nurse_viewpoint(generation)",
            navigator,
        )
        self.assertIn("request.behavior_tree = self.nurse_bt_xml", navigator)
        self.assertIn('"/controller_selector"', navigator)
        self.assertIn("controller_for_route(", navigator)
        self.assertIn(
            "self.create_timer(0.2, self._publish_controller_selection)",
            navigator,
        )
        self.assertIn('"controller_id": self.active_controller_id', navigator)
        self.assertIn('"controller_route": {', navigator)

    def test_global_far_noise_is_not_retained(self):
        global_livox = self.config["global_costmap"]["global_costmap"][
            "ros__parameters"
        ]["obstacle_layer"]["livox"]
        local_livox = self.config["local_costmap"]["local_costmap"][
            "ros__parameters"
        ]["obstacle_layer"]["livox"]

        global_params = self.config["global_costmap"]["global_costmap"][
            "ros__parameters"
        ]
        local_params = self.config["local_costmap"]["local_costmap"][
            "ros__parameters"
        ]
        planner_params = self.config["planner_server"]["ros__parameters"]

        self.assertEqual(local_params["width"], 14)
        self.assertEqual(local_params["height"], 14)
        self.assertEqual(local_params["robot_radius"], 0.23)
        self.assertEqual(global_params["robot_radius"], 0.23)
        local_footprint = ast.literal_eval(local_params["footprint"])
        global_footprint = ast.literal_eval(global_params["footprint"])
        self.assertEqual(local_footprint, global_footprint)
        self.assertEqual(len(local_footprint), 16)
        self.assertEqual(local_params["footprint_padding"], 0.0)
        self.assertEqual(global_params["footprint_padding"], 0.0)
        for x_value, y_value in local_footprint:
            self.assertAlmostEqual(
                (x_value ** 2 + y_value ** 2) ** 0.5, 0.23, places=3
            )
        self.assertEqual(global_params["update_frequency"], 4.0)
        self.assertEqual(planner_params["expected_planner_frequency"], 2.0)
        self.assertEqual(global_livox["observation_persistence"], 0.2)
        self.assertEqual(global_livox["obstacle_max_range"], 4.5)
        self.assertEqual(global_livox["raytrace_max_range"], 5.0)
        self.assertEqual(local_livox["observation_persistence"], 0.0)
        self.assertEqual(local_livox["obstacle_max_range"], 5.5)
        self.assertEqual(local_livox["raytrace_max_range"], 6.0)

    def test_real_chassis_output_remains_default(self):
        launch_source = (PACKAGE / "launch" / "obstacle.launch.py").read_text(
            encoding="utf-8"
        )
        self.assertIn(
            'DeclareLaunchArgument("dry_run", default_value="false")',
            launch_source,
        )
        self.assertIn("RewrittenYaml", launch_source)
        self.assertIn(
            '"default_nav_to_pose_bt_xml": bt_xml_file', launch_source
        )
        self.assertNotIn('"default_bt_xml_filename": bt_xml_file', launch_source)

    def test_lidar_odometry_starts_in_monitor_only_mode(self):
        guard = self.config["lidar_odometry_guard"]["ros__parameters"]
        self.assertTrue(guard["enabled"])
        self.assertTrue(guard["monitor_only"])
        self.assertFalse(guard["use_static_map_filter"])
        self.assertEqual(guard["process_period_s"], 0.5)
        self.assertLessEqual(guard["max_points"], 600)
        self.assertGreaterEqual(guard["required_bad_windows"], 3)

        launch_source = (PACKAGE / "launch" / "obstacle.launch.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('executable="lidar_odometry_guard"', launch_source)

    def test_cone_compensation_is_navigation_only(self):
        guard = self.config["lidar_odometry_guard"]["ros__parameters"]
        monitor = self.config["collision_monitor"]["ros__parameters"]
        local_livox = self.config["local_costmap"]["local_costmap"][
            "ros__parameters"
        ]["obstacle_layer"]["livox"]
        global_livox = self.config["global_costmap"]["global_costmap"][
            "ros__parameters"
        ]["obstacle_layer"]["livox"]

        self.assertEqual(guard["cloud_topic"], "/livox/lidar_filtered")
        self.assertEqual(monitor["livox"]["topic"], "/livox/lidar_safety")
        self.assertEqual(monitor["livox"]["min_height"], 0.12)
        self.assertEqual(local_livox["topic"], "/livox/lidar_nav")
        self.assertEqual(global_livox["topic"], "/livox/lidar_nav")

        launch_source = (PACKAGE / "launch" / "obstacle.launch.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('executable="cone_footprint_compensator"', launch_source)
        self.assertEqual(launch_source.count('executable="lidar_self_filter"'), 2)
        self.assertIn('name="lidar_safety_filter"', launch_source)
        self.assertIn('"output_topic": "/livox/lidar_safety"', launch_source)
        self.assertIn(
            '"status_topic": "/medical_nav/lidar_safety_filter_status"',
            launch_source,
        )
        self.assertIn('"output_min_z": 0.12', launch_source)
        self.assertIn('"ground_filter_enabled": False', launch_source)
        self.assertIn('"cone_height": 0.65', launch_source)
        self.assertIn('"physical_base_radius": 0.155', launch_source)
        self.assertIn('"base_radius": 0.18', launch_source)
        self.assertIn('"disk_spacing": 0.04', launch_source)
        self.assertIn('"min_points": 5', launch_source)
        self.assertIn('"min_vertical_span": 0.12', launch_source)
        self.assertIn('"center_merge_distance": 0.28', launch_source)
        self.assertIn('"max_range": 6.0', launch_source)
        self.assertIn('"persistence_s": 0.60', launch_source)
        self.assertIn('"persistence_frame": "odom"', launch_source)
        self.assertIn('"tf_future_fallback_enabled": True', launch_source)
        self.assertIn('"tf_future_fallback_max_gap_s": 0.20', launch_source)

    def test_navigation_cloud_chain_drops_backlog_for_low_latency(self):
        qos_source = (
            PACKAGE / "obstacle_detector" / "qos_profiles.py"
        ).read_text(encoding="utf-8")
        lidar_filter = (
            PACKAGE / "obstacle_detector" / "lidar_transform.py"
        ).read_text(encoding="utf-8")
        cone_compensator = (
            PACKAGE / "obstacle_detector" / "cone_footprint.py"
        ).read_text(encoding="utf-8")

        self.assertIn("history=HistoryPolicy.KEEP_LAST", qos_source)
        self.assertIn("depth=1", qos_source)
        self.assertIn(
            "reliability=ReliabilityPolicy.BEST_EFFORT", qos_source
        )
        self.assertIn("durability=DurabilityPolicy.VOLATILE", qos_source)
        self.assertGreaterEqual(
            lidar_filter.count("low_latency_sensor_qos()"), 2
        )
        self.assertGreaterEqual(
            cone_compensator.count("low_latency_sensor_qos()"), 2
        )
        self.assertNotIn("qos_profile_sensor_data", lidar_filter)
        self.assertNotIn("qos_profile_sensor_data", cone_compensator)

    def test_x_drive_blind_zones_are_visible_and_speed_guarded(self):
        launch_source = (PACKAGE / "launch" / "obstacle.launch.py").read_text(
            encoding="utf-8"
        )
        setup_source = (PACKAGE / "setup.py").read_text(encoding="utf-8")
        rviz_source = (
            PACKAGE.parent / "3_可视化工具" / "medical_nav.rviz"
        ).read_text(encoding="utf-8")

        self.assertIn('executable="blind_zone_visualizer"', launch_source)
        self.assertIn('"angles_deg": [45.0, 135.0, -135.0, -45.0]', launch_source)
        self.assertIn('"blind_zone_speed_m_s": 0.70', launch_source)
        self.assertIn('"blind_zone_min_overlap_m": 0.60', launch_source)
        self.assertIn("blind_zone_visualizer =", setup_source)
        self.assertIn("X-Drive Lidar Blind Zones", rviz_source)
        self.assertIn("/medical_nav/blind_zones", rviz_source)

    def test_goal_handoff_has_independent_stop_guards(self):
        root = PACKAGE.parents[1]
        medical_task = (root / "Program" / "Core" / "Src" / "MedicalTask.c").read_text(
            encoding="utf-8"
        )
        navigator = (PACKAGE / "obstacle_detector" / "medical_navigator.py").read_text(
            encoding="utf-8"
        )
        bridge = (PACKAGE / "obstacle_detector" / "stm32_bridge.py").read_text(
            encoding="utf-8"
        )
        launch_source = (PACKAGE / "launch" / "obstacle.launch.py").read_text(
            encoding="utf-8"
        )

        self.assertIn("MEDICAL_ORDER_HANDOFF_STOP_MS 500U", medical_task)
        self.assertIn("medical_set_state(MEDICAL_TASK_SCAN_ORDER);", medical_task)
        self.assertIn("waiting_for_cancel", navigator)
        self.assertIn("previous_handle.get_result_async()", navigator)
        self.assertIn("retrying cancellation", navigator)
        self.assertNotIn('last_result = "cancel_timeout"', navigator)
        self.assertIn('source="goal_handoff"', bridge)
        self.assertIn('source="navigator_following_hold"', bridge)
        self.assertIn("navigation_motion_is_authorized", bridge)
        self.assertIn('"goal_handoff_hold_s": 0.5', launch_source)
        self.assertIn('"navigator_following_hold_s": 0.3', launch_source)
        self.assertIn('"navigator_status_timeout_s": 1.2', launch_source)
        self.assertIn('"goal_handoff_timeout_s": 1.0', launch_source)

    def test_bridge_publishes_post_limiter_command_diagnostics(self):
        bridge = (
            PACKAGE / "obstacle_detector" / "stm32_bridge.py"
        ).read_text(encoding="utf-8")

        self.assertIn('"/medical_nav/bridge_cmd_debug"', bridge)
        self.assertIn("def _publish_bridge_cmd_debug(", bridge)
        self.assertIn('"safe_cmd"', bridge)
        self.assertIn('"release_limited_cmd"', bridge)
        self.assertIn('"sent_cmd"', bridge)
        self.assertIn('"gate_release"', bridge)
        self.assertIn('"mode": "continuous_asymmetric"', bridge)
        self.assertIn('"heading_yaw_priority"', bridge)
        self.assertIn("priority_yaw=heading_yaw_priority", bridge)
        self.assertIn('"reverse_hold_active"', bridge)
        self.assertIn('"transport_sent"', bridge)
        self.assertIn(
            "# Diagnostics must never interrupt the velocity command path.",
            bridge,
        )

    def test_start_gate_and_dashboard_status_states(self):
        root = PACKAGE.parents[1]
        medical_task = (root / "Program" / "Core" / "Src" / "MedicalTask.c").read_text(
            encoding="utf-8"
        )
        medical_header = (root / "Program" / "Core" / "Inc" / "MedicalTask.h").read_text(
            encoding="utf-8"
        )
        dashboard = (PACKAGE / "obstacle_detector" / "scan_dashboard.py").read_text(
            encoding="utf-8"
        )
        dashboard_core = (
            PACKAGE / "obstacle_detector" / "scan_dashboard_core.py"
        ).read_text(encoding="utf-8")
        bridge = (PACKAGE / "obstacle_detector" / "stm32_bridge.py").read_text(
            encoding="utf-8"
        )
        navigator = (PACKAGE / "obstacle_detector" / "medical_navigator.py").read_text(
            encoding="utf-8"
        )

        self.assertIn("MEDICAL_TASK_WAIT_START = 14", medical_header)
        self.assertIn("MEDICAL_TASK_WAIT_BED1_START = 15", medical_header)
        self.assertIn("MEDICAL_TASK_WAIT_BED3_START = 16", medical_header)
        self.assertIn("medical_set_state(MEDICAL_TASK_WAIT_START);", medical_task)
        self.assertIn("medical_start_button_event", medical_task)
        self.assertIn("BTN_C_GPIO_Port", medical_task)
        self.assertIn("TASK_WAIT_START", bridge)
        self.assertIn('"start_gate"', bridge)
        self.assertIn('status.get("plan_ready", False)', bridge)
        self.assertIn('"plan_ready": self.plan_ready', navigator)
        self.assertIn("len(msg.poses) > 0", navigator)
        self.assertIn("_hide_order_value", dashboard)
        self.assertIn("describe_start_status", dashboard)
        self.assertIn('self.start_value = QLabel("等待")', dashboard)
        self.assertIn('camera_layout.addWidget(self.start_value)', dashboard)
        self.assertNotIn('start_card, self.start_value', dashboard)
        # The status word and every colour live in the testable core mapping.
        self.assertIn("START_STATUS_TEXT", dashboard_core)
        self.assertIn("TASK_ACTIVE_STATES", dashboard_core)
        for expected_text in ("等待", "发车", "运行", "成功"):
            self.assertIn(expected_text, dashboard_core)
        self.assertIn('START_STATUS_COLOR[state]', dashboard_core)
        for obsolete_text in (
            "等待状态",
            "等待连接",
            "请按 A",
            "等待路径规划",
            "行驶中",
            "已到达",
            "已停车",
        ):
            self.assertNotIn(obsolete_text, dashboard)

    def test_nurse_qr_waits_for_smoothed_stop_before_stm32_transition(self):
        navigator = (
            PACKAGE / "obstacle_detector" / "medical_navigator.py"
        ).read_text(encoding="utf-8")
        bridge = (PACKAGE / "obstacle_detector" / "stm32_bridge.py").read_text(
            encoding="utf-8"
        )
        launch_source = (PACKAGE / "launch" / "obstacle.launch.py").read_text(
            encoding="utf-8"
        )

        self.assertIn('source="nurse_scan_soft_stop"', bridge)
        self.assertIn('"defer_until_stopped"', bridge)
        self.assertIn("SettledStopDetector", bridge)
        self.assertIn("self.nav_status = NAV_FOLLOWING", navigator)
        self.assertIn('"nurse_scan_stop_linear_m_s": 0.03', launch_source)
        self.assertIn('"nurse_scan_stop_settle_s": 0.15', launch_source)

    def test_bed_docking_requires_slow_stable_corridor_handoff(self):
        root = PACKAGE.parents[1]
        medical_task = (root / "Program" / "Core" / "Src" / "MedicalTask.c").read_text(
            encoding="utf-8"
        )
        self.assertIn("MEDICAL_DOCK_SAMPLE_COUNT 8U", medical_task)
        self.assertIn("MEDICAL_DOCK_SAMPLE_TRIM_COUNT 1U", medical_task)
        self.assertIn("MEDICAL_DOCK_SAMPLE_SETTLE_MS 100U", medical_task)
        self.assertIn("MEDICAL_DOCK_SAMPLE_TIMEOUT_MS 1500U", medical_task)
        self.assertIn("MEDICAL_DOCK_TRACK_MIN_SAMPLES 5U", medical_task)
        self.assertIn("MEDICAL_DOCK_TRACK_SPREAD_MM 40.0f", medical_task)
        self.assertIn("MEDICAL_DOCK_TRACK_RADIUS_MM 350.0f", medical_task)
        self.assertIn("MEDICAL_DOCK_TRACK_YAW_DEG 3.0f", medical_task)
        self.assertIn(
            "MEDICAL_DOCK_HANDOFF_MAX_SPEED_MM_S 450.0f", medical_task
        )
        self.assertIn("MEDICAL_DOCK_HANDOFF_STABLE_MS 150U", medical_task)
        self.assertIn(
            "MEDICAL_DOCK_SAFE_FORWARD_HALF_WIDTH_MM 300.0f", medical_task
        )
        self.assertIn(
            "MEDICAL_DOCK_SAFE_APPROACH_DEPTH_MM 350.0f", medical_task
        )
        self.assertIn("MEDICAL_DOCK_SAFE_OVERSHOOT_MM 40.0f", medical_task)
        self.assertIn("MEDICAL_DOCK_TIMEOUT_MS 3000U", medical_task)
        self.assertIn("MEDICAL_DOCK_SIDE_SPLIT_THRESHOLD_MM 1600U", medical_task)
        self.assertIn("MEDICAL_DOCK_LASER_MAX_CORRECTION_MM 400.0f", medical_task)
        self.assertIn("MEDICAL_DOCK_SPEED_LOC_K 1000.0f", medical_task)
        self.assertIn("MEDICAL_DOCK_TIMEOUT_BEEP_FIRST_MS 200U", medical_task)
        self.assertIn("MEDICAL_DOCK_TIMEOUT_BEEP_GAP_MS 100U", medical_task)
        self.assertIn("MEDICAL_DOCK_TIMEOUT_BEEP_SECOND_MS 300U", medical_task)
        self.assertRegex(
            medical_task,
            r"if \(s_current_bed == 1U\)\s*"
            r"\{\s*return STP23L_GetSampleC\(distance_mm, frame_sequence\);\s*\}\s*"
            r"return STP23L_GetSampleA\(distance_mm, frame_sequence\);",
        )
        self.assertIn("medical_is_bed_navigation_state() != 0U", medical_task)
        self.assertIn("medical_docking_in_safe_corridor", medical_task)
        self.assertIn("medical_docking_handoff_target", medical_task)
        self.assertIn("medical_docking_capture_targets", medical_task)
        self.assertIn("medical_docking_get_tracked_target", medical_task)
        self.assertIn("medical_docking_finish_or_correct_tracked", medical_task)
        self.assertRegex(
            medical_task,
            r"medical_is_bed_navigation_state\(\) != 0U[\s\S]*?"
            r"medical_docking_handoff_target[\s\S]*?"
            r"medical_set_state[\s\S]*?"
            r"medical_docking_start_absolute_move[\s\S]*?return 1U;",
        )
        self.assertIn("medical_docking_collect_samples", medical_task)
        self.assertIn("medical_docking_collect_front_samples", medical_task)
        self.assertIn("MEDICAL_DOCK_SAMPLE_INITIAL", medical_task)
        self.assertIn("MEDICAL_DOCK_MOVE_TRACKED", medical_task)
        self.assertIn("MEDICAL_DOCK_MOVE_TRACKED_CORRECTION", medical_task)
        self.assertIn("MEDICAL_DOCK_MOVE_COMBINED", medical_task)
        self.assertIn("MEDICAL_DOCK_MOVE_SPLIT_SIDE", medical_task)
        self.assertIn("MEDICAL_DOCK_SAMPLE_SPLIT_FRONT", medical_task)
        self.assertIn(
            "DJI_Chassis_SetSpeedLocationGain(MEDICAL_DOCK_SPEED_LOC_K)",
            medical_task,
        )
        self.assertIn("DJI_Chassis_SetSpeedLocationGain(0.0f)", medical_task)
        self.assertIn("medical_docking_timeout_beep_start", medical_task)
        self.assertRegex(
            medical_task,
            r"ChassisCtrl_MoveTarget\(pos_x \+ field_x_delta,\s*"
            r"pos_y \+ field_y_delta,\s*0\.0f,",
        )

    def test_arm_overlaps_final_docking_and_retracts_during_navigation(self):
        root = PACKAGE.parents[1]
        medical_task = (root / "Program" / "Core" / "Src" / "MedicalTask.c").read_text(
            encoding="utf-8"
        )
        motor_control = (
            root / "Program" / "Core" / "Src" / "DJIMotorCtrlSTM32.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn("MEDICAL_ARM_DEPLOY_TIME_S 1.5f", medical_task)
        self.assertIn("MEDICAL_ARM_RETRACT_TIME_S 1.5f", medical_task)
        self.assertIn("MEDICAL_DOCK_SAMPLE_INITIAL", medical_task)
        self.assertIn("MEDICAL_DOCK_MOVE_COMBINED", medical_task)
        self.assertRegex(
            medical_task,
            r"case MEDICAL_TASK_NAV_BED1:[\s\S]*?"
            r"medical_arm_set_deployed\(0U\);[\s\S]*?"
            r"NUC_Nav_RequestGoal\(NUC_NAV_GOAL_BED1\);",
        )
        self.assertRegex(
            medical_task,
            r"case MEDICAL_TASK_NAV_HOME:[\s\S]*?"
            r"medical_arm_set_deployed\(0U\);[\s\S]*?"
            r"NUC_Nav_RequestGoal\(NUC_NAV_GOAL_HOME\);",
        )
        self.assertIn("pos_default.max_out = 6000.f;", motor_control)

    def test_stm32_uses_normalized_xdrive_and_matching_odometry(self):
        root = PACKAGE.parents[1]
        motor_control = (
            root / "Program" / "Core" / "Src" / "DJIMotorCtrlSTM32.cpp"
        ).read_text(encoding="utf-8")
        nav_transport = (
            root / "Program" / "Core" / "Src" / "NUC_Obstacle.c"
        ).read_text(encoding="utf-8")

        self.assertIn("kInvSqrt2 = 0.70710678118654752440f", motor_control)
        self.assertIn("kSqrt2 = 1.41421356237309504880f", motor_control)
        self.assertIn("(Vx+Vy) * kInvSqrt2 + W", motor_control)
        self.assertIn("0.25f * kSqrt2", motor_control)
        self.assertIn("kMaxPlanarSpeedMmS = 3000.0f", motor_control)
        self.assertIn("kMaxWheelSurfaceSpeedMmS = 3000.0f", motor_control)
        self.assertIn("speed_default.kp = 6.5f;", motor_control)
        self.assertIn("speed_default.ki = 1.f;", motor_control)
        self.assertIn("speed_default.kd = 0.01f;", motor_control)
        self.assertIn("speed_default.max_out = 10000.f;", motor_control)
        self.assertIn("#define NAV_STM32_MAX_LINEAR_MM_S 3000", nav_transport)

    def test_bed_workflow_uses_bounded_fast_docking(self):
        root = PACKAGE.parents[1]
        medical_task = (root / "Program" / "Core" / "Src" / "MedicalTask.c").read_text(
            encoding="utf-8"
        )
        chassis_control = (
            root / "Program" / "Core" / "Src" / "ChassisCtrl.c"
        ).read_text(encoding="utf-8")
        pid_control = (
            root / "Program" / "Core" / "Src" / "PID.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn("MEDICAL_DISPENSE_HOLD_MS 2500U", medical_task)
        self.assertIn("MEDICAL_DOCK_SAMPLE_COUNT 8U", medical_task)
        self.assertIn("MEDICAL_DOCK_SAMPLE_TRIM_COUNT 1U", medical_task)
        self.assertIn("MEDICAL_DOCK_SAMPLE_TIMEOUT_MS 1500U", medical_task)
        self.assertIn("CHASSIS_CTRL_REACH_X_MM 5.0f", chassis_control)
        self.assertIn("CHASSIS_CTRL_REACH_Y_MM 5.0f", chassis_control)
        self.assertIn("CHASSIS_CTRL_REACH_YAW_DEG 0.5f", chassis_control)
        self.assertIn("CHASSIS_CTRL_XY_SLEW_RPM_PER_S 300.0f", chassis_control)
        self.assertIn("CHASSIS_CTRL_YAW_SLEW_RPM_PER_S 600.0f", chassis_control)
        self.assertNotIn("CHASSIS_CTRL_MIN_XY_RPM", chassis_control)
        self.assertNotIn("CHASSIS_CTRL_MIN_YAW_RPM", chassis_control)
        self.assertIn("has_previous_", pid_control)
        self.assertIn("(has_previous_ != 0U)", pid_control)

    def test_scanner_uses_full_frame_and_bed_proximity_gate(self):
        scanner = yaml.safe_load(SCANNER_PARAMS.read_text(encoding="utf-8"))[
            "code_scanner"
        ]["ros__parameters"]
        self.assertEqual(scanner["roi_left"], 0.0)
        self.assertEqual(scanner["roi_right"], 1.0)
        self.assertEqual(scanner["roi_top"], 0.0)
        self.assertEqual(scanner["roi_bottom"], 1.0)
        self.assertEqual(scanner["bed_scan_activation_distance_m"], 1.5)
        self.assertEqual(scanner["pose_timeout_s"], 0.5)
        self.assertEqual(scanner["decode_width"], 1280)
        self.assertEqual(scanner["decode_height"], 800)
        self.assertEqual(scanner["full_resolution_fallback_period"], 4)
        self.assertEqual(
            scanner["tele_camera_device"],
            "/dev/v4l/by-id/usb-BLC-240823--A_SDYH-8P0P-video-index0",
        )
        self.assertEqual(scanner["tele_width"], 1920)
        self.assertEqual(scanner["tele_height"], 1080)
        self.assertEqual(scanner["tele_fps"], 5.0)
        self.assertEqual(scanner["tele_scan_timeout_s"], 15.0)
        self.assertEqual(
            scanner["tele_preview_topic"],
            "/medical_nav/tele_scanner_preview/compressed",
        )

        launch_source = (PACKAGE / "launch" / "obstacle.launch.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('"bed_scan_activation_distance_m": 1.5', launch_source)
        self.assertIn('"field_config": field_map', launch_source)
        self.assertIn('"tele_camera_device": tele_scan_camera', launch_source)
        self.assertIn('"tele_scan_camera"', launch_source)

        scanner_source = (PACKAGE / "obstacle_detector" / "code_scanner.py").read_text(
            encoding="utf-8"
        )
        dashboard_source = (
            PACKAGE / "obstacle_detector" / "scan_dashboard.py"
        ).read_text(encoding="utf-8")
        self.assertIn("tele_camera_assists(self.task_state, format)", scanner_source)
        self.assertIn('source = f"tele_{source}"', scanner_source)
        self.assertIn('(("YUYV", True), ("driver-default", False))', scanner_source)
        self.assertIn('else (("MJPG", True), ("driver-default", False))', scanner_source)
        self.assertIn(
            "tele_camera_should_capture(self.task_state)",
            scanner_source,
        )
        self.assertIn("camera for USB handoff", scanner_source)
        self.assertIn("cv2.VideoWriter_fourcc(*profile_name)", scanner_source)
        self.assertIn('"tele_scan_state": self._tele_scan_state()', scanner_source)
        self.assertIn('and source.startswith("tele_")', scanner_source)
        self.assertIn('"mission_epoch": self.mission_epoch', scanner_source)
        self.assertIn('self.tele_camera_view.setText("失败")', dashboard_source)
        self.assertIn("长焦辅助摄像头", dashboard_source)
        self.assertIn(
            '"/medical_nav/tele_scanner_preview/compressed"', dashboard_source
        )


if __name__ == "__main__":
    unittest.main()
