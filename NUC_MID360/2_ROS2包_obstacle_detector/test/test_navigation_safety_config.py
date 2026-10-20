"""Static checks for the live-replanning and collision-monitor profile."""

import ast
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
PARAMS = PACKAGE / "config" / "nav2_params.yaml"
FIELD_MAP = PACKAGE / "config" / "field_map.yaml"
SCANNER_PARAMS = PACKAGE / "config" / "scanner.yaml"
BT_XML = PACKAGE / "config" / "navigate_to_pose_1hz.xml"
NURSE_BT_XML = PACKAGE / "config" / "navigate_to_pose_nurse_fast.xml"


class NavigationSafetyConfigTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = yaml.safe_load(PARAMS.read_text(encoding="utf-8"))

    def test_single_dwb_profile_is_used_for_every_route(self):
        params = self.config["controller_server"]["ros__parameters"]
        self.assertEqual(params["controller_frequency"], 20.0)
        self.assertEqual(params["controller_plugins"], ["FollowPathDWB"])
        follow_path = params["FollowPathDWB"]
        self.assertEqual(follow_path["plugin"], "dwb_core::DWBLocalPlanner")
        self.assertEqual((follow_path["min_vel_x"], follow_path["max_vel_x"]), (-2.0, 2.0))
        self.assertEqual((follow_path["min_vel_y"], follow_path["max_vel_y"]), (-2.0, 2.0))
        self.assertEqual(follow_path["acc_lim_x"], 2.0)
        self.assertEqual(follow_path["acc_lim_y"], 2.0)
        self.assertEqual(follow_path["decel_lim_x"], -2.0)
        self.assertEqual(follow_path["decel_lim_y"], -2.0)
        self.assertEqual(follow_path["sim_time"], 1.0)
        self.assertEqual(follow_path["vx_samples"], 21)
        self.assertEqual(follow_path["vy_samples"], 21)
        self.assertIn("BaseObstacle", follow_path["critics"])
        self.assertNotIn("CostCritic", follow_path["critics"])
        self.assertNotIn("FollowPathCornerDWB", params)

        for tree_path in (BT_XML, NURSE_BT_XML):
            tree_source = tree_path.read_text(encoding="utf-8")
            self.assertNotIn("PlannerSelector", tree_source)
            self.assertNotIn("selected_planner", tree_source)
            self.assertNotIn("ControllerSelector", tree_source)
            self.assertNotIn("selected_controller", tree_source)
            self.assertIn('planner_id="GridBased"', tree_source)
            self.assertIn('controller_id="FollowPathDWB"', tree_source)
        plugin_lib_names = self.config["bt_navigator"]["ros__parameters"][
            "plugin_lib_names"
        ]
        self.assertNotIn("nav2_planner_selector_bt_node", plugin_lib_names)
        self.assertNotIn("nav2_controller_selector_bt_node", plugin_lib_names)
        self.assertNotIn("nav2_goal_checker_selector_bt_node", plugin_lib_names)

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
        follow_path = params["FollowPathDWB"]
        self.assertEqual(follow_path["max_speed_xy"], 2.0)
        self.assertEqual(follow_path["max_vel_theta"], 0.0)
        self.assertEqual(follow_path["min_speed_theta"], 0.0)
        self.assertEqual(follow_path["vtheta_samples"], 1)
        self.assertEqual(follow_path["BaseObstacle.scale"], 0.05)
        self.assertEqual(follow_path["PathDist.scale"], 16.0)
        self.assertEqual(follow_path["GoalDist.scale"], 10.0)
        self.assertNotIn("PathAlign", follow_path["critics"])
        self.assertNotIn("GoalAlign", follow_path["critics"])
        self.assertNotIn("Twirling", follow_path["critics"])
        smoother = self.config["velocity_smoother"]["ros__parameters"]
        self.assertEqual(smoother["max_velocity"][2], 0.0)
        self.assertEqual(smoother["min_velocity"][2], 0.0)

    def test_parking_exemption_is_identical_in_both_costmaps(self):
        local = self.config["local_costmap"]["local_costmap"]["ros__parameters"]
        global_costmap = self.config["global_costmap"]["global_costmap"][
            "ros__parameters"
        ]
        expected_plugins = [
            "static_layer",
            "obstacle_layer",
            "inflation_layer",
            "parking_exemption_layer",
        ]
        self.assertEqual(local["plugins"], expected_plugins)
        self.assertEqual(global_costmap["plugins"], expected_plugins)
        self.assertEqual(
            local["parking_exemption_layer"],
            global_costmap["parking_exemption_layer"],
        )
        exemption = local["parking_exemption_layer"]
        self.assertEqual(
            exemption["plugin"],
            "medical_clearance_planner::ParkingExemptionLayer",
        )
        self.assertTrue(exemption["enabled"])
        self.assertEqual(exemption["zone_frame"], "map")
        self.assertEqual((exemption["bed1_x"], exemption["bed1_y"]), (5.4, 2.25))
        self.assertEqual((exemption["bed3_x"], exemption["bed3_y"]), (5.4, -2.2))
        self.assertEqual(exemption["bed_radius"], 0.30)
        self.assertEqual((exemption["home_x"], exemption["home_y"]), (0.0, 0.0))
        self.assertEqual(
            (exemption["home_size_x"], exemption["home_size_y"]),
            (1.0, 1.0),
        )
        field = yaml.safe_load(FIELD_MAP.read_text(encoding="utf-8"))
        for goal_name, prefix in (
            ("bed1", "bed1"),
            ("bed3", "bed3"),
            ("home", "home"),
        ):
            goal = field["goals"][goal_name]
            self.assertAlmostEqual(
                exemption[f"{prefix}_x"], goal["y_mm"] / 1000.0
            )
            self.assertAlmostEqual(
                exemption[f"{prefix}_y"], -goal["x_mm"] / 1000.0
            )

    def test_collision_monitor_restores_field_tested_single_stage_profile(self):
        normal_smoother = self.config["velocity_smoother"]["ros__parameters"]
        monitor = self.config["collision_monitor"]["ros__parameters"]
        local_costmap = self.config["local_costmap"]["local_costmap"][
            "ros__parameters"
        ]
        launch_source = (PACKAGE / "launch" / "obstacle.launch.py").read_text(
            encoding="utf-8"
        )

        # Ordinary motion is ramp-limited before entering the safety chain.
        self.assertEqual(normal_smoother["feedback"], "OPEN_LOOP")
        self.assertEqual(normal_smoother["smoothing_frequency"], 20.0)
        self.assertEqual(normal_smoother["max_accel"], [2.5, 2.5, 2.2])
        self.assertEqual(normal_smoother["max_decel"], [-2.0, -2.0, -2.5])
        self.assertEqual(normal_smoother["max_velocity"], [2.0, 2.0, 0.0])
        self.assertEqual(normal_smoother["min_velocity"], [-2.0, -2.0, 0.0])
        self.assertFalse(normal_smoother["scale_velocities"])
        self.assertEqual((local_costmap["width"], local_costmap["height"]), (14, 14))
        local_livox = local_costmap["obstacle_layer"]["livox"]
        self.assertEqual(local_livox["obstacle_max_range"], 5.5)
        self.assertEqual(local_livox["raytrace_max_range"], 6.0)
        self.assertNotIn("collision_monitor_predictive", self.config)
        self.assertEqual(monitor["polygons"], ["ApproachPolygon", "StopPolygon"])
        self.assertEqual(monitor["cmd_vel_in_topic"], "/cmd_vel_home_limited")
        self.assertEqual(
            monitor["cmd_vel_out_topic"], "/cmd_vel_collision_safe"
        )
        self.assertIn(
            '"collision_input_topic": "/cmd_vel_home_limited"',
            (PACKAGE / "launch" / "obstacle.launch.py").read_text(
                encoding="utf-8"
            ),
        )
        self.assertEqual(monitor["source_timeout"], 0.3)
        self.assertNotIn("safety_velocity_smoother", self.config)
        self.assertIn('executable="omni_escape"', launch_source)
        self.assertIn('"input_topic": "/cmd_vel_collision_safe"', launch_source)
        self.assertIn('"output_topic": "/cmd_vel_safe"', launch_source)
        self.assertIn('"stall_duration_s": 4.0', launch_source)
        self.assertIn('"blocked_intent_hold_s": 6.0', launch_source)
        self.assertIn('"maximum_stall_displacement_m": 0.08', launch_source)
        self.assertIn('"escape_speed_m_s": 0.20', launch_source)
        self.assertIn('"travel_distance_m": 0.30', launch_source)
        self.assertIn('"path_timeout_s": 6.0', launch_source)

        approach = monitor["ApproachPolygon"]
        self.assertEqual(approach["action_type"], "approach")
        self.assertEqual(approach["time_before_collision"], 0.8)
        self.assertEqual(approach["simulation_time_step"], 0.1)
        self.assertEqual(approach["max_points"], 5)
        self.assertAlmostEqual(
            max(abs(value) for value in approach["points"]), 0.28
        )

        stop = monitor["StopPolygon"]
        self.assertEqual(stop["action_type"], "stop")
        self.assertEqual(stop["max_points"], 3)
        self.assertEqual(len(approach["points"]) % 2, 0)
        self.assertEqual(len(stop["points"]) % 2, 0)
        self.assertEqual(len(stop["points"]), 16)
        self.assertAlmostEqual(max(abs(value) for value in stop["points"]), 0.265)

        bridge_source = (
            PACKAGE / "obstacle_detector" / "stm32_bridge.py"
        ).read_text(encoding="utf-8")
        chassis_control = (
            PACKAGE.parents[1] / "Program" / "Core" / "Src" / "ChassisCtrl.c"
        ).read_text(encoding="utf-8")
        stm32_main = (
            PACKAGE.parents[1] / "Program" / "Core" / "Src" / "main.c"
        ).read_text(encoding="utf-8")
        hwt_source = (
            PACKAGE.parents[1] / "Program" / "Core" / "Src" / "HWT101CT.c"
        ).read_text(encoding="utf-8")
        self.assertEqual(launch_source.count('"self_radius": 0.265'), 2)
        self.assertEqual(
            launch_source.count('"ground_removal_min_radius": 0.27'), 2
        )
        self.assertIn('"self_filter_shape": "polygon"', launch_source)
        self.assertIn('"self_polygon_sides": 16', launch_source)
        self.assertIn('"calibration_stop_radius": 0.285', launch_source)
        self.assertIn(
            '"self_polygon_topic": "/medical_nav/safety_self_filter_polygon"',
            launch_source,
        )
        self.assertEqual(launch_source.count('package="nav2_collision_monitor"'), 1)
        self.assertNotIn("nav2_velocity_smoother", launch_source)
        self.assertNotIn('name="collision_monitor_predictive"', launch_source)
        self.assertIn('executable="braking_safety_shadow"', launch_source)
        self.assertIn('name="braking_safety_shadow"', launch_source)
        self.assertIn(
            '"output_topic": "/medical_nav/braking_shadow_cmd_vel"',
            launch_source,
        )
        self.assertIn(
            '"status_topic": "/medical_nav/braking_shadow_status"',
            launch_source,
        )
        self.assertIn('"model_braking_decel_m_s2": 2.5', launch_source)
        self.assertIn('"reaction_time_s": 0.20', launch_source)
        self.assertIn('"safety_margin_m": 0.15', launch_source)
        self.assertIn('"footprint_radius_m": 0.245', launch_source)
        self.assertIn('"max_lookahead_m": 4.0', launch_source)
        self.assertIn('"cloud_timeout_s": 0.20', launch_source)
        self.assertIn('"odom_timeout_s": 0.15', launch_source)
        self.assertIn('"update_rate_hz": 50.0', launch_source)
        self.assertIn('executable="home_approach_limiter"', launch_source)
        self.assertIn('name="home_approach_limiter"', launch_source)
        self.assertIn('executable="corner_speed_limiter"', launch_source)
        self.assertIn('name="corner_speed_limiter"', launch_source)
        corner_nodes = [
            node for node in ast.walk(ast.parse(launch_source))
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
            and node.func.id == "Node"
            and any(keyword.arg == "executable"
                    and isinstance(keyword.value, ast.Constant)
                    and keyword.value.value == "corner_speed_limiter"
                    for keyword in node.keywords)
        ]
        self.assertEqual(len(corner_nodes), 1)
        corner_params = next(keyword.value for keyword in corner_nodes[0].keywords
                             if keyword.arg == "parameters")
        self.assertIs(ast.literal_eval(corner_params)[0]["enabled"], True)
        self.assertIn('"raw_path_topic": "/plan"', launch_source)
        self.assertIn(
            '"smoothed_path_topic": "/transformed_global_plan"',
            launch_source,
        )
        self.assertIn('"output_topic": "/speed_limit"', launch_source)
        self.assertNotIn("controller_selector", launch_source)
        self.assertIn('"profile_shadow_enabled": True', launch_source)
        self.assertIn(
            '"reference_profile_topic": "/medical_nav/reference_speed_profile"',
            launch_source,
        )
        self.assertIn(
            '"reference_status_topic": "/medical_nav/reference_speed_status"',
            launch_source,
        )
        self.assertIn('"profile_lateral_accel_m_s2": 2.30', launch_source)
        self.assertIn('"profile_braking_decel_m_s2": 2.0', launch_source)
        self.assertIn('"profile_reaction_time_s": 0.20', launch_source)
        self.assertIn('"profile_lookahead_distance_m": 6.0', launch_source)
        self.assertIn('"profile_curvature_span_m": 0.50', launch_source)
        self.assertIn('"profile_publish_rate_hz": 5.0', launch_source)
        self.assertNotIn("FollowPathCornerDWB", launch_source)
        self.assertIn('"lookahead_distance_m": 3.50', launch_source)
        self.assertIn('"braking_decel_m_s2": 2.00', launch_source)
        self.assertIn('"reaction_time_s": 0.20', launch_source)
        self.assertIn('"max_corner_speed_m_s": 1.00', launch_source)
        self.assertIn('"release_hold_s": 0.50', launch_source)
        self.assertIn('"straight_confirmation_s": 0.20', launch_source)
        self.assertIn('"update_rate_hz": 20.0', launch_source)
        self.assertIn('"min_corner_speed_m_s": 0.50', launch_source)
        self.assertIn('"lateral_accel_m_s2": 2.30', launch_source)
        self.assertIn('"tangent_span_m": 0.75', launch_source)
        self.assertIn('"min_turn_angle_deg": 35.0', launch_source)
        self.assertIn('"braking_margin_m": 0.20', launch_source)
        self.assertIn('"input_topic": "/cmd_vel"', launch_source)
        self.assertIn('"output_topic": "/cmd_vel_home_limited"', launch_source)
        self.assertIn('"home_task_state": 9', launch_source)
        self.assertIn('"bed1_task_state": 3', launch_source)
        self.assertIn('"bed3_task_state": 6', launch_source)
        self.assertIn('"max_speed_m_s": 3.0', launch_source)
        self.assertNotIn('"soft_decel_m_s2"', launch_source)
        self.assertNotIn('"decel_start_distance_m"', launch_source)
        self.assertIn('"field_braking_decel_m_s2": 2.0', launch_source)
        self.assertIn('"home_braking_decel_m_s2": 2.0', launch_source)
        self.assertIn('"home_reaction_time_s": 0.22', launch_source)
        self.assertIn('"bed_max_speed_m_s": 3.0', launch_source)
        self.assertIn('"wheel_odom_topic": "/medical_nav/wheel_odom"', launch_source)
        self.assertIn('"status_topic": "/medical_nav/bed_approach_status"', launch_source)
        self.assertIn('"bed_forward_decel_m_s2": 2.0', launch_source)
        self.assertIn('"bed_side_decel_m_s2": 2.2', launch_source)
        self.assertIn('"bed_reaction_time_s": 0.22', launch_source)
        self.assertIn('"bed_braking_margin_m": 0.03', launch_source)
        self.assertIn('"pose_timeout_s": 0.5', launch_source)
        self.assertIn('"gate_release_wheel_accel_m_s2": 0.0', launch_source)
        self.assertIn('"gate_release_yaw_radius_m": 0.25', launch_source)
        self.assertIn('"max_axis_speed_m_s": 3.0', launch_source)
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
        self.assertIn('"heading_hold_enabled": False', launch_source)
        self.assertIn(
            '"heading_hold_task_states": [1, 3, 6, 9]', launch_source
        )
        self.assertIn('"heading_hold_target_deg": 0.0', launch_source)
        self.assertIn('"heading_hold_kp": 2.2', launch_source)
        self.assertIn('"heading_hold_ki": 0.0', launch_source)
        self.assertIn('"heading_hold_kd": 0.45', launch_source)
        self.assertIn('"heading_hold_deadband_deg": 1.0', launch_source)
        self.assertIn(
            '"heading_hold_max_wz_rad_s": 0.40', launch_source
        )
        self.assertNotIn("heading_hold_reverse_", launch_source)
        self.assertIn(
            '"heading_hold_yaw_rate_deadband_rad_s": 0.02', launch_source
        )
        self.assertNotIn("stm32_yaw_hold_requested", bridge_source)
        self.assertIn(
            "nav_wz = 0.0 if nurse_scan_translation_only else float(wz)",
            bridge_source,
        )
        self.assertIn("NAV_YAW_HOLD_KP 2.2f", chassis_control)
        self.assertIn("NAV_YAW_HOLD_KD 0.45f", chassis_control)
        self.assertIn("NAV_YAW_HOLD_MAX_RAD_S 0.40f", chassis_control)
        self.assertNotIn("NAV_YAW_HOLD_REVERSE", chassis_control)
        self.assertIn("task_state == MEDICAL_TASK_NAV_NURSE", stm32_main)
        self.assertIn("task_state == MEDICAL_TASK_NAV_BED1", stm32_main)
        self.assertIn("task_state == MEDICAL_TASK_NAV_BED3", stm32_main)
        self.assertIn("task_state == MEDICAL_TASK_NAV_HOME", stm32_main)
        self.assertIn("ChassisCtrl_NavYawUpdate", stm32_main)
        self.assertIn(
            "forward_mm_s != 0 || left_mm_s != 0 || yaw_ccw_cdeg_s != 0",
            stm32_main,
        )
        self.assertNotIn("ChassisCtrl_NavYawReset", stm32_main)
        self.assertIn('"yaw_sign": -1.0', launch_source)
        self.assertIn("HWT101_GetYawStamped(", stm32_main)
        self.assertIn("float yaw_deg = -hwt_yaw;", stm32_main)
        self.assertIn("HWT101_GetYaw(),", stm32_main)
        self.assertIn("HWT101_GetWz());", stm32_main)
        self.assertIn("static volatile uint32_t s_last_wz_ms;", hwt_source)
        self.assertIn(
            "(HAL_GetTick() - s_last_wz_ms) >= HWT_YAW_MAX_AGE_MS",
            hwt_source,
        )
        self.assertIn(
            '"gate_release_rearm_drop_m_s": 0.25', launch_source
        )
        self.assertNotIn('"collision_monitor_predictive",', launch_source)

        setup_source = (PACKAGE / "setup.py").read_text(encoding="utf-8")
        approach_source = (
            PACKAGE / "obstacle_detector" / "home_approach_limiter.py"
        ).read_text(encoding="utf-8")
        self.assertIn("clamp_axis_velocity", approach_source)
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
        self.assertIn(
            "braking_safety_shadow = obstacle_detector.braking_safety_shadow:main",
            setup_source,
        )
        braking_shadow_source = (
            PACKAGE / "obstacle_detector" / "braking_safety_shadow.py"
        ).read_text(encoding="utf-8")
        self.assertIn('"shadow_mode": True', braking_shadow_source)
        self.assertIn('"affects_control": False', braking_shadow_source)
        self.assertNotIn('create_publisher(\n            Twist, "/cmd_vel_safe"', braking_shadow_source)

    def test_behavior_trees_use_bounded_replanning_rates(self):
        root = ET.parse(BT_XML).getroot()
        rate_controllers = root.findall(".//RateController")
        self.assertEqual(len(rate_controllers), 1)
        self.assertEqual(float(rate_controllers[0].attrib["hz"]), 1.0)
        smoothers = root.findall(".//SmoothPath")
        self.assertEqual(len(smoothers), 1)
        self.assertEqual(
            smoothers[0].attrib["smoother_id"], "constrained_smoother"
        )
        self.assertEqual(
            float(smoothers[0].attrib["max_smoothing_duration"]), 0.45
        )
        smooth_fallback = root.find(".//Fallback[@name='SmoothOrKeepOriginal']")
        self.assertIsNotNone(smooth_fallback)
        self.assertIsNotNone(smooth_fallback.find("SmoothPath"))
        self.assertIsNotNone(smooth_fallback.find("AlwaysSuccess"))
        plugin_names = self.config["bt_navigator"]["ros__parameters"][
            "plugin_lib_names"
        ]
        self.assertIn("nav2_smooth_path_action_bt_node", plugin_names)
        smoother_params = self.config["smoother_server"]["ros__parameters"]
        self.assertEqual(
            smoother_params["smoother_plugins"], ["constrained_smoother"]
        )
        smoother = smoother_params["constrained_smoother"]
        self.assertEqual(
            smoother["plugin"],
            "nav2_constrained_smoother/ConstrainedSmoother",
        )
        self.assertFalse(smoother["reversing_enabled"])
        self.assertEqual(smoother["path_downsampling_factor"], 2)
        self.assertEqual(smoother["path_upsampling_factor"], 1)
        self.assertTrue(smoother["keep_start_orientation"])
        self.assertTrue(smoother["keep_goal_orientation"])
        self.assertEqual(smoother["minimum_turning_radius"], 0.75)
        self.assertEqual(smoother["w_curve"], 30.0)
        self.assertEqual(smoother["w_dist"], 0.0)
        self.assertEqual(smoother["w_smooth"], 2000000.0)
        self.assertEqual(smoother["w_cost"], 0.03)
        self.assertEqual(smoother["optimizer"]["max_iterations"], 70)
        package_root = ET.parse(PACKAGE / "package.xml").getroot()
        dependencies = {
            element.text
            for element in package_root
            if element.tag in {"depend", "exec_depend"}
        }
        self.assertIn("nav2_constrained_smoother", dependencies)
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
        # A collision-checked smoother must sit after the current planner,
        # and write the exact blackboard path consumed by FollowPath. On
        # failure AlwaysSuccess keeps this cycle's newly computed raw path.
        for tree in (root, nurse_root):
            with self.subTest(tree=tree.find("BehaviorTree").attrib["ID"]):
                plan_sequence = tree.find(".//SmoothPath/../..")
                self.assertEqual(plan_sequence.tag, "Sequence")
                compute, fallback = list(plan_sequence)
                self.assertEqual(compute.tag, "ComputePathToPose")
                self.assertEqual(fallback.tag, "Fallback")
                smooth, keep_raw = list(fallback)
                self.assertEqual(smooth.tag, "SmoothPath")
                self.assertEqual(keep_raw.tag, "AlwaysSuccess")
                self.assertEqual(smooth.attrib["check_for_collisions"], "true")
                path_key = compute.attrib["path"]
                self.assertEqual(smooth.attrib["unsmoothed_path"], path_key)
                self.assertEqual(smooth.attrib["smoothed_path"], path_key)
                self.assertEqual(tree.find(".//FollowPath").attrib["path"], path_key)
        for tree in (root, nurse_root):
            self.assertIsNone(tree.find(".//ControllerSelector"))
            self.assertEqual(
                tree.find(".//FollowPath").attrib["controller_id"],
                "FollowPathDWB",
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
        self.assertNotIn('"/controller_selector"', navigator)
        self.assertNotIn("controller_for_route(", navigator)
        self.assertIn('FOLLOW_PATH_CONTROLLER = "FollowPathDWB"', navigator)
        self.assertIn('"controller_id": self.active_controller_id', navigator)
        self.assertNotIn('"controller_route": {', navigator)
        self.assertIn("yaw_deg = 0.0", navigator)

        bridge_safety = (
            PACKAGE / "obstacle_detector" / "bridge_safety.py"
        ).read_text(encoding="utf-8")
        self.assertIn(
            "(TASK_NAV_BED1, TASK_NAV_BED3, TASK_NAV_HOME)",
            bridge_safety,
        )
        bridge = (
            PACKAGE / "obstacle_detector" / "stm32_bridge.py"
        ).read_text(encoding="utf-8")
        self.assertIn("nurse_scan_translation_only", bridge)

    def test_rviz_distinguishes_smoothed_candidate_from_controller_path(self):
        rviz = yaml.safe_load(
            (PACKAGE.parent / "3_可视化工具" / "medical_nav.rviz").read_text(
                encoding="utf-8"
            )
        )
        paths = {
            display["Topic"]["Value"]: display
            for display in rviz["Visualization Manager"]["Displays"]
            if display["Class"] == "rviz_default_plugins/Path"
        }
        raw = paths["/medical_nav/plan"]
        candidate = paths["/plan_smoothed"]
        followed = paths["/transformed_global_plan"]
        for display in (candidate, followed):
            self.assertTrue(display["Enabled"])
            # Humble's smoother/DWB publishers are volatile. A transient
            # local subscription would not match either publisher.
            self.assertEqual(display["Topic"]["Durability Policy"], "Volatile")
            self.assertEqual(display["Buffer Length"], 1)
        self.assertIn("Candidate", candidate["Name"])
        self.assertIn("Followed", followed["Name"])
        self.assertNotEqual(candidate["Color"], followed["Color"])
        self.assertLess(raw["Alpha"], followed["Alpha"])

        server = self.config["smoother_server"]["ros__parameters"]
        self.assertEqual(server["costmap_topic"], "global_costmap/costmap_raw")
        self.assertEqual(server["footprint_topic"], "global_costmap/published_footprint")

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
        self.assertEqual(local_params["robot_radius"], 0.24)
        self.assertEqual(global_params["robot_radius"], 0.24)
        local_footprint = ast.literal_eval(local_params["footprint"])
        global_footprint = ast.literal_eval(global_params["footprint"])
        self.assertEqual(local_footprint, global_footprint)
        self.assertEqual(len(local_footprint), 16)
        self.assertEqual(local_params["footprint_padding"], 0.0)
        self.assertEqual(global_params["footprint_padding"], 0.0)
        for x_value, y_value in local_footprint:
            self.assertAlmostEqual(
                (x_value ** 2 + y_value ** 2) ** 0.5, 0.24, places=3
            )
        self.assertEqual(global_params["update_frequency"], 4.0)
        self.assertEqual(planner_params["expected_planner_frequency"], 2.0)
        self.assertEqual(global_livox["observation_persistence"], 0.0)
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
        self.assertEqual(
            launch_source.count('"motion_compensation_enabled": True'), 2
        )
        # Both lidar filters use odometry for deskew, and the braking shadow
        # uses the same measured velocity only for physical-model diagnostics.
        self.assertEqual(launch_source.count('"odom_topic": "/odom"'), 4)
        self.assertEqual(
            launch_source.count('"max_motion_extrapolation_s": 0.06'), 2
        )
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
        self.assertIn('"min_points": 15', launch_source)
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

    def test_robot_self_returns_are_removed_before_motion_deskew(self):
        source = (
            PACKAGE / "obstacle_detector" / "lidar_transform.py"
        ).read_text(encoding="utf-8")
        callback = source.index("def _cloud_callback")
        deskew = source.index("compensated = deskew_points(", callback)

        for filter_call in (
            "keep_top_plate = top_plate_filter_mask(",
            "keep_self = polygon_self_filter_mask(",
            "keep_arm = arm_filter_mask(",
        ):
            self.assertLess(source.index(filter_call, callback), deskew)
        self.assertGreater(source.index("keep_ground, plane", callback), deskew)

    def test_blind_zones_are_absent_from_runtime_and_rviz(self):
        launch_source = (PACKAGE / "launch" / "obstacle.launch.py").read_text(
            encoding="utf-8"
        )
        limiter_source = (
            PACKAGE / "obstacle_detector" / "corner_speed_limiter.py"
        ).read_text(encoding="utf-8")
        core_source = (
            PACKAGE / "obstacle_detector" / "corner_speed_core.py"
        ).read_text(encoding="utf-8")
        setup_source = (PACKAGE / "setup.py").read_text(encoding="utf-8")
        rviz_source = (
            PACKAGE.parent / "3_可视化工具" / "medical_nav.rviz"
        ).read_text(encoding="utf-8")
        deploy_source = (
            PACKAGE.parent / "4_启动脚本" / "deploy_nuc.py"
        ).read_text(encoding="utf-8")
        recorder_source = (
            PACKAGE.parent / "4_启动脚本" / "record_bed13_3runs.sh"
        ).read_text(encoding="utf-8")
        self.assertIn("/trajectories", recorder_source)
        # Legacy/DWB trajectory output is optional; it must not prevent the
        # recorder from becoming ready when the unified DWB does not publish it.
        self.assertNotIn(
            '    "/medical_nav/wheel_odom",\n'
            '    "/odom",\n'
            '    "/trajectories",\n'
            '    "/livox/lidar_nav",',
            recorder_source,
        )
        self.assertIn("DWB_DIAGNOSTIC_TOPICS", recorder_source)
        self.assertIn("timeout 15s ros2 topic list -t", recorder_source)
        self.assertIn("timeout 15s ros2 node list", recorder_source)
        self.assertIn("setsid ros2 bag record", recorder_source)
        self.assertIn("timeout 90s bash -c", recorder_source)
        self.assertIn("/medical_nav/stp23l", recorder_source)
        self.assertIn("/medical_nav/stp23l/left", recorder_source)
        self.assertIn("/medical_nav/stp23l/front", recorder_source)
        self.assertIn("/medical_nav/stp23l/right", recorder_source)
        self.assertNotIn("/medical_nav/controller_selector", recorder_source)
        self.assertIn("/medical_nav/reference_speed_profile", recorder_source)
        self.assertIn("/medical_nav/reference_speed_status", recorder_source)
        self.assertIn('"REFERENCE_SPEED":', recorder_source)
        self.assertIn("/medical_nav/braking_shadow_cmd_vel", recorder_source)
        self.assertIn("/medical_nav/braking_shadow_status", recorder_source)
        self.assertIn('"BRAKE_SHADOW":', recorder_source)
        self.assertIn('"BRAKE_SHADOW_STATUS":', recorder_source)
        self.assertGreaterEqual(
            recorder_source.count('"/medical_nav/reference_speed_profile"'), 1
        )
        self.assertGreaterEqual(
            recorder_source.count('"/medical_nav/reference_speed_status"'), 2
        )
        self.assertGreaterEqual(
            recorder_source.count('"/medical_nav/braking_shadow_cmd_vel"'), 2
        )
        self.assertGreaterEqual(
            recorder_source.count('"/medical_nav/braking_shadow_status"'), 2
        )

        self.assertNotIn('executable="blind_zone_visualizer"', launch_source)
        self.assertNotIn("/medical_nav/blind_zones", launch_source)
        self.assertNotIn("/medical_nav/blind_zone_status", launch_source)
        self.assertNotIn('"blind_zone_enabled"', launch_source)
        self.assertNotIn('"blind_zone_half_width_deg"', launch_source)
        self.assertNotIn('"blind_zone_speed_m_s"', launch_source)
        self.assertNotIn('"blind_zone_min_overlap_m"', launch_source)
        self.assertNotIn('"blind_zone_lookahead_m"', launch_source)
        self.assertNotIn("blind_zone_", limiter_source)
        self.assertNotIn("blind_zone_speed_limit", limiter_source)
        self.assertNotIn("blind_zone_speed_limit", core_source)
        self.assertNotIn("BlindZoneSpeedDecision", limiter_source)
        self.assertNotIn("BlindZoneSpeedDecision", core_source)
        self.assertIn("blind_zone_visualizer =", setup_source)
        self.assertNotIn("Cross-Drive Lidar Blind Zones", rviz_source)
        self.assertNotIn("/medical_nav/blind_zones", rviz_source)
        self.assertNotIn(
            "grep -Fxq /blind_zone_visualizer",
            deploy_source,
        )
        self.assertNotIn("/medical_nav/blind_zone_status", deploy_source)
        self.assertNotIn("/medical_nav/blind_zone_status", recorder_source)
        self.assertIn('export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"', recorder_source)
        self.assertIn(
            'export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"',
            recorder_source,
        )
        for topic in (
            "/local_costmap/costmap_raw",
            "/local_costmap/costmap_updates",
            "/global_costmap/costmap_raw",
            "/global_costmap/costmap_updates",
        ):
            self.assertIn(topic, recorder_source)

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
        self.assertIn('"goal_handoff_hold_s": 0.0', launch_source)
        self.assertIn('"navigator_following_hold_s": 0.0', launch_source)
        self.assertIn('self.declare_parameter("goal_handoff_hold_s", 0.0)', bridge)
        self.assertIn('self.declare_parameter("navigator_following_hold_s", 0.0)', bridge)
        self.assertIn('self.declare_parameter("gate_release_wheel_accel_m_s2", 0.0)', bridge)
        self.assertIn('"navigator_status_timeout_s": 1.2', launch_source)
        self.assertIn('"goal_handoff_timeout_s": 1.0', launch_source)

    def test_collision_stop_requires_explicit_timestamped_clear(self):
        root = PACKAGE.parents[1]
        parser = (root / "Program" / "Core" / "Src" / "NUC_Obstacle.c").read_text(
            encoding="utf-8"
        )
        chassis = (root / "Program" / "Core" / "Src" / "main.c").read_text(
            encoding="utf-8"
        )
        protocol = (PACKAGE / "obstacle_detector" / "nav_protocol.py").read_text(
            encoding="utf-8"
        )
        bridge = (PACKAGE / "obstacle_detector" / "stm32_bridge.py").read_text(
            encoding="utf-8"
        )
        stop_check = parser.split("uint8_t NUC_Nav_EmergencyStopActive(void)", 1)[1]
        stop_check = stop_check.split("void NUC_Nav_ClearVelocity(void)", 1)[0]
        clear_handler = parser.split(
            "type == NAV_MSG_EMERGENCY_CLEAR && payload_len == 2U", 1
        )[1]
        clear_handler = clear_handler.split("else if", 1)[0]
        self.assertIn("s_nav_emergency_stop = 1U;", parser)
        self.assertIn("s_nav_emergency_stop = 0U;", parser)
        self.assertIn("NAV_MSG_EMERGENCY_CLEAR 0x44U", parser)
        self.assertIn("clear_stamp == s_nav_velocity_stamp_cs", clear_handler)
        self.assertIn("nav_stamp_is_newer(clear_stamp", clear_handler)
        self.assertNotIn("s_nav_forward_mm_s", clear_handler)
        self.assertIn("MSG_EMERGENCY_CLEAR = 0x44", protocol)
        self.assertIn("def encode_emergency_clear", protocol)
        self.assertIn("elif clear_emergency:", bridge)
        self.assertIn("MSG_EMERGENCY_CLEAR", bridge)
        self.assertRegex(
            bridge,
            r'source="task_gate",\s*clear_emergency=True',
        )
        self.assertRegex(
            bridge,
            r'source="nav2",\s*clear_emergency=True',
        )
        self.assertNotRegex(
            bridge,
            r'source="collision_stop",\s*clear_emergency=True',
        )
        self.assertIn("return active;", stop_check)
        self.assertNotIn("NAV_VELOCITY_TIMEOUT_MS", stop_check)
        self.assertIn("NUC_Nav_EmergencyStopActive()", chassis)
        self.assertIn("DJI_Chassis_SetCommandImmediate(0.0f, 0.0f, 0.0f)", chassis)

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
        self.assertIn(
            "NUC_Nav_RestartGoal(NUC_NAV_GOAL_BED1);",
            medical_task,
        )
        self.assertIn("TASK_WAIT_START", bridge)
        self.assertIn('"start_gate"', bridge)
        self.assertIn('status.get("plan_ready", False)', bridge)
        self.assertRegex(
            bridge,
            r"recovery_allowed=\(\s*self\.pose is not None\s*"
            r"and self\.pose\.task_state in START_WAIT_TASK_STATES\s*\)",
        )
        self.assertNotIn(
            "and self.pose.task_state == TASK_WAIT_START",
            bridge,
        )
        self.assertIn('"plan_ready": self.plan_ready', navigator)
        self.assertIn("len(msg.poses) > 0", navigator)
        self.assertIn(
            "self, SmoothPath, \"smooth_path\"",
            navigator,
        )
        self.assertIn(
            '"prestart_smoothed_path_ready"',
            navigator,
        )
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

    def test_bed_docking_keeps_moving_handoff_and_stopped_fallback(self):
        root = PACKAGE.parents[1]
        medical_task = (root / "Program" / "Core" / "Src" / "MedicalTask.c").read_text(
            encoding="utf-8"
        )
        medical_header = (root / "Program" / "Core" / "Inc" / "MedicalTask.h").read_text(
            encoding="utf-8"
        )
        main_source = (root / "Program" / "Core" / "Src" / "main.c").read_text(
            encoding="utf-8"
        )
        bridge_source = (
            PACKAGE / "obstacle_detector" / "stm32_bridge.py"
        ).read_text(encoding="utf-8")
        self.assertIn("MEDICAL_DOCK_SAMPLE_COUNT 8U", medical_task)
        self.assertIn("MEDICAL_DOCK_SAMPLE_TRIM_COUNT 1U", medical_task)
        self.assertIn("MEDICAL_DOCK_SAMPLE_SETTLE_MS 100U", medical_task)
        self.assertIn("MEDICAL_DOCK_SAMPLE_TIMEOUT_MS 1500U", medical_task)
        self.assertIn("MEDICAL_DOCK_FALLBACK_STILL_MS 2000U", medical_task)
        self.assertIn("MEDICAL_DOCK_FALLBACK_SAMPLE_COUNT 5U", medical_task)
        self.assertIn("MEDICAL_DOCK_VERIFY_SAMPLE_COUNT 1U", medical_task)
        self.assertIn("medical_docking_skip_after_max_corrections", medical_task)
        self.assertIn("MEDICAL_DOCK_STILL_LINEAR_MM_S 30.0f", medical_task)
        self.assertIn("MEDICAL_DOCK_STILL_YAW_CDEG_S 100.0f", medical_task)
        self.assertIn("MEDICAL_DOCK_TRACK_MIN_SAMPLES 3U", medical_task)
        self.assertIn("MEDICAL_DOCK_TRACK_PREFERRED_SPREAD_MM 10.0f", medical_task)
        self.assertIn("MEDICAL_DOCK_TRACK_FALLBACK_SPREAD_MM 40.0f", medical_task)
        self.assertRegex(
            medical_task,
            r"best_spread > MEDICAL_DOCK_TRACK_PREFERRED_SPREAD_MM[\s\S]*?"
            r"sample_count < MEDICAL_DOCK_SAMPLE_COUNT[\s\S]*?"
            r"best_spread > MEDICAL_DOCK_TRACK_FALLBACK_SPREAD_MM",
        )
        self.assertIn("MEDICAL_DOCK_FRONT_TARGET_MM 350.0f", medical_task)
        self.assertIn(
            "MEDICAL_DOCK_BED1_MEDICINE1_SIDE_TARGET_MM 1137.0f", medical_task
        )
        self.assertIn(
            "MEDICAL_DOCK_BED1_MEDICINE3_SIDE_TARGET_MM 1100.0f", medical_task
        )
        self.assertIn(
            "MEDICAL_DOCK_BED3_MEDICINE1_SIDE_TARGET_MM 1137.0f", medical_task
        )
        self.assertIn(
            "MEDICAL_DOCK_BED3_MEDICINE3_SIDE_TARGET_MM 1152.0f", medical_task
        )
        self.assertIn("MEDICAL_DOCK_TOLERANCE_MM 5", medical_task)
        self.assertNotIn("MEDICAL_DOCK_BOX_CENTER_SEPARATION_MM", medical_task)
        self.assertIn("MEDICAL_DOCK_TRACK_RADIUS_MM 800.0f", medical_task)
        self.assertIn("MEDICAL_DOCK_TRACK_YAW_DEG 3.0f", medical_task)
        self.assertNotIn("MEDICAL_DOCK_HANDOFF_MAX_SPEED_MM_S", medical_task)
        self.assertNotIn("MEDICAL_DOCK_HANDOFF_STABLE_MS", medical_task)
        self.assertNotIn("MEDICAL_DOCK_SAFE_FORWARD_HALF_WIDTH_MM", medical_task)
        self.assertNotIn("MEDICAL_DOCK_SAFE_APPROACH_DEPTH_MM", medical_task)
        self.assertNotIn("MEDICAL_DOCK_SAFE_OVERSHOOT_MM", medical_task)
        self.assertIn("MEDICAL_DOCK_TIMEOUT_MS 8000U", medical_task)
        self.assertIn("MEDICAL_DOCK_SIDE_SPLIT_THRESHOLD_MM 1600U", medical_task)
        self.assertIn("MEDICAL_DOCK_LASER_MAX_CORRECTION_MM 400.0f", medical_task)
        self.assertIn("MEDICAL_DOCK_SPEED_LOC_K 1000.0f", medical_task)
        self.assertIn("MEDICAL_DOCK_TIMEOUT_BEEP_FIRST_MS 200U", medical_task)
        self.assertIn("MEDICAL_DOCK_TIMEOUT_BEEP_GAP_MS 100U", medical_task)
        self.assertIn("MEDICAL_DOCK_TIMEOUT_BEEP_SECOND_MS 300U", medical_task)
        self.assertRegex(
            medical_task,
            r"if \(s_current_bed == 1U\)\s*"
            r"\{\s*return STP23L_GetSampleA\(distance_mm, frame_sequence\);\s*\}\s*"
            r"return STP23L_GetSampleC\(distance_mm, frame_sequence\);",
        )
        self.assertIn(
            '"a": ("left", "stp23l_left", telemetry.a_mm, 0x01)',
            bridge_source,
        )
        self.assertIn(
            '"b": ("front", "stp23l_front", telemetry.b_mm, 0x02)',
            bridge_source,
        )
        self.assertIn(
            '"c": ("right", "stp23l_right", telemetry.c_mm, 0x04)',
            bridge_source,
        )
        self.assertIn(
            '"a": self.create_publisher(Range, "/medical_nav/stp23l/left", 10)',
            bridge_source,
        )
        self.assertIn(
            '"c": self.create_publisher(Range, "/medical_nav/stp23l/right", 10)',
            bridge_source,
        )
        self.assertIn('"a_left_mm": self.stp23l.a_mm', bridge_source)
        self.assertIn('"c_right_mm": self.stp23l.c_mm', bridge_source)
        self.assertIn("medical_is_bed_navigation_state() != 0U", medical_task)
        self.assertNotIn("medical_docking_in_safe_corridor", medical_task)
        self.assertNotIn("medical_docking_handoff_target", medical_task)
        self.assertIn("medical_docking_capture_targets", medical_task)
        self.assertIn("medical_docking_get_tracked_target", medical_task)
        self.assertIn("medical_docking_finish_or_correct_tracked", medical_task)
        self.assertRegex(
            medical_task,
            r"medical_is_bed_navigation_state\(\) != 0U[\s\S]*?"
            r"medical_docking_capture_targets[\s\S]*?"
            r"medical_docking_get_tracked_target[\s\S]*?"
            r"medical_set_state[\s\S]*?"
            r"medical_docking_start_absolute_move[\s\S]*?return 1U;",
        )
        self.assertIn("medical_docking_collect_samples", medical_task)
        self.assertIn("medical_docking_collect_front_samples", medical_task)
        self.assertRegex(
            medical_task,
            r"s_dock_phase == MEDICAL_DOCK_HANDOFF[\s\S]*?"
            r"planar_speed > MEDICAL_DOCK_STILL_LINEAR_MM_S[\s\S]*?"
            r"HAL_GetTick\(\) - s_dock_still_since_ms[\s\S]*?"
            r"MEDICAL_DOCK_FALLBACK_STILL_MS[\s\S]*?"
            r"medical_docking_sample_reset\(\);",
        )
        self.assertRegex(
            medical_task,
            r"sample_goal = \(s_dock_phase == MEDICAL_DOCK_SAMPLE_INITIAL\)"
            r"[\s\S]*?MEDICAL_DOCK_FALLBACK_SAMPLE_COUNT[\s\S]*?"
            r"medical_docking_collect_samples\(sample_goal\)",
        )
        self.assertIn("float measured_yaw_ccw_cdeg_s", medical_header)
        self.assertRegex(
            main_source,
            r"MedicalTask_DockingControl\([\s\S]*?"
            r"measured_forward_mm_s, measured_left_mm_s,[\s\S]*?"
            r"measured_yaw_ccw_cdeg_s\);",
        )
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

    def test_formal_workflow_enables_prestart_order_scan_arm_and_tts(self):
        root = PACKAGE.parents[1]
        medical_task = (
            root / "Program" / "Core" / "Src" / "MedicalTask.c"
        ).read_text(encoding="utf-8")
        launch_source = (
            PACKAGE / "launch" / "obstacle.launch.py"
        ).read_text(encoding="utf-8")
        bridge_source = (
            PACKAGE / "obstacle_detector" / "stm32_bridge.py"
        ).read_text(encoding="utf-8")
        dashboard_source = (
            PACKAGE / "obstacle_detector" / "scan_dashboard.py"
        ).read_text(encoding="utf-8")

        self.assertIn("MEDICAL_NAVIGATION_TIMING_TEST 0U", medical_task)
        self.assertIn("medical_set_state(MEDICAL_TASK_WAIT_START);", medical_task)
        update_source = medical_task.split("void MedicalTask_Update(void)", 1)[1]
        wait_start = update_source.split("case MEDICAL_TASK_WAIT_START:", 1)[1]
        wait_start = wait_start.split("case MEDICAL_TASK_WAIT_BED1_START:", 1)[0]
        self.assertIn("medical_take_scan", wait_start)
        self.assertIn("MEDICAL_TASK_WAIT_BED1_START", wait_start)
        self.assertIn("MEDICAL_TASK_WAIT_BED3_START", wait_start)
        self.assertIn("medical_set_state(MEDICAL_TASK_NAV_NURSE);", wait_start)
        self.assertIn(
            "medical_handle_nav_result(MEDICAL_TASK_DOCK_BED1);",
            medical_task,
        )
        self.assertIn(
            "medical_handle_nav_result(MEDICAL_TASK_DOCK_BED3);",
            medical_task,
        )
        self.assertIn("medical_arm_set_deployed(1U);", medical_task)
        self.assertIn("MEDICAL_DISPENSE_ANNOUNCE_DELAY_MS 700U", medical_task)
        self.assertIn("NUC_Nav_RequestTTS(s_current_bed)", medical_task)
        self.assertIn("NUC_TTS_COMPLETED", medical_task)
        self.assertIn(
            'DeclareLaunchArgument("scanner_enabled", default_value="true")',
            launch_source,
        )
        self.assertIn('QPushButton("一号语音")', dashboard_source)
        self.assertIn('QPushButton("三号语音")', dashboard_source)
        self.assertIn('"/medical_nav/tts_debug_command"', dashboard_source)
        self.assertIn('"/medical_nav/tts_debug_command"', bridge_source)
        self.assertIn("self.tts_player.speak_bed, bed", bridge_source)
        self.assertIn("def _tts_debug_allowed(self)", bridge_source)

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
            r"NUC_Nav_RestartGoal\(NUC_NAV_GOAL_BED1\);",
        )
        self.assertRegex(
            medical_task,
            r"case MEDICAL_TASK_NAV_HOME:[\s\S]*?"
            r"medical_arm_set_deployed\(0U\);[\s\S]*?"
            r"NUC_Nav_RestartGoal\(NUC_NAV_GOAL_HOME\);",
        )
        self.assertIn("pos_default.max_out = 6000.f;", motor_control)

    def test_stm32_uses_cross_drive_and_matching_odometry(self):
        root = PACKAGE.parents[1]
        motor_control = (
            root / "Program" / "Core" / "Src" / "DJIMotorCtrlSTM32.cpp"
        ).read_text(encoding="utf-8")
        nav_transport = (
            root / "Program" / "Core" / "Src" / "NUC_Obstacle.c"
        ).read_text(encoding="utf-8")

        self.assertNotIn("kInvSqrt2", motor_control)
        self.assertIn("out_rpm[0] = Vy + W", motor_control)
        self.assertIn("out_rpm[1] = -Vx + W", motor_control)
        self.assertIn("out_rpm[2] = -Vy + W", motor_control)
        self.assertIn("out_rpm[3] = Vx + W", motor_control)
        self.assertIn("(wheel_rpm[3] - wheel_rpm[1]) * 0.5f", motor_control)
        self.assertIn("(wheel_rpm[0] - wheel_rpm[2]) * 0.5f", motor_control)
        self.assertIn("kMaxAxisSpeedMmS = 3000.0f", motor_control)
        self.assertNotIn("fminf(", motor_control)
        self.assertNotIn("fmaxf(", motor_control)
        self.assertIn("kMaxWheelSurfaceSpeedMmS = 3000.0f", motor_control)
        self.assertNotIn("kBodyAccelRpmPerS", motor_control)
        self.assertNotIn("kYawAccelRpmPerS", motor_control)
        self.assertNotIn("slew_command(", motor_control)
        self.assertIn("speed_default.kp = 8.0f;", motor_control)
        self.assertIn("speed_default.ki = 1.f;", motor_control)
        self.assertIn("speed_default.kd = 0.01f;", motor_control)
        self.assertIn("speed_default.max_out = speed_max_out;", motor_control)
        self.assertIn("g_chassis_bus.attach(can_, 13000.f);", motor_control)
        self.assertIn("g_arm_bus.attach(can_, 10000.f);", motor_control)
        self.assertIn("#define NAV_STM32_MAX_LINEAR_MM_S 3000", nav_transport)

    def test_ops_native_axes_rotate_into_physical_field_frame(self):
        root = PACKAGE.parents[1]
        ops_source = (
            root / "Program" / "Core" / "Src" / "OPS.c"
        ).read_text(encoding="utf-8")

        self.assertIn(
            "OPS_CROSS_FRAME_INV_SQRT2 0.70710678118654752440f",
            ops_source,
        )
        self.assertIn(
            "*field_x = (native_x - native_y) * OPS_CROSS_FRAME_INV_SQRT2;",
            ops_source,
        )
        self.assertIn(
            "*field_y = (native_x + native_y) * OPS_CROSS_FRAME_INV_SQRT2;",
            ops_source,
        )
        self.assertGreaterEqual(ops_source.count("OPS_NativeToFieldXY("), 4)

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
        self.assertIn("MEDICAL_BED_SCAN_TIMEOUT_MS 5000U", medical_task)
        self.assertIn("s_bed_scan_timeout_fallback", medical_task)
        self.assertIn("MEDICAL_DOCK_SAMPLE_COUNT 8U", medical_task)
        self.assertIn("MEDICAL_DOCK_SAMPLE_TRIM_COUNT 1U", medical_task)
        self.assertIn("MEDICAL_DOCK_SAMPLE_TIMEOUT_MS 1500U", medical_task)
        self.assertIn("CHASSIS_CTRL_REACH_X_MM 5.0f", chassis_control)
        self.assertIn("CHASSIS_CTRL_REACH_Y_MM 5.0f", chassis_control)
        self.assertIn("CHASSIS_CTRL_REACH_YAW_DEG 0.5f", chassis_control)
        self.assertNotIn("CHASSIS_CTRL_XY_SLEW_RPM_PER_S", chassis_control)
        self.assertNotIn("CHASSIS_CTRL_YAW_SLEW_RPM_PER_S", chassis_control)
        self.assertNotIn("chassis_ctrl_slew(", chassis_control)
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
        self.assertEqual(scanner["tele_fps"], 30.0)
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
        bridge_source = (
            PACKAGE / "obstacle_detector" / "stm32_bridge.py"
        ).read_text(encoding="utf-8")
        dashboard_source = (
            PACKAGE / "obstacle_detector" / "scan_dashboard.py"
        ).read_text(encoding="utf-8")
        self.assertIn("tele_camera_assists(self.task_state, format)", scanner_source)
        self.assertIn("TASK_WAIT_START,", scanner_source)
        scanner_imports = scanner_source.split("from .scanner_core import (", 1)[1]
        scanner_imports = scanner_imports.split(")", 1)[0]
        self.assertNotIn("TASK_WAIT_START", scanner_imports)
        bridge_imports = bridge_source.split("from .scanner_core import (", 1)[1]
        bridge_imports = bridge_imports.split(")", 1)[0]
        self.assertNotIn("TASK_WAIT_START", bridge_imports)
        self.assertIn('source = f"tele_{source}"', scanner_source)
        self.assertIn(
            'profiles = (("MJPG", True), ("driver-default", False))',
            scanner_source,
        )
        self.assertIn("self._decode_fast(tele_gray, format)", scanner_source)
        self.assertIn("self._decode(tele_full_gray, format)", scanner_source)
        self.assertNotIn("camera_handoff_lock", scanner_source)
        self.assertNotIn("_tele_capture_required", scanner_source)
        self.assertIn("cv2.VideoWriter_fourcc(*profile_name)", scanner_source)
        self.assertIn("self.camera_open_lock = threading.Lock()", scanner_source)
        self.assertIn("with self.camera_open_lock:", scanner_source)
        self.assertIn("self.tele_state_lock = threading.RLock()", scanner_source)
        self.assertIn('"tele_scan_state": tele_scan_state', scanner_source)
        self.assertIn('if source.startswith("tele_"):', scanner_source)
        self.assertIn('"mission_epoch": self.mission_epoch', scanner_source)
        self.assertIn('self.tele_camera_view.setText("失败")', dashboard_source)
        self.assertIn("长焦辅助摄像头", dashboard_source)
        self.assertIn(
            '"/medical_nav/tele_scanner_preview/compressed"', dashboard_source
        )

    def test_cold_start_has_automatic_recovery_and_responsive_dashboard(self):
        root = PACKAGE.parents[1]
        startup = (root / "NUC_MID360" / "4_启动脚本" / "start_obstacle.sh").read_text(
            encoding="utf-8"
        )
        mid360_startup = (
            root / "NUC_MID360" / "4_启动脚本" / "start_mid360.sh"
        ).read_text(encoding="utf-8")
        mid360_service = (
            root / "NUC_MID360" / "4_启动脚本" / "mid360.service"
        ).read_text(encoding="utf-8")
        service = (
            root / "NUC_MID360" / "4_启动脚本" / "obstacle-detector.service"
        ).read_text(encoding="utf-8")
        dashboard_start = (
            root / "NUC_MID360" / "4_启动脚本" / "start_scan_dashboard.sh"
        ).read_text(encoding="utf-8")
        dashboard = (
            PACKAGE / "obstacle_detector" / "scan_dashboard.py"
        ).read_text(encoding="utf-8")
        deploy = (
            root / "NUC_MID360" / "4_启动脚本" / "deploy_nuc.py"
        ).read_text(encoding="utf-8")

        self.assertIn("ros2 run obstacle_detector startup_probe", startup)
        self.assertNotIn("ros2 lifecycle get", startup)
        self.assertIn("wait_for_lidar_sample", startup)
        self.assertIn("exit 69", startup)
        self.assertIn("exit 70", startup)
        self.assertIn("/livox/lidar", mid360_startup)
        self.assertIn("MID360_STARTUP_TIMEOUT_S", mid360_startup)
        self.assertIn("exit 69", mid360_startup)
        self.assertIn("StartLimitIntervalSec=0", mid360_service)
        self.assertIn("Restart=on-failure", service)
        self.assertIn("StartLimitIntervalSec=180", service)
        self.assertIn("StartLimitBurst=3", service)
        self.assertIn("KillMode=mixed", mid360_service)
        self.assertIn("KillMode=control-group", service)
        for unit in (mid360_service, service):
            self.assertIn("KillSignal=SIGINT", unit)
            self.assertIn("TimeoutStopSec=12", unit)
            self.assertIn("SendSIGKILL=yes", unit)
        for script in (mid360_startup, startup):
            self.assertIn("handle_stop()", script)
            self.assertIn("deadline=$((SECONDS + 6))", script)
            self.assertIn("deadline=$((SECONDS + 2))", script)
            self.assertIn("kill -TERM \"$launch_pid\"", script)
            self.assertIn("kill -KILL \"$launch_pid\"", script)
            self.assertNotIn("trap stop_launch INT TERM EXIT", script)
        self.assertIn("SupplementaryGroups=dialout video audio", service)
        self.assertIn("QT_AUTO_SCREEN_SCALE_FACTOR=0", dashboard_start)
        self.assertIn("QT_ENABLE_HIGHDPI_SCALING=0", dashboard_start)
        self.assertIn("unset QT_SCREEN_SCALE_FACTORS", dashboard_start)
        self.assertNotIn("flock -n", dashboard_start)
        self.assertNotIn("medical-scan-dashboard.pid", dashboard_start)
        self.assertIn("Qt.AA_DisableHighDpiScaling", dashboard)
        self.assertIn("def _acquire_instance_lock", dashboard)
        self.assertIn("fcntl.LOCK_EX | fcntl.LOCK_NB", dashboard)
        self.assertIn("medical-scan-dashboard.pid", dashboard)
        self.assertIn("_release_instance_lock(instance_lock)", dashboard)
        self.assertNotIn("setFixedHeight", dashboard)
        self.assertNotIn("setGeometry(screen.geometry())", dashboard)
        self.assertIn("def minimumSizeHint", dashboard)
        self.assertIn("return False", dashboard)
        self.assertIn("Qt.KeepAspectRatio", dashboard)
        self.assertIn("SCAN_CAMERA_OVERRIDE is None", deploy)
        self.assertIn("TELE_SCAN_CAMERA_OVERRIDE is None", deploy)
        self.assertNotIn("medical_nav_releases", deploy)
        self.assertNotIn("medical_nav_current", deploy)
        self.assertNotIn("route_preflight", deploy)
        self.assertIn("Confirming the robot is stationary", deploy)
        self.assertIn("Checking Mid360 without restarting it", deploy)
        self.assertNotIn("systemctl restart mid360.service", deploy)
        self.assertIn("ros2 run obstacle_detector startup_probe", startup)
        self.assertIn("systemd-notify --ready", startup)
        self.assertIn("Type=notify", service)
        self.assertIn("NotifyAccess=all", service)
        self.assertNotIn("ros2 lifecycle get", deploy)
        self.assertIn("SYSTEMD_PAGER=cat", deploy)
        self.assertIn("sudo -S -p ''", deploy)
        self.assertNotIn("sudo -S\"", deploy)
        self.assertIn("systemctl restart --no-block obstacle-detector.service", deploy)
        self.assertIn("systemctl reset-failed obstacle-detector.service", deploy)
        self.assertIn("navigation ready after", deploy)
        self.assertIn(
            "pkill -TERM -f 'obstacle_detector[/]scan_dashboard'", deploy
        )
        self.assertIn("dashboard_pids=$(pgrep -f", deploy)
        self.assertIn(r'[ \"$#\" -eq 1 ]', deploy)


if __name__ == "__main__":
    unittest.main()
