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
        follow_path = self.config["controller_server"]["ros__parameters"][
            "FollowPath"
        ]
        self.assertEqual(follow_path["time_steps"], 40)
        self.assertEqual(follow_path["model_dt"], 0.05)
        self.assertAlmostEqual(
            follow_path["time_steps"] * follow_path["model_dt"], 2.0
        )
        self.assertEqual(follow_path["batch_size"], 1200)
        self.assertEqual(follow_path["vx_std"], 0.30)
        self.assertEqual(follow_path["vy_std"], 0.18)
        self.assertEqual(follow_path["wz_std"], 0.15)
        self.assertEqual(follow_path["prune_distance"], 3.0)
        self.assertEqual(follow_path["temperature"], 0.25)
        self.assertEqual(
            follow_path["PathFollowCritic"]["offset_from_furthest"], 8
        )
        self.assertEqual(follow_path["PathAlignCritic"]["cost_weight"], 10.0)
        self.assertEqual(follow_path["PathAngleCritic"]["cost_weight"], 1.5)
        self.assertFalse(follow_path["CostCritic"]["consider_footprint"])
        self.assertFalse(follow_path["visualize"])

    def test_omni_controller_holds_task_yaw_during_translation(self):
        follow_path = self.config["controller_server"]["ros__parameters"]["FollowPath"]
        self.assertEqual(follow_path["motion_model"], "Omni")
        self.assertEqual(follow_path["vx_min"], -1.20)
        self.assertEqual(follow_path["vy_max"], 1.5)
        self.assertFalse(follow_path["PathAngleCritic"]["enabled"])
        self.assertEqual(follow_path["PathAngleCritic"]["mode"], 1)
        self.assertTrue(follow_path["GoalAngleCritic"]["enabled"])
        self.assertEqual(follow_path["GoalAngleCritic"]["cost_weight"], 10.0)
        self.assertEqual(
            follow_path["GoalAngleCritic"]["threshold_to_consider"], 10.0
        )
        self.assertIn("TwirlingCritic", follow_path["critics"])
        self.assertTrue(follow_path["TwirlingCritic"]["enabled"])
        self.assertEqual(
            follow_path["TwirlingCritic"]["twirling_cost_weight"], 10.0
        )

    def test_collision_monitor_restores_field_tested_single_stage_profile(self):
        normal_smoother = self.config["velocity_smoother"]["ros__parameters"]
        monitor = self.config["collision_monitor"]["ros__parameters"]

        # Ordinary motion is ramp-limited before entering the safety chain.
        self.assertEqual(normal_smoother["max_accel"], [1.5, 1.5, 1.8])
        self.assertEqual(normal_smoother["max_decel"], [-1.3, -1.3, -2.0])
        self.assertNotIn("collision_monitor_predictive", self.config)
        self.assertEqual(monitor["polygons"], ["ApproachPolygon", "StopPolygon"])
        self.assertEqual(monitor["cmd_vel_in_topic"], "/cmd_vel_home_limited")
        self.assertEqual(monitor["cmd_vel_out_topic"], "/cmd_vel_safe")
        self.assertNotIn("safety_velocity_smoother", self.config)

        approach = monitor["ApproachPolygon"]
        self.assertEqual(approach["action_type"], "approach")
        self.assertEqual(approach["time_before_collision"], 0.8)
        self.assertEqual(approach["simulation_time_step"], 0.1)
        self.assertEqual(approach["max_points"], 3)

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
        self.assertIn('"input_topic": "/cmd_vel"', launch_source)
        self.assertIn('"output_topic": "/cmd_vel_home_limited"', launch_source)
        self.assertIn('"home_task_state": 9', launch_source)
        self.assertIn('"bed1_task_state": 3', launch_source)
        self.assertIn('"bed3_task_state": 6', launch_source)
        self.assertIn('"max_speed_m_s": 2.0', launch_source)
        self.assertIn('"soft_decel_m_s2": 1.425', launch_source)
        self.assertIn('"terminal_speed_m_s": 0.10', launch_source)
        self.assertIn('"terminal_distance_m": 0.10', launch_source)
        self.assertIn('"bed_max_speed_m_s": 2.0', launch_source)
        self.assertIn('"bed_soft_decel_m_s2": 1.0', launch_source)
        self.assertIn('"bed_terminal_speed_m_s": 0.15', launch_source)
        self.assertIn('"bed_terminal_distance_m": 0.20', launch_source)
        self.assertIn('"pose_timeout_s": 0.5', launch_source)
        self.assertNotIn('"collision_monitor_predictive",', launch_source)

        setup_source = (PACKAGE / "setup.py").read_text(encoding="utf-8")
        self.assertIn(
            "home_approach_limiter = obstacle_detector.home_approach_limiter:main",
            setup_source,
        )

    def test_behavior_trees_replan_at_two_hertz(self):
        root = ET.parse(BT_XML).getroot()
        rate_controllers = root.findall(".//RateController")
        self.assertEqual(len(rate_controllers), 1)
        self.assertEqual(float(rate_controllers[0].attrib["hz"]), 2.0)
        self.assertNotIn("error_code_id", BT_XML.read_text(encoding="utf-8"))

        nurse_root = ET.parse(NURSE_BT_XML).getroot()
        nurse_rate_controllers = nurse_root.findall(".//RateController")
        self.assertEqual(len(nurse_rate_controllers), 1)
        self.assertEqual(float(nurse_rate_controllers[0].attrib["hz"]), 2.0)

        bt_params = self.config["bt_navigator"]["ros__parameters"]
        self.assertEqual(
            bt_params["default_bt_xml_filename"], BT_XML.name
        )

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

        self.assertEqual(local_params["width"], 8)
        self.assertEqual(local_params["height"], 8)
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
        self.assertEqual(local_livox["obstacle_max_range"], 3.0)
        self.assertEqual(local_livox["raytrace_max_range"], 3.5)

    def test_real_chassis_output_remains_default(self):
        launch_source = (PACKAGE / "launch" / "obstacle.launch.py").read_text(
            encoding="utf-8"
        )
        self.assertIn(
            'DeclareLaunchArgument("dry_run", default_value="false")',
            launch_source,
        )
        self.assertIn("RewrittenYaml", launch_source)
        self.assertIn('"default_bt_xml_filename": bt_xml_file', launch_source)

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
        self.assertEqual(monitor["livox"]["topic"], "/livox/lidar_nav")
        self.assertEqual(local_livox["topic"], "/livox/lidar_nav")
        self.assertEqual(global_livox["topic"], "/livox/lidar_nav")

        launch_source = (PACKAGE / "launch" / "obstacle.launch.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('executable="cone_footprint_compensator"', launch_source)
        self.assertIn('"cone_height": 0.65', launch_source)
        self.assertIn('"physical_base_radius": 0.155', launch_source)
        self.assertIn('"base_radius": 0.165', launch_source)
        self.assertIn('"max_range": 6.0', launch_source)
        self.assertIn('"persistence_s": 0.35', launch_source)
        self.assertIn('"persistence_frame": "odom"', launch_source)

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

    def test_abnormal_bed_docking_splits_side_and_front_moves(self):
        root = PACKAGE.parents[1]
        medical_task = (root / "Program" / "Core" / "Src" / "MedicalTask.c").read_text(
            encoding="utf-8"
        )
        self.assertIn("MEDICAL_DOCK_SIDE_SPLIT_THRESHOLD_MM 1600U", medical_task)
        self.assertRegex(
            medical_task,
            r"if \(s_state == MEDICAL_TASK_DOCK_BED1\)\s*"
            r"\{\s*return STP23L_GetSampleC\(distance_mm, frame_sequence\);\s*\}\s*"
            r"return STP23L_GetSampleA\(distance_mm, frame_sequence\);",
        )
        self.assertIn("MEDICAL_DOCK_MOVE_SPLIT_SIDE", medical_task)
        self.assertIn("MEDICAL_DOCK_SAMPLE_SPLIT_FRONT", medical_task)
        self.assertIn("MEDICAL_DOCK_MOVE_SPLIT_FRONT", medical_task)
        self.assertIn("medical_docking_collect_front_samples", medical_task)

    def test_arm_overlaps_final_docking_and_retracts_during_navigation(self):
        root = PACKAGE.parents[1]
        medical_task = (root / "Program" / "Core" / "Src" / "MedicalTask.c").read_text(
            encoding="utf-8"
        )
        motor_control = (
            root / "Program" / "Core" / "Src" / "DJIMotorCtrlSTM32.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn("MEDICAL_ARM_DEPLOY_TIME_S 1.5f", medical_task)
        self.assertIn("MEDICAL_ARM_RETRACT_TIME_S 2.0f", medical_task)
        self.assertRegex(
            medical_task,
            r"move_phase == MEDICAL_DOCK_MOVE_COMBINED\) \|\|\s*"
            r"\(move_phase == MEDICAL_DOCK_MOVE_SPLIT_FRONT\)",
        )
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

    def test_bed_workflow_uses_bounded_fast_docking(self):
        root = PACKAGE.parents[1]
        medical_task = (root / "Program" / "Core" / "Src" / "MedicalTask.c").read_text(
            encoding="utf-8"
        )
        chassis_control = (
            root / "Program" / "Core" / "Src" / "ChassisCtrl.c"
        ).read_text(encoding="utf-8")

        self.assertIn("MEDICAL_DISPENSE_HOLD_MS 2500U", medical_task)
        self.assertIn("MEDICAL_DOCK_SAMPLE_COUNT 10U", medical_task)
        self.assertIn("MEDICAL_DOCK_SAMPLE_TRIM_COUNT 1U", medical_task)
        self.assertIn("MEDICAL_DOCK_SAMPLE_SETTLE_MS 100U", medical_task)
        self.assertIn("CHASSIS_CTRL_REACH_X_MM 5.0f", chassis_control)
        self.assertIn("CHASSIS_CTRL_REACH_Y_MM 5.0f", chassis_control)
        self.assertIn("CHASSIS_CTRL_REACH_YAW_DEG 0.5f", chassis_control)

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
