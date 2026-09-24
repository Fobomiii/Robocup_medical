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
        self.assertEqual(follow_path["vy_std"], 0.25)
        self.assertEqual(follow_path["prune_distance"], 3.0)
        self.assertEqual(follow_path["temperature"], 0.25)
        self.assertEqual(
            follow_path["PathFollowCritic"]["offset_from_furthest"], 10
        )
        self.assertEqual(follow_path["PathAlignCritic"]["cost_weight"], 8.0)
        self.assertEqual(follow_path["PathAngleCritic"]["cost_weight"], 1.5)
        self.assertFalse(follow_path["CostCritic"]["consider_footprint"])
        self.assertFalse(follow_path["visualize"])

    def test_omni_controller_can_reverse_without_forcing_path_heading(self):
        follow_path = self.config["controller_server"]["ros__parameters"]["FollowPath"]
        self.assertEqual(follow_path["motion_model"], "Omni")
        self.assertEqual(follow_path["vx_min"], -0.40)
        self.assertTrue(follow_path["PathAngleCritic"]["enabled"])
        self.assertEqual(follow_path["PathAngleCritic"]["mode"], 1)
        self.assertTrue(follow_path["GoalAngleCritic"]["enabled"])
        self.assertEqual(
            follow_path["GoalAngleCritic"]["threshold_to_consider"], 1.2
        )

    def test_collision_monitor_has_predictive_and_emergency_guards(self):
        monitor = self.config["collision_monitor"]["ros__parameters"]
        self.assertEqual(
            monitor["polygons"], ["ApproachPolygon", "StopPolygon"]
        )

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

    def test_behavior_tree_replans_at_one_hertz(self):
        root = ET.parse(BT_XML).getroot()
        rate_controllers = root.findall(".//RateController")
        self.assertEqual(len(rate_controllers), 1)
        self.assertEqual(float(rate_controllers[0].attrib["hz"]), 1.0)
        self.assertNotIn("error_code_id", BT_XML.read_text(encoding="utf-8"))

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

    def test_global_obstacles_are_stable_without_slowing_local_clearing(self):
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
        self.assertEqual(global_params["update_frequency"], 2.0)
        self.assertEqual(planner_params["expected_planner_frequency"], 1.0)
        self.assertEqual(global_livox["observation_persistence"], 0.3)
        self.assertEqual(global_livox["obstacle_max_range"], 4.5)
        self.assertEqual(global_livox["raytrace_max_range"], 5.0)
        self.assertEqual(local_livox["observation_persistence"], 0.0)

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
        self.assertIn('"base_radius": 0.155', launch_source)

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

    def test_scanner_uses_full_frame_and_bed_proximity_gate(self):
        scanner = yaml.safe_load(SCANNER_PARAMS.read_text(encoding="utf-8"))[
            "code_scanner"
        ]["ros__parameters"]
        self.assertEqual(scanner["roi_left"], 0.0)
        self.assertEqual(scanner["roi_right"], 1.0)
        self.assertEqual(scanner["roi_top"], 0.0)
        self.assertEqual(scanner["roi_bottom"], 1.0)
        self.assertEqual(scanner["bed_scan_activation_distance_m"], 1.2)
        self.assertEqual(scanner["pose_timeout_s"], 0.5)

        launch_source = (PACKAGE / "launch" / "obstacle.launch.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('"bed_scan_activation_distance_m": 1.2', launch_source)
        self.assertIn('"field_config": field_map', launch_source)


if __name__ == "__main__":
    unittest.main()
