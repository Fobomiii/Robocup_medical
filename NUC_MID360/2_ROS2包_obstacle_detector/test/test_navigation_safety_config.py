"""Static checks for the live-replanning and collision-monitor profile."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
PARAMS = PACKAGE / "config" / "nav2_params.yaml"
BT_XML = PACKAGE / "config" / "navigate_to_pose_1hz.xml"


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
        self.assertEqual(follow_path["temperature"], 0.30)
        self.assertEqual(
            follow_path["PathFollowCritic"]["offset_from_furthest"], 10
        )
        self.assertFalse(follow_path["visualize"])

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

        bt_params = self.config["bt_navigator"]["ros__parameters"]
        self.assertEqual(
            bt_params["default_bt_xml_filename"], BT_XML.name
        )

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
        planner_params = self.config["planner_server"]["ros__parameters"]

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


if __name__ == "__main__":
    unittest.main()
