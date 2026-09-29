"""Integration checks for the custom Nav2 clearance planner package."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


OBSTACLE_PACKAGE = Path(__file__).resolve().parents[1]
PLANNER_PACKAGE = OBSTACLE_PACKAGE.parent / OBSTACLE_PACKAGE.name.replace(
    "obstacle_detector", "clearance_planner"
)


class ClearancePlannerConfigTest(unittest.TestCase):
    def test_plugin_export_matches_nav2_configuration(self) -> None:
        config = yaml.safe_load(
            (OBSTACLE_PACKAGE / "config" / "nav2_params.yaml").read_text(
                encoding="utf-8"
            )
        )
        planner = config["planner_server"]["ros__parameters"]["GridBased"]
        plugin_xml = ET.parse(
            PLANNER_PACKAGE / "clearance_planner_plugin.xml"
        ).getroot()
        exported = plugin_xml.find("class")
        self.assertIsNotNone(exported)
        self.assertEqual(planner["plugin"], exported.attrib["name"])
        self.assertEqual(
            planner["plugin"], "medical_clearance_planner/ClearancePlanner"
        )

    def test_private_preference_does_not_expand_visible_inflation(self) -> None:
        config = yaml.safe_load(
            (OBSTACLE_PACKAGE / "config" / "nav2_params.yaml").read_text(
                encoding="utf-8"
            )
        )
        global_params = config["global_costmap"]["global_costmap"]["ros__parameters"]
        local_params = config["local_costmap"]["local_costmap"]["ros__parameters"]
        planner = config["planner_server"]["ros__parameters"]["GridBased"]
        self.assertEqual(global_params["inflation_layer"]["inflation_radius"], 0.25)
        self.assertEqual(local_params["inflation_layer"]["inflation_radius"], 0.25)
        self.assertGreater(planner["preferred_clearance"], 0.25)
        self.assertGreater(planner["clearance_weight"], 0.0)
        self.assertGreater(planner["density_weight"], 0.0)
        self.assertGreater(planner["goal_exemption_radius"], 0.0)
        self.assertGreaterEqual(planner["costmap_weight"], 0.0)

    def test_time_budget_uses_real_axis_speed_limits(self) -> None:
        config = yaml.safe_load(
            (OBSTACLE_PACKAGE / "config" / "nav2_params.yaml").read_text(
                encoding="utf-8"
            )
        )
        planner = config["planner_server"]["ros__parameters"]["GridBased"]
        controller = config["controller_server"]["ros__parameters"]["FollowPath"]
        self.assertEqual(planner["forward_speed"], controller["vx_max"])
        self.assertEqual(planner["reverse_speed"], abs(controller["vx_min"]))
        self.assertEqual(planner["lateral_speed"], controller["vy_max"])
        self.assertAlmostEqual(planner["max_wheel_speed"], 2.00)
        self.assertAlmostEqual(planner["simplification_time_tolerance"], 1.01)
        self.assertGreaterEqual(planner["max_time_ratio"], 1.0)
        self.assertLessEqual(planner["max_time_ratio"], 1.25)
        self.assertGreaterEqual(planner["min_time_slack"], 0.0)
        self.assertLessEqual(planner["min_time_slack"], 1.0)
        self.assertAlmostEqual(planner["route_switch_risk_improvement"], 0.05)
        self.assertAlmostEqual(planner["route_switch_time_improvement"], 1.0)
        self.assertAlmostEqual(planner["route_reuse_max_distance"], 0.50)

    def test_bounded_time_clearance_model_is_active(self) -> None:
        config = yaml.safe_load(
            (OBSTACLE_PACKAGE / "config" / "nav2_params.yaml").read_text(
                encoding="utf-8"
            )
        )
        planner = config["planner_server"]["ros__parameters"]["GridBased"]
        for parameter in (
            "preferred_clearance",
            "clearance_weight",
            "clearance_power",
            "density_radius",
            "density_weight",
            "density_normalization",
            "goal_exemption_radius",
            "start_exemption_radius",
        ):
            self.assertIn(parameter, planner)
        source = (
            PLANNER_PACKAGE / "src" / "clearance_planner.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn("traversalRisks", source)
        self.assertIn("preferred_clearance", source)
        self.assertIn("density_weight", source)
        self.assertIn("fastestTimeField", source)
        self.assertIn("constrainedSafePath", source)
        self.assertIn("time_budget", source)
        self.assertIn("std::max({x_time, y_time, wheel_time})", source)
        self.assertIn("max_wheel_speed", source)
        self.assertIn("polylineTime", source)
        self.assertIn("simplification_time_tolerance", source)
        self.assertIn("route_switch_risk_improvement", source)
        self.assertIn("route_switch_time_improvement", source)
        self.assertIn("has_previous_route_", source)
        self.assertIn("risk_improvement >= params.route_switch_risk_improvement ||", source)
        self.assertIn("time_improvement >= params.route_switch_time_improvement", source)
        self.assertNotIn("1.0 + costmap_penalty", source)
        self.assertNotIn("traversalTimes", source)

    def test_plugin_source_and_manifest_are_present(self) -> None:
        manifest = ET.parse(PLANNER_PACKAGE / "package.xml").getroot()
        self.assertEqual(manifest.findtext("name"), "medical_clearance_planner")
        self.assertTrue(
            (PLANNER_PACKAGE / "src" / "clearance_planner.cpp").is_file()
        )
        self.assertTrue((PLANNER_PACKAGE / "CMakeLists.txt").is_file())

    def test_all_path_poses_keep_requested_goal_orientation(self) -> None:
        source = (
            PLANNER_PACKAGE / "src" / "clearance_planner.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn(
            "pose.pose.orientation = goal.pose.orientation;",
            source,
        )
        self.assertNotIn("yawQuaternion", source)


if __name__ == "__main__":
    unittest.main()
