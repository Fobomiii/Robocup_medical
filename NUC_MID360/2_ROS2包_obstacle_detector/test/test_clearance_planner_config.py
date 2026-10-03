"""Integration checks for the custom Nav2 clearance planner package."""

import ast
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
        self.assertAlmostEqual(planner["preferred_clearance"], 0.65)
        self.assertAlmostEqual(planner["clearance_weight"], 12.0)
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
        controller_params = config["controller_server"]["ros__parameters"]
        for controller_name in controller_params["controller_plugins"]:
            controller = controller_params[controller_name]
            self.assertEqual(planner["forward_speed"], controller["vx_max"])
            self.assertEqual(
                planner["reverse_speed"], abs(controller["vx_min"])
            )
            self.assertEqual(planner["lateral_speed"], controller["vy_max"])
        self.assertAlmostEqual(planner["max_planar_speed"], 3.00)
        self.assertAlmostEqual(planner["max_wheel_speed"], 3.00)
        self.assertAlmostEqual(planner["simplification_time_tolerance"], 1.01)
        self.assertGreaterEqual(planner["max_time_ratio"], 1.0)
        self.assertLessEqual(planner["max_time_ratio"], 1.25)
        self.assertAlmostEqual(planner["min_time_slack"], 0.25)
        self.assertAlmostEqual(planner["route_switch_risk_improvement"], 0.55)
        self.assertAlmostEqual(planner["route_switch_time_improvement"], 0.50)
        self.assertAlmostEqual(planner["route_switch_max_slowdown"], 0.00)
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

    def test_x_drive_blind_zone_path_shaping_is_enabled(self) -> None:
        with (OBSTACLE_PACKAGE / "config" / "nav2_params.yaml").open(
            "r", encoding="utf-8"
        ) as stream:
            config = yaml.safe_load(stream)
        planner = config["planner_server"]["ros__parameters"]["GridBased"]
        self.assertTrue(planner["blind_zone_enabled"])
        self.assertEqual(
            planner["blind_zone_angles_deg"],
            [45.0, 135.0, -135.0, -45.0],
        )
        self.assertGreater(planner["blind_zone_half_width_deg"], 0.0)
        self.assertGreater(planner["blind_zone_min_overlap_m"], 0.0)
        self.assertGreater(planner["blind_zone_lookahead_m"], 0.0)
        self.assertGreater(planner["blind_zone_heading_tolerance_deg"], 0.0)
        self.assertAlmostEqual(planner["blind_zone_min_overlap_m"], 0.60)
        self.assertAlmostEqual(planner["blind_zone_lookahead_m"], 2.0)
        self.assertAlmostEqual(planner["blind_zone_side_switch_improvement"], 0.25)
        self.assertGreater(planner["blind_zone_min_lateral_offset"], 0.0)
        self.assertGreater(planner["blind_zone_lateral_offset"], 0.0)
        self.assertLessEqual(
            planner["blind_zone_min_lateral_offset"],
            planner["blind_zone_lateral_offset"],
        )
        self.assertGreaterEqual(
            planner["blind_zone_max_detour_time_ratio"], 1.0
        )

        source = (
            PLANNER_PACKAGE / "src" / "clearance_planner.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn("addBlindZoneDoglegs", source)
        self.assertIn("longestBlindZoneOverlap", source)
        self.assertIn("blind_zone_side_switch_improvement", source)
        self.assertIn("worldSegmentCost", source)
        self.assertIn("blind-zone doglegs", source)
        self.assertIn("density_weight", source)
        self.assertIn("fastestTimeField", source)
        self.assertIn("constrainedSafePath", source)
        self.assertIn("time_budget", source)
        self.assertIn(
            "std::max({x_time, y_time, planar_time, wheel_time})", source
        )
        self.assertIn("params.max_planar_speed", source)
        self.assertIn("/ kSqrtTwo", source)
        self.assertIn("max_wheel_speed", source)
        self.assertIn("polylineTime", source)
        self.assertIn("simplification_time_tolerance", source)
        self.assertIn("route_switch_risk_improvement", source)
        self.assertIn("route_switch_time_improvement", source)
        self.assertIn("route_switch_max_slowdown", source)
        self.assertIn("has_previous_route_", source)
        self.assertIn("densifyPath", source)
        self.assertIn("retained final route", source)
        for parameter in (
            "forward_speed",
            "reverse_speed",
            "lateral_speed",
            "max_planar_speed",
            "max_wheel_speed",
        ):
            self.assertIn(f'declare("{parameter}", 3.00)', source)
        self.assertIn("const bool clearly_faster", source)
        self.assertIn("const bool clearly_safer_without_slowing", source)
        self.assertIn(
            "time_improvement >= -params.route_switch_max_slowdown", source
        )
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

    def test_execution_path_uses_native_nav2_contract(self) -> None:
        xml = (OBSTACLE_PACKAGE / "config" / "navigate_to_pose_1hz.xml").read_text(encoding="utf-8")
        self.assertIn('ComputePathToPose goal="{goal}" path="{path}"', xml)
        self.assertIn('FollowPath path="{path}"', xml)
        self.assertNotIn("SelectExecutionPath", xml)
        launch = (OBSTACLE_PACKAGE / "launch" / "obstacle.launch.py").read_text(encoding="utf-8")
        self.assertIn('"execution_path_topic": "/plan"', launch)


if __name__ == "__main__":
    unittest.main()
