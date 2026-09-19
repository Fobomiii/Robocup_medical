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

    def test_plugin_source_and_manifest_are_present(self) -> None:
        manifest = ET.parse(PLANNER_PACKAGE / "package.xml").getroot()
        self.assertEqual(manifest.findtext("name"), "medical_clearance_planner")
        self.assertTrue(
            (PLANNER_PACKAGE / "src" / "clearance_planner.cpp").is_file()
        )
        self.assertTrue((PLANNER_PACKAGE / "CMakeLists.txt").is_file())


if __name__ == "__main__":
    unittest.main()
