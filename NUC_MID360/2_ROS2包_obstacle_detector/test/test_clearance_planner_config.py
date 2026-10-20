"""Static checks for the active Smac 2D global planner configuration."""

from pathlib import Path
import unittest

import yaml


PACKAGE = Path(__file__).resolve().parents[1]


class SmacPlannerConfigTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.config = yaml.safe_load(
            (PACKAGE / "config" / "nav2_params.yaml").read_text(
                encoding="utf-8"
            )
        )
        cls.planner = cls.config["planner_server"]["ros__parameters"][
            "GridBased"
        ]

    def test_smac_2d_is_the_active_plugin(self) -> None:
        self.assertEqual(
            self.planner["plugin"], "nav2_smac_planner/SmacPlanner2D"
        )
        package_xml = (PACKAGE / "package.xml").read_text(encoding="utf-8")
        self.assertIn("<exec_depend>nav2_smac_planner</exec_depend>", package_xml)

    def test_smac_parameters_are_explicit_and_fixed_heading_is_preserved(self) -> None:
        expected = {
            "tolerance": 0.10,
            "allow_unknown": False,
            "downsample_costmap": False,
            "downsampling_factor": 1,
            "max_iterations": 1000000,
            "max_on_approach_iterations": 1000,
            "max_planning_time": 1.5,
            "cost_travel_multiplier": 5.0,
            "use_final_approach_orientation": False,
        }
        for name, value in expected.items():
            self.assertEqual(self.planner[name], value, name)

    def test_global_field_prefers_clearance_and_local_field_stays_agile(self) -> None:
        global_params = self.config["global_costmap"]["global_costmap"][
            "ros__parameters"
        ]
        local_params = self.config["local_costmap"]["local_costmap"][
            "ros__parameters"
        ]
        global_inflation = global_params["inflation_layer"]
        local_inflation = local_params["inflation_layer"]
        self.assertEqual(global_inflation["inflation_radius"], 0.60)
        self.assertEqual(global_inflation["cost_scaling_factor"], 6.0)
        self.assertEqual(local_inflation["inflation_radius"], 0.60)
        self.assertEqual(local_inflation["cost_scaling_factor"], 10.0)

    def test_custom_clearance_and_blind_zone_parameters_are_not_passed_to_smac(
        self,
    ) -> None:
        stale_parameters = {
            "forward_speed",
            "reverse_speed",
            "lateral_speed",
            "preferred_clearance",
            "clearance_weight",
            "density_weight",
            "corner_rounding_enabled",
            "route_switch_time_improvement",
            "blind_zone_enabled",
            "blind_zone_angles_deg",
            "blind_zone_lateral_offset",
        }
        self.assertTrue(stale_parameters.isdisjoint(self.planner))

    def test_behavior_trees_continue_to_request_gridbased(self) -> None:
        for tree_name in (
            "navigate_to_pose_1hz.xml",
            "navigate_to_pose_nurse_fast.xml",
        ):
            tree = (PACKAGE / "config" / tree_name).read_text(encoding="utf-8")
            self.assertIn('planner_id="GridBased"', tree)


if __name__ == "__main__":
    unittest.main()
