"""Regression checks for the Nav2 static map raster."""

import math
import os
import unittest

import yaml


CONFIG_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "config")


class StaticMapTest(unittest.TestCase):
    def test_map_contains_fixtures_and_keeps_goals_free(self):
        with open(os.path.join(CONFIG_DIR, "static_map.yaml"), encoding="utf-8") as stream:
            metadata = yaml.safe_load(stream)
        with open(os.path.join(CONFIG_DIR, "field_map.yaml"), encoding="utf-8") as stream:
            field = yaml.safe_load(stream)
        with open(os.path.join(CONFIG_DIR, metadata["image"]), "rb") as stream:
            magic, comment, dimensions, maximum, pixels = stream.read().split(b"\n", 4)

        self.assertEqual(magic, b"P5")
        self.assertTrue(comment.startswith(b"#"))
        width, height = (int(value) for value in dimensions.split())
        self.assertEqual(maximum, b"255")
        self.assertEqual(len(pixels), width * height)
        self.assertGreater(pixels.count(0), 0)

        resolution = float(metadata["resolution"])
        origin_x, origin_y = (float(value) for value in metadata["origin"][:2])

        # Every fixed fixture, including the three bedside cabinets, must be
        # represented by an occupied cell at its surveyed centre.
        for fixture in field["fixtures"]:
            ros_x = float(fixture["y_mm"]) / 1000.0
            ros_y = -float(fixture["x_mm"]) / 1000.0
            col = int(round((ros_x - origin_x) / resolution))
            row = height - 1 - int(round((ros_y - origin_y) / resolution))
            self.assertTrue(0 <= col < width and 0 <= row < height, fixture["name"])
            self.assertEqual(pixels[row * width + col], 0, fixture["name"])

        for name, goal in field["goals"].items():
            ros_x = float(goal["y_mm"]) / 1000.0
            ros_y = -float(goal["x_mm"]) / 1000.0
            col = int(round((ros_x - origin_x) / resolution))
            row = height - 1 - int(round((ros_y - origin_y) / resolution))
            self.assertTrue(0 <= col < width and 0 <= row < height, name)
            self.assertNotEqual(pixels[row * width + col], 0, name)

        fixtures = {fixture["name"]: fixture for fixture in field["fixtures"]}
        for bed_name in ("bed1", "bed2", "bed3"):
            bed = fixtures[bed_name]
            lower_edge = float(bed["y_mm"]) - float(bed["height_mm"]) / 2.0
            self.assertAlmostEqual(lower_edge, 4450.0, delta=25.0, msg=bed_name)

        parking_checks = (
            ("bed1", "bedside_cabinet1", "bed1", "right", "left", 3500.0),
            ("bed3", "bedside_cabinet3", "bed3", "left", "right", 3500.0),
        )
        for (
            goal_name,
            cabinet_name,
            bed_name,
            bed_side_name,
            field_side_name,
            field_side_x,
        ) in parking_checks:
            goal = field["goals"][goal_name]
            cabinet = fixtures[cabinet_name]
            bed = fixtures[bed_name]
            goal_x = float(goal["x_mm"])
            goal_y = float(goal["y_mm"])
            cabinet_x_min = float(cabinet["x_mm"]) - float(cabinet["width_mm"]) / 2.0
            cabinet_x_max = float(cabinet["x_mm"]) + float(cabinet["width_mm"]) / 2.0
            cabinet_y_min = float(cabinet["y_mm"]) - float(cabinet["height_mm"]) / 2.0
            cabinet_y_max = float(cabinet["y_mm"]) + float(cabinet["height_mm"]) / 2.0
            cabinet_dx = max(cabinet_x_min - goal_x, 0.0, goal_x - cabinet_x_max)
            cabinet_dy = max(cabinet_y_min - goal_y, 0.0, goal_y - cabinet_y_max)
            cabinet_clearance = math.hypot(cabinet_dx, cabinet_dy)
            bed_side = (
                float(bed["x_mm"]) + float(bed["width_mm"]) / 2.0
                if bed_side_name == "right"
                else float(bed["x_mm"]) - float(bed["width_mm"]) / 2.0
            )
            field_side_x = -field_side_x if field_side_name == "left" else field_side_x
            self.assertAlmostEqual(cabinet_clearance, 500.0, delta=1.0, msg=goal_name)
            self.assertAlmostEqual(abs(goal_x - bed_side), 300.0, delta=1.0, msg=goal_name)
            self.assertAlmostEqual(abs(goal_x - field_side_x), 1300.0, delta=1.0, msg=goal_name)

        # The 600 mm cabinets share an edge with their associated beds.
        touching = (
            ("bed1", "bedside_cabinet1", "right", "left"),
            ("bed2", "bedside_cabinet2", "right", "left"),
            ("bed3", "bedside_cabinet3", "left", "right"),
        )
        for bed_name, cabinet_name, bed_edge, cabinet_edge in touching:
            bed = fixtures[bed_name]
            cabinet = fixtures[cabinet_name]
            bed_x = float(bed["x_mm"]) + (
                float(bed["width_mm"]) / 2.0 if bed_edge == "right" else -float(bed["width_mm"]) / 2.0
            )
            cabinet_x = float(cabinet["x_mm"]) + (
                float(cabinet["width_mm"]) / 2.0 if cabinet_edge == "right" else -float(cabinet["width_mm"]) / 2.0
            )
            self.assertAlmostEqual(bed_x, cabinet_x, delta=1.0, msg=cabinet_name)


if __name__ == "__main__":
    unittest.main()
