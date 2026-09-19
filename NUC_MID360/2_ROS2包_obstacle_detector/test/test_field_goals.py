"""Tests for the goal-only configuration loader used by real Nav2."""

import os
import unittest

from obstacle_detector.field_goals import load_field_goals


CONFIG = os.path.join(
    os.path.dirname(os.path.dirname(__file__)), "config", "field_map.yaml"
)


class FieldGoalsTest(unittest.TestCase):
    def test_required_medical_goals_have_protocol_ids(self):
        by_name, by_id = load_field_goals(CONFIG)

        self.assertEqual(
            {name: goal.goal_id for name, goal in by_name.items()},
            {"home": 1, "nurse": 2, "bed1": 3, "bed3": 4},
        )
        self.assertEqual(set(by_id), {1, 2, 3, 4})


if __name__ == "__main__":
    unittest.main()
