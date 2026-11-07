import json
import tempfile
import unittest
from pathlib import Path

from rov_control import ActionValidationError
from rov_trajectory import load_segments, parse_segments, validate_plan


class TrajectoryParsingTests(unittest.TestCase):
    def test_sequence_maps_actions_and_counts_transition_time(self):
        document = {
            "segments": [
                {"action": "forward", "level": 100, "duration": 1.0},
                {"action": "down", "level": 50, "duration": 0.5},
            ]
        }
        segments = parse_segments(document)
        self.assertEqual(segments[0].axes, (100, 0, 500, 0))
        self.assertEqual(segments[1].axes, (0, 0, 450, 0))
        total = validate_plan(segments, 0.2, 10.0, 250, 125)
        self.assertAlmostEqual(total, 1.7)

    def test_circle_is_composite_forward_and_yaw_with_warning(self):
        segments = parse_segments(
            {
                "segments": [
                    {
                        "type": "circle",
                        "radius_m": 2.0,
                        "duration": 10.0,
                        "forward_level": 150,
                        "yaw_level": 75,
                        "direction": "right",
                    }
                ]
            }
        )
        self.assertEqual(segments[0].axes, (150, 0, 500, 75))
        self.assertEqual(segments[0].kind, "circle_open_loop")
        self.assertIn("不保证", segments[0].metadata["warning"])

    def test_rejects_out_of_range_vertical_and_nonfinite_values(self):
        with self.assertRaises(ActionValidationError):
            parse_segments({"segments": [{"action": "down", "level": 501, "duration": 1}]})
        with self.assertRaises(ActionValidationError):
            parse_segments({"segments": [{"action": "forward", "level": "nan", "duration": 1}]})

    def test_loads_json_file(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trajectory.json"
            path.write_text(json.dumps({"segments": [{"type": "neutral", "duration": 0.2}]}), encoding="utf-8")
            document, segments = load_segments(path)
            self.assertEqual(document["segments"][0]["type"], "neutral")
            self.assertEqual(segments[0].axes, (0, 0, 500, 0))


if __name__ == "__main__":
    unittest.main()
