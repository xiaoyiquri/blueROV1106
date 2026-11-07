import unittest
import json
import tempfile
from pathlib import Path

from rov_control import ControlError
from rov_host_goto import body_error, circular_mean, compute_manual_axes, dvl_yaw_rad, load_config, load_origin_state, save_origin_state


class HostGotoTests(unittest.TestCase):
    def test_body_error_rotates_fixed_ned_into_body_axes(self):
        forward, right, forward_v, right_v = body_error((1.0, 0.0, 0.0), (0.2, 0.1, 0.0), 3.141592653589793 / 2)
        self.assertAlmostEqual(forward, 0.0, places=7)
        self.assertAlmostEqual(right, -1.0, places=7)
        self.assertAlmostEqual(forward_v, 0.1, places=7)
        self.assertAlmostEqual(right_v, -0.2, places=7)

    def test_dvl_yaw_is_degrees_to_radians(self):
        self.assertAlmostEqual(dvl_yaw_rad(90.0, 0.0, 1.0), 3.141592653589793 / 2, places=7)

    def test_circular_mean_handles_heading_wrap(self):
        self.assertAlmostEqual(abs(circular_mean((3.13, -3.13))), 3.141592653589793, places=2)

    def test_positive_ned_z_commands_manual_down(self):
        config = {
            "horizontal_kp_level_per_m": 120.0,
            "horizontal_kd_level_per_m_s": 80.0,
            "vertical_kp_level_per_m": 100.0,
            "vertical_kd_level_per_m_s": 80.0,
            "yaw_kp_level_per_rad": 100.0,
            "max_horizontal_level": 250,
            "max_vertical_offset": 125,
        }
        self.assertEqual(compute_manual_axes((0.0, 0.0, 1.0), (0.0, 0.0, 0.0), 0.0, 0.0, config)[2], 400)

    def test_task_axes_use_relative_reset_heading(self):
        config = {
            "horizontal_kp_level_per_m": 100.0,
            "horizontal_kd_level_per_m_s": 0.0,
            "vertical_kp_level_per_m": 100.0,
            "vertical_kd_level_per_m_s": 0.0,
            "yaw_kp_level_per_rad": 100.0,
            "max_horizontal_level": 400,
            "max_vertical_offset": 200,
        }
        # 当前 yaw 与复位 yaw 相同，固定任务 +X 必须仍然是机体前进。
        axes = compute_manual_axes(
            (1.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            1.0,
            1.0,
            config,
        )
        self.assertEqual(axes, (100, 0, 500, 0))

    def test_unverified_transform_is_refused(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.json"
            path.write_text(
                json.dumps(
                    {
                        "dvl_to_task_ned_rotation": [1, 0, 0, 0, 1, 0, 0, 0, 1],
                        "frame_transform_verified": False,
                    }
                ),
                encoding="utf-8",
            )
            with self.assertRaises(ControlError):
                load_config(path)

    def test_origin_state_persists_and_validates_frame(self):
        config = load_config(Path("host_goto_config.json"))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "origin.json"
            save_origin_state(path, config, (1.0, 2.0, 3.0), 0.25)
            state = load_origin_state(path, config)
            self.assertEqual(state["origin"], (1.0, 2.0, 3.0))
            self.assertAlmostEqual(state["yaw_target_rad"], 0.25)


if __name__ == "__main__":
    unittest.main()
