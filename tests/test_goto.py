import time
import unittest
from types import SimpleNamespace

from rov_control import VehicleState
from rov_goto import (
    EKF_ATTITUDE,
    EKF_POS_HORIZ_REL,
    EKF_POS_VERT_AGL,
    EKF_VELOCITY_HORIZ,
    EKF_VELOCITY_VERT,
    current_position,
    estimator_status,
)


class GotoValidationTests(unittest.TestCase):
    def setUp(self):
        self.config = {"position_timeout_s": 1.0, "ekf_timeout_s": 1.0}
        self.controller = SimpleNamespace(vehicle=VehicleState())

    def test_zero_position_is_valid_when_fresh(self):
        now = time.monotonic()
        self.controller.vehicle.local_position = {
            "x_m": 0.0,
            "y_m": 0.0,
            "z_m": 0.0,
            "vx_m_s": 0.0,
            "vy_m_s": 0.0,
            "vz_m_s": 0.0,
        }
        self.controller.vehicle.last_local_position_mono = now
        ok, _, position = current_position(self.controller, self.config)
        self.assertTrue(ok)
        self.assertEqual(position["x_m"], 0.0)

    def test_stale_position_is_invalid(self):
        self.controller.vehicle.local_position = {field: 0.0 for field in ("x_m", "y_m", "z_m", "vx_m_s", "vy_m_s", "vz_m_s")}
        self.controller.vehicle.last_local_position_mono = time.monotonic() - 2.0
        ok, reason, _ = current_position(self.controller, self.config)
        self.assertFalse(ok)
        self.assertIn("过期", reason)

    def test_ekf_requires_horizontal_and_vertical_position_and_velocity(self):
        self.controller.vehicle.ekf_status = {
            "flags": EKF_ATTITUDE | EKF_VELOCITY_HORIZ | EKF_VELOCITY_VERT | EKF_POS_HORIZ_REL | EKF_POS_VERT_AGL
        }
        self.controller.vehicle.last_ekf_status_mono = time.monotonic()
        ok, _, flags = estimator_status(self.controller, self.config)
        self.assertTrue(ok)
        self.assertEqual(flags, self.controller.vehicle.ekf_status["flags"])

    def test_const_position_ekf_is_rejected(self):
        self.controller.vehicle.ekf_status = {
            "flags": EKF_ATTITUDE | EKF_VELOCITY_HORIZ | EKF_VELOCITY_VERT | EKF_POS_HORIZ_REL | EKF_POS_VERT_AGL | (1 << 7)
        }
        self.controller.vehicle.last_ekf_status_mono = time.monotonic()
        ok, reason, _ = estimator_status(self.controller, self.config)
        self.assertFalse(ok)
        self.assertIn("常位置", reason)


if __name__ == "__main__":
    unittest.main()
