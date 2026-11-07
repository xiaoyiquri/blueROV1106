import unittest

from dvl_mavlink_bridge import BridgeError, DvlSample, dvl_attitude_delta_rad, mat_vec, parse_dead_reckoning, parse_velocity_validity


class DvlBridgeTests(unittest.TestCase):
    def test_identity_transform_and_valid_sample(self):
        sample = parse_dead_reckoning(
            {
                "x": 1,
                "y": 2,
                "z": 3,
                "roll": 0.1,
                "pitch": 0.2,
                "yaw": 0.3,
                "std": 0.1,
                "ts": 10,
                "status": 0,
                "format": "json",
                "velocity_valid": True,
                "transducers": [{"beam_valid": True}],
            }
        )
        self.assertEqual(mat_vec((1, 0, 0, 0, 1, 0, 0, 0, 1), sample.position), (1.0, 2.0, 3.0))
        self.assertTrue(sample.velocity_valid)

    def test_missing_dvl_fields_rejected(self):
        with self.assertRaises(BridgeError):
            parse_dead_reckoning({"x": 0})

    def test_velocity_validity_is_carried_to_dead_reckoning(self):
        valid, beams = parse_velocity_validity(
            {"time": 1, "fom": 0.1, "velocity_valid": True, "status": 0, "transducers": [{"beam_valid": True}, {"beam_valid": True}]}
        )
        self.assertTrue(valid)
        self.assertTrue(beams)

    def test_dvl_angles_are_converted_from_degrees_and_wrap(self):
        previous = DvlSample((0, 0, 0), (0, 0, 359), False, False, None, 0)
        current = DvlSample((0, 0, 0), (0, 0, 1), False, False, None, 0)
        self.assertAlmostEqual(dvl_attitude_delta_rad(current, previous)[2], 2 * 3.141592653589793 / 180, places=7)


if __name__ == "__main__":
    unittest.main()
