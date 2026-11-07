import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import rov_control as rc


class FakeMessage:
    def __init__(self, message_type, system=1, component=1, **fields):
        self._type = message_type
        self._system = system
        self._component = component
        for key, value in fields.items():
            setattr(self, key, value)

    def get_type(self):
        return self._type

    def get_srcSystem(self):
        return self._system

    def get_srcComponent(self):
        return self._component

    def to_dict(self):
        return {key: value for key, value in vars(self).items() if not key.startswith("_")}


class FakeMav:
    def __init__(self):
        self.manual = []
        self.heartbeats = []

    def manual_control_send(self, *values):
        self.manual.append(values)

    def heartbeat_send(self, *values):
        self.heartbeats.append(values)


class StateMachineMav(FakeMav):
    def __init__(self, queue):
        super().__init__()
        self.queue = queue

    def set_mode_send(self, target_system, base_mode, custom_mode):
        self.queue.append(
            FakeMessage(
                "HEARTBEAT",
                base_mode=base_mode,
                custom_mode=19,
                autopilot=rc.mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                type=rc.mavutil.mavlink.MAV_TYPE_SUBMARINE,
            )
        )
        self.queue.append(FakeMessage("COMMAND_ACK", command=rc.mavutil.mavlink.MAV_CMD_DO_SET_MODE, result=0))

    def command_long_send(self, target_system, target_component, command, *params):
        if command == rc.mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            armed = bool(params[1])
            self.queue.append(
                FakeMessage(
                    "HEARTBEAT",
                    base_mode=rc.mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                    | (rc.mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0),
                    custom_mode=19,
                    autopilot=rc.mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                    type=rc.mavutil.mavlink.MAV_TYPE_SUBMARINE,
                )
            )
            self.queue.append(FakeMessage("COMMAND_ACK", command=command, result=0))


class StateMachineMaster:
    def __init__(self):
        self.queue = [
            FakeMessage(
                "HEARTBEAT",
                base_mode=rc.mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                custom_mode=19,
                autopilot=rc.mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                type=rc.mavutil.mavlink.MAV_TYPE_SUBMARINE,
            )
        ]
        self.mav = StateMachineMav(self.queue)

    def recv_match(self, blocking=False):
        return self.queue.pop(0) if self.queue else None

    def mode_mapping(self):
        return {"MANUAL": 19}


class MappingAndValidationTests(unittest.TestCase):
    def test_all_actions_keep_other_axes_neutral(self):
        self.assertEqual(rc.map_action("forward", 100), (100, 0, 500, 0))
        self.assertEqual(rc.map_action("backward", 100), (-100, 0, 500, 0))
        self.assertEqual(rc.map_action("left", 100), (0, -100, 500, 0))
        self.assertEqual(rc.map_action("right", 100), (0, 100, 500, 0))
        self.assertEqual(rc.map_action("yaw-left", 100), (0, 0, 500, -100))
        self.assertEqual(rc.map_action("yaw-right", 100), (0, 0, 500, 100))
        self.assertEqual(rc.map_action("up", 100), (0, 0, 600, 0))
        self.assertEqual(rc.map_action("down", 100), (0, 0, 400, 0))

    def test_rejects_protocol_and_action_errors(self):
        with self.assertRaises(rc.ActionValidationError):
            rc.map_action("forward", -1)
        with self.assertRaises(rc.ActionValidationError):
            rc.map_action("forward", 1001)
        with self.assertRaises(rc.ActionValidationError):
            rc.map_action("up", 501)
        with self.assertRaises(rc.ActionValidationError):
            rc.map_action("not-an-action", 1)

    def test_parameter_validation_happens_before_connection(self):
        args = SimpleNamespace(
            action="forward",
            level=251,
            duration=1.0,
            max_duration=5.0,
            max_horizontal_level=250,
            max_vertical_level=125,
        )
        with self.assertRaises(rc.ActionValidationError):
            rc.validate_move_parameters(args)
        args.level = 1
        args.duration = float("nan")
        with self.assertRaises(rc.ActionValidationError):
            rc.validate_move_parameters(args)


class MessageAndControlTests(unittest.TestCase):
    def setUp(self):
        self.tempdir = tempfile.TemporaryDirectory()
        self.logger = rc.SessionLogger(Path(self.tempdir.name))
        self.controller = rc.Controller(
            connection="fake",
            baud=115200,
            source_system=255,
            source_component=190,
            target_system=1,
            target_component=1,
            logger=self.logger,
        )
        self.controller.vehicle.target_system = 1
        self.controller.vehicle.target_component = 1
        self.controller.master = SimpleNamespace(mav=FakeMav())

    def tearDown(self):
        self.logger.close()
        self.tempdir.cleanup()

    def test_heartbeat_and_attitude_units_are_retained(self):
        heartbeat = FakeMessage(
            "HEARTBEAT",
            autopilot=rc.mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            type=rc.mavutil.mavlink.MAV_TYPE_SUBMARINE,
            base_mode=rc.mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            custom_mode=19,
        )
        self.controller.process_message(heartbeat)
        self.assertEqual(self.controller.vehicle.mode, "MANUAL")
        self.assertFalse(self.controller.vehicle.armed)
        attitude = FakeMessage(
            "ATTITUDE",
            roll=0.1,
            pitch=-0.2,
            yaw=0.3,
            rollspeed=0.01,
            pitchspeed=0.02,
            yawspeed=0.03,
        )
        self.controller.process_message(attitude)
        self.assertEqual(self.controller.vehicle.attitude["roll_rad"], 0.1)
        self.assertEqual(self.controller.vehicle.attitude["yawspeed_rad_s"], 0.03)

    def test_source_filter_does_not_update_vehicle_from_other_system(self):
        other = FakeMessage(
            "HEARTBEAT",
            system=42,
            component=1,
            autopilot=rc.mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            type=rc.mavutil.mavlink.MAV_TYPE_SUBMARINE,
            base_mode=rc.mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
            custom_mode=0,
        )
        self.controller.process_message(other)
        self.assertIsNone(self.controller.vehicle.last_heartbeat_mono)
        self.assertFalse(self.controller.vehicle.armed is True)

    def test_manual_control_always_sends_buttons_zero(self):
        self.controller.send_manual((12, -34, 500, 56))
        self.assertEqual(self.controller.master.mav.manual, [(1, 12, -34, 500, 56, 0)])

    def test_mode_arm_and_disarm_state_flow(self):
        master = StateMachineMaster()
        self.controller.master = master
        self.controller.set_state(rc.State.CONNECTED)
        self.controller.vehicle.target_system = 1
        self.controller.vehicle.target_component = 1
        self.controller.process_message(master.queue.pop(0))
        self.controller.request_mode()
        self.controller.set_state(rc.State.NEUTRAL)
        self.controller.send_manual(rc.NEUTRAL)
        with mock.patch.object(self.controller, "send_manual", side_effect=AssertionError("GUIDED-like arm must not send MANUAL_CONTROL")):
            self.controller.request_arm(send_neutral=False)
        self.assertTrue(self.controller.arm_confirmed)
        with mock.patch.object(rc, "DEFAULT_STOP_DURATION", 0.01), mock.patch.object(
            self.controller, "send_manual", side_effect=AssertionError("GUIDED-like cleanup must not send MANUAL_CONTROL")
        ):
            self.assertTrue(self.controller.stop_and_disarm(send_neutral=False))
        self.assertEqual(self.controller.state, rc.State.DONE)
        self.assertTrue(self.controller.disarm_confirmed)


if __name__ == "__main__":
    unittest.main()
