from types import SimpleNamespace
import unittest

from robot import config
from robot.controller import ControllerInput


class FakeEcodes:
    EV_KEY = 1
    EV_ABS = 3
    bytype = {
        EV_KEY: {},
        EV_ABS: {},
    }


class FakeDevice:
    def __init__(self, events):
        self.events = events

    def read(self):
        events = self.events
        self.events = []
        return events


def controller_with_events(events):
    controller = ControllerInput.__new__(ControllerInput)
    controller.device = FakeDevice(events)
    controller.ecodes = FakeEcodes
    controller.error = ""
    controller.last_event = "none"
    controller.axis_values = {}
    controller.axis_ranges = {
        config.CONTROLLER_LEFT_X_AXIS: (-32768, 32767),
        config.CONTROLLER_LEFT_Y_AXIS: (-32768, 32767),
        config.CONTROLLER_RIGHT_X_AXIS: (-32768, 32767),
        config.CONTROLLER_RIGHT_Y_AXIS: (-32768, 32767),
        config.CONTROLLER_LEFT_TRIGGER_AXIS: (0, 255),
        config.CONTROLLER_RIGHT_TRIGGER_AXIS: (0, 255),
    }
    controller.supported_axis_codes = []
    controller.supported_key_codes = []
    controller.disconnected = False
    controller._menu_stick_source = "right"
    controller._menu_input_sequence = 0
    controller._menu_stick_sequence = {"left": 0, "right": 0}
    controller._trigger_active = {"left": False, "right": False}
    return controller


class ControllerInputTests(unittest.TestCase):
    def test_mode_buttons_are_edge_triggered(self):
        controller = controller_with_events([
            SimpleNamespace(type=FakeEcodes.EV_KEY, code=config.CONTROLLER_A_BUTTON, value=1),
            SimpleNamespace(type=FakeEcodes.EV_KEY, code=config.CONTROLLER_B_BUTTON, value=1),
            SimpleNamespace(type=FakeEcodes.EV_KEY, code=config.CONTROLLER_Y_BUTTON, value=1),
            SimpleNamespace(type=FakeEcodes.EV_KEY, code=config.CONTROLLER_CAPTURE_BUTTON, value=1),
            SimpleNamespace(type=FakeEcodes.EV_KEY, code=config.CONTROLLER_STABILITY_BUTTON, value=1),
            SimpleNamespace(type=FakeEcodes.EV_KEY, code=config.CONTROLLER_LEFT_BUMPER_BUTTON, value=1),
            SimpleNamespace(type=FakeEcodes.EV_KEY, code=config.CONTROLLER_RIGHT_BUMPER_BUTTON, value=1),
            SimpleNamespace(type=FakeEcodes.EV_KEY, code=config.CONTROLLER_LEFT_TRIGGER_BUTTON, value=1),
            SimpleNamespace(type=FakeEcodes.EV_KEY, code=config.CONTROLLER_RIGHT_TRIGGER_BUTTON, value=1),
        ])

        update = controller.poll()

        self.assertTrue(update.a_pressed)
        self.assertTrue(update.b_pressed)
        self.assertTrue(update.y_pressed)
        self.assertTrue(update.capture_pressed)
        self.assertTrue(update.stability_pressed)
        self.assertTrue(update.left_bumper_pressed)
        self.assertTrue(update.right_bumper_pressed)
        self.assertEqual(update.throttle_limit_delta, 0)
        self.assertFalse(update.front_suspension_pressed)
        self.assertFalse(update.rear_suspension_pressed)
        self.assertFalse(update.y_released)

    def test_button_release_and_unmapped_input_do_not_change_modes(self):
        controller = controller_with_events([
            SimpleNamespace(type=FakeEcodes.EV_KEY, code=config.CONTROLLER_A_BUTTON, value=0),
            SimpleNamespace(type=FakeEcodes.EV_KEY, code=999, value=1),
        ])

        update = controller.poll()

        self.assertFalse(update.a_pressed)
        self.assertFalse(update.b_pressed)
        self.assertFalse(update.y_pressed)
        self.assertFalse(update.y_released)
        self.assertFalse(update.capture_pressed)
        self.assertFalse(update.stability_pressed)
        self.assertEqual(update.throttle_limit_delta, 0)

    def test_trigger_buttons_adjust_throttle_limit(self):
        controller = controller_with_events([
            SimpleNamespace(type=FakeEcodes.EV_KEY, code=config.CONTROLLER_LEFT_TRIGGER_BUTTON, value=1),
        ])
        self.assertEqual(controller.poll().throttle_limit_delta, -1)

        controller.device.events = [
            SimpleNamespace(type=FakeEcodes.EV_KEY, code=config.CONTROLLER_RIGHT_TRIGGER_BUTTON, value=1),
        ]
        self.assertEqual(controller.poll().throttle_limit_delta, 1)

    def test_y_release_is_reported_separately_from_y_press(self):
        controller = controller_with_events([
            SimpleNamespace(type=FakeEcodes.EV_KEY, code=config.CONTROLLER_Y_BUTTON, value=0),
        ])

        update = controller.poll()

        self.assertFalse(update.y_pressed)
        self.assertTrue(update.y_released)

    def test_dpad_axis_and_buttons_select_front_and_rear_suspension(self):
        controller = controller_with_events([
            SimpleNamespace(type=FakeEcodes.EV_ABS, code=config.CONTROLLER_DPAD_Y_AXIS, value=-1),
            SimpleNamespace(type=FakeEcodes.EV_KEY, code=config.CONTROLLER_DPAD_DOWN_BUTTON, value=1),
        ])

        update = controller.poll()

        self.assertTrue(update.front_suspension_pressed)
        self.assertTrue(update.rear_suspension_pressed)
        self.assertEqual(update.throttle_limit_delta, 0)

        controller.device.events = [
            SimpleNamespace(type=FakeEcodes.EV_ABS, code=config.CONTROLLER_DPAD_Y_AXIS, value=-1),
        ]
        update = controller.poll()
        self.assertTrue(update.front_suspension_pressed)
        self.assertFalse(update.rear_suspension_pressed)

    def test_radial_menu_uses_the_most_recently_moved_stick(self):
        controller = controller_with_events([])
        self.assertEqual(controller.right_stick(), (0.0, 0.0))

        controller.device.events = [
            SimpleNamespace(type=FakeEcodes.EV_ABS, code=config.CONTROLLER_LEFT_X_AXIS, value=32767),
        ]
        controller.poll()
        (menu_x, menu_y), source = controller.menu_stick()
        self.assertEqual(source, "left")
        self.assertGreater(menu_x, 0.9)
        self.assertAlmostEqual(menu_y, 0.0, places=3)

        controller.device.events = [
            SimpleNamespace(type=FakeEcodes.EV_ABS, code=config.CONTROLLER_RIGHT_X_AXIS, value=32767),
        ]
        controller.poll()
        (menu_x, menu_y), source = controller.menu_stick()
        self.assertEqual(source, "right")
        self.assertGreater(menu_x, 0.9)
        self.assertAlmostEqual(menu_y, 0.0, places=3)

    def test_analog_triggers_adjust_limit_only_on_press_edges(self):
        controller = controller_with_events([
            SimpleNamespace(type=FakeEcodes.EV_ABS, code=config.CONTROLLER_LEFT_TRIGGER_AXIS, value=200),
            SimpleNamespace(type=FakeEcodes.EV_ABS, code=config.CONTROLLER_RIGHT_TRIGGER_AXIS, value=200),
        ])

        update = controller.poll()
        self.assertEqual(update.throttle_limit_delta, 0)

        controller.device.events = [
            SimpleNamespace(type=FakeEcodes.EV_ABS, code=config.CONTROLLER_LEFT_TRIGGER_AXIS, value=220),
        ]
        self.assertEqual(controller.poll().throttle_limit_delta, 0)

        controller.device.events = [
            SimpleNamespace(type=FakeEcodes.EV_ABS, code=config.CONTROLLER_LEFT_TRIGGER_AXIS, value=0),
            SimpleNamespace(type=FakeEcodes.EV_ABS, code=config.CONTROLLER_LEFT_TRIGGER_AXIS, value=255),
        ]
        self.assertEqual(controller.poll().throttle_limit_delta, -1)

if __name__ == "__main__":
    unittest.main()
