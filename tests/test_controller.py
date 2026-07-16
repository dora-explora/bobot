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
    }
    controller.supported_axis_codes = []
    controller.supported_key_codes = []
    controller.disconnected = False
    return controller


class ControllerInputTests(unittest.TestCase):
    def test_mode_buttons_are_edge_triggered(self):
        controller = controller_with_events([
            SimpleNamespace(type=FakeEcodes.EV_KEY, code=config.CONTROLLER_A_BUTTON, value=1),
            SimpleNamespace(type=FakeEcodes.EV_KEY, code=config.CONTROLLER_B_BUTTON, value=1),
            SimpleNamespace(type=FakeEcodes.EV_KEY, code=config.CONTROLLER_Y_BUTTON, value=1),
        ])

        update = controller.poll()

        self.assertTrue(update.a_pressed)
        self.assertTrue(update.b_pressed)
        self.assertTrue(update.y_pressed)

    def test_button_release_and_unmapped_input_do_not_change_modes(self):
        controller = controller_with_events([
            SimpleNamespace(type=FakeEcodes.EV_KEY, code=config.CONTROLLER_A_BUTTON, value=0),
            SimpleNamespace(type=FakeEcodes.EV_KEY, code=999, value=1),
        ])

        update = controller.poll()

        self.assertFalse(update.a_pressed)
        self.assertFalse(update.b_pressed)
        self.assertFalse(update.y_pressed)

    def test_radial_menu_uses_only_right_stick(self):
        controller = controller_with_events([])
        controller.axis_values[config.CONTROLLER_LEFT_X_AXIS] = 32767
        controller.axis_values[config.CONTROLLER_LEFT_Y_AXIS] = -32768

        self.assertEqual(controller.right_stick(), (0.0, 0.0))

        controller.axis_values[config.CONTROLLER_RIGHT_X_AXIS] = 32767
        controller.axis_values[config.CONTROLLER_RIGHT_Y_AXIS] = 0
        menu_x, menu_y = controller.right_stick()
        self.assertGreater(menu_x, 0.9)
        self.assertAlmostEqual(menu_y, 0.0, places=3)


if __name__ == "__main__":
    unittest.main()
