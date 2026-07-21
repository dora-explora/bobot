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
    controller._menu_stick_source = "right"
    controller._menu_input_sequence = 0
    controller._menu_stick_sequence = {"left": 0, "right": 0}
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


if __name__ == "__main__":
    unittest.main()
