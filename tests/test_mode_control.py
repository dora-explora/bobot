import unittest

from robot.controller import ControllerUpdate
from robot.mode_control import ModeControl
from robot.models import DriveCommand


class ModeControlTests(unittest.TestCase):
    def test_a_enters_manual_from_static_after_neutral_transition_frame(self):
        modes = ModeControl("static")

        decision = modes.update(ControllerUpdate(a_pressed=True), (0.0, 0.0))

        self.assertEqual(modes.active_state, "manual")
        self.assertTrue(decision.neutralize_this_frame)
        planned = DriveCommand(mode="manual", left=0.4, right=0.4)
        self.assertEqual(modes.gate_command(planned, decision.neutralize_this_frame).mode, "disabled")
        self.assertIs(modes.gate_command(planned), planned)

    def test_releasing_y_selects_detector_and_requires_a_before_output_is_enabled(self):
        modes = ModeControl("static")
        modes.update(ControllerUpdate(y_pressed=True), (0.0, 1.0))

        decision = modes.update(ControllerUpdate(y_released=True), (0.0, 1.0))

        self.assertEqual(modes.active_state, "detector")
        self.assertFalse(modes.detector_throttle_enabled)
        self.assertTrue(decision.neutralize_this_frame)
        planned = DriveCommand(steering=0.8, throttle=0.1, mode="assist")
        gated = modes.gate_command(planned)
        self.assertEqual(gated.mode, "disabled")
        self.assertEqual(gated.steering, 0.0)
        self.assertEqual(gated.throttle, 0.0)

        modes.update(ControllerUpdate(a_pressed=True), (0.0, 0.0))
        self.assertTrue(modes.detector_throttle_enabled)
        self.assertIs(modes.gate_command(planned), planned)

    def test_second_a_from_enabled_detector_enters_manual(self):
        modes = ModeControl("detector")
        modes.update(ControllerUpdate(a_pressed=True), (0.0, 0.0))

        decision = modes.update(ControllerUpdate(a_pressed=True), (0.0, 0.0))

        self.assertEqual(modes.active_state, "manual")
        self.assertFalse(modes.detector_throttle_enabled)
        self.assertTrue(decision.neutralize_this_frame)

    def test_b_enters_static_outside_menu(self):
        modes = ModeControl("manual")

        decision = modes.update(ControllerUpdate(b_pressed=True), (0.0, 0.0))

        self.assertEqual(modes.active_state, "static")
        self.assertTrue(decision.neutralize_this_frame)
        self.assertFalse(modes.output_enabled)

    def test_b_takes_priority_over_simultaneous_menu_press(self):
        modes = ModeControl("manual")

        modes.update(ControllerUpdate(b_pressed=True, y_pressed=True), (0.0, 1.0))

        self.assertEqual(modes.active_state, "static")
        self.assertFalse(modes.menu_active)

    def test_b_closes_menu_to_paused_state_with_neutral_transition(self):
        modes = ModeControl("manual")
        modes.update(ControllerUpdate(y_pressed=True), (0.0, 0.0))
        self.assertTrue(modes.menu_active)
        self.assertFalse(modes.output_enabled)

        decision = modes.update(ControllerUpdate(b_pressed=True), (0.0, 0.0))

        self.assertFalse(modes.menu_active)
        self.assertEqual(modes.active_state, "manual")
        self.assertTrue(decision.neutralize_this_frame)

    def test_stick_selects_each_radial_sector(self):
        self.assertEqual(ModeControl.radial_selection((0.0, 1.0)), "detector")
        self.assertEqual(ModeControl.radial_selection((1.0, 0.0)), "manual")
        self.assertEqual(ModeControl.radial_selection((0.0, -1.0)), "capture")
        self.assertEqual(ModeControl.radial_selection((-1.0, 0.0)), "static")
        self.assertIsNone(ModeControl.radial_selection((0.0, 0.0)))

    def test_capture_state_never_enables_output(self):
        modes = ModeControl("capture")

        self.assertFalse(modes.output_enabled)
        command = DriveCommand(mode="capture", throttle=1.0, left=1.0, right=1.0)
        gated = modes.gate_command(command)

        self.assertEqual(gated.mode, "disabled")
        self.assertEqual(gated.throttle, 0.0)

    def test_controller_loss_stops_enabled_detector(self):
        modes = ModeControl("detector")
        modes.update(ControllerUpdate(a_pressed=True), (0.0, 0.0))

        decision = modes.update(ControllerUpdate(controller_lost=True), (0.0, 0.0))

        self.assertEqual(modes.active_state, "static")
        self.assertTrue(decision.neutralize_this_frame)


if __name__ == "__main__":
    unittest.main()
