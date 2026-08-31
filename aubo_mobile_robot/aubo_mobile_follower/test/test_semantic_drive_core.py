#!/usr/bin/env python3

import math
import unittest

from aubo_mobile_follower.semantic_drive_core import (
    DetectionGate,
    SemanticDriveCore,
    normalize_angle,
)


class DetectionGateTest(unittest.TestCase):
    def test_requires_consecutive_labels(self):
        gate = DetectionGate(3)
        self.assertIsNone(gate.update("stop"))
        self.assertIsNone(gate.update("turn_left"))
        self.assertIsNone(gate.update("turn_left"))
        self.assertEqual(gate.update("turn_left"), "turn_left")

    def test_persistent_label_does_not_retrigger_until_it_disappears(self):
        gate = DetectionGate(2)
        self.assertIsNone(gate.update("stop"))
        self.assertEqual(gate.update("stop"), "stop")
        self.assertIsNone(gate.update("stop"))
        self.assertIsNone(gate.update("stop"))
        self.assertIsNone(gate.update(None))
        self.assertIsNone(gate.update("stop"))
        self.assertEqual(gate.update("stop"), "stop")


class SemanticDriveCoreTest(unittest.TestCase):
    def setUp(self):
        self.core = SemanticDriveCore(
            {
                "stop_hold_time": 2.0,
                "slow_scale": 0.25,
                "slow_duration": 3.0,
                "turn_angle": math.pi / 2.0,
                "turn_kp": 1.0,
                "turn_min_speed": 0.1,
                "turn_max_speed": 0.4,
                "turn_tolerance": 0.02,
                "turn_settle_time": 0.2,
                "cooldown": 4.0,
            }
        )

    def test_stop_then_resume_nominal_command(self):
        self.assertTrue(self.core.trigger("stop", 10.0))
        self.assertEqual(self.core.command(0.2, 0.1, 11.0)[:2], (0.0, 0.0))
        self.assertEqual(self.core.command(0.2, 0.1, 12.1)[:2], (0.2, 0.1))

    def test_slow_scales_both_differential_drive_components(self):
        self.assertTrue(self.core.trigger("slow_down", 5.0))
        linear, angular, state = self.core.command(0.2, -0.4, 6.0)
        self.assertAlmostEqual(linear, 0.05)
        self.assertAlmostEqual(angular, -0.1)
        self.assertEqual(state, SemanticDriveCore.SLOW_FOLLOW)

    def test_left_turn_uses_wrapped_yaw_and_settles(self):
        start = math.radians(170.0)
        self.assertTrue(self.core.trigger("turn_left", 1.0, start))
        linear, angular, state = self.core.command(0.2, 0.0, 1.1, start)
        self.assertEqual(linear, 0.0)
        self.assertGreater(angular, 0.0)
        self.assertEqual(state, SemanticDriveCore.TURN_LEFT)

        target = normalize_angle(start + math.pi / 2.0)
        self.assertEqual(self.core.command(0.2, 0.0, 2.0, target)[:2], (0.0, 0.0))
        linear, angular, state = self.core.command(0.2, 0.0, 2.3, target)
        self.assertEqual((linear, angular), (0.0, 0.0))
        self.assertEqual(state, SemanticDriveCore.FOLLOW_LINE)

    def test_cooldown_rejects_repeated_action(self):
        self.assertTrue(self.core.trigger("stop", 1.0))
        self.assertFalse(self.core.trigger("stop", 2.0))
        self.assertTrue(self.core.trigger("stop", 5.1))

    def test_stop_interrupts_turn_and_turn_cannot_interrupt_stop(self):
        self.assertTrue(self.core.trigger("turn_left", 1.0, 0.0))
        self.assertTrue(self.core.trigger("stop", 1.1, 0.0))
        self.assertFalse(self.core.trigger("turn_right", 1.2, 0.0))
        self.assertEqual(self.core.command(0.2, 0.0, 1.3)[:2], (0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
