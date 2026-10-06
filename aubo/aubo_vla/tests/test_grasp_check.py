import unittest

from check_grasp_recording import task_state, verify_placements


class GraspCheckTest(unittest.TestCase):
    def test_task_state_with_details(self):
        self.assertEqual(task_state('READY | waiting for panel command'), 'READY')
        self.assertEqual(task_state('ERROR | operation failed'), 'ERROR')

    def test_measured_lift_and_placement_required(self):
        initial = {'red_block': [0.54, -0.12, 0.12]}
        final = {'red_block': [0.44, -0.25, 0.12]}
        targets = {'red_block': [0.44, -0.25]}
        verify_placements(initial, final, {'red_block': 0.35}, targets)
        with self.assertRaisesRegex(RuntimeError, 'No measured lift'):
            verify_placements(initial, final, {'red_block': 0.12}, targets)
        with self.assertRaisesRegex(RuntimeError, 'outside'):
            verify_placements(initial, initial, {'red_block': 0.35}, targets)
        with self.assertRaisesRegex(RuntimeError, 'outside'):
            verify_placements(initial, {'red_block': [0.44, -0.25, 0.5]},
                              {'red_block': 0.5}, targets)
