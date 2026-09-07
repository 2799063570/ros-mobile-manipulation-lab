"""Run actual Python lift/cartesian methods with fake transport, without ROS."""
import ast
import copy
import inspect
import math
from pathlib import Path
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock

source = ast.parse((Path(__file__).parents[1] / 'scripts/color_sorting_task.py').read_text())
original = next(n for n in source.body if isinstance(n, ast.ClassDef))
methods = [n for n in original.body if isinstance(n, ast.FunctionDef)
           and n.name in ('_lift_with_recovery', '_cartesian_to')]
module = ast.Module(body=[ast.ClassDef(name='Task', bases=[], keywords=[], body=methods,
                                      decorator_list=[])], type_ignores=[])
ros = SimpleNamespace(is_shutdown=lambda: False, loginfo=MagicMock(), logwarn=MagicMock())
env = dict(math=math, rospy=ros, String=lambda **kw: kw, copy=copy, inspect=inspect)
exec(compile(ast.fix_missing_locations(module), '<actual lift methods>', 'exec'), env)

class LiftTest(unittest.TestCase):
    def setUp(self):
        self.task = env['Task']()
        t = self.task
        t.lift_height, t.lift_min_height, t.lift_height_step = .26, .18, .02
        t.lift_max_attempts, t.table_z = 5, .14
        t._stop_requested = threading.Event()
        t._pose = lambda x, y, z: (x, y, z)
        t._last_failure = ''
        t._failure_publisher = MagicMock()
        t._set_failure = lambda category, detail: setattr(t, '_last_failure', category+' | '+detail)

    def test_lowering_stops_on_success(self):
        calls = []
        def attempt(pose, label, require_complete=False):
            self.assertTrue(require_complete)
            calls.append(pose[2])
            self.task._last_failure = 'PLANNING_FAILED | test' if len(calls) < 3 else ''
            return len(calls) == 3
        self.task._cartesian_to = attempt
        self.assertTrue(self.task._lift_with_recovery(1, 2, 'lift'))
        self.assertEqual(len(calls), 3)
        self.assertAlmostEqual(calls[-1], .36)
        self.assertEqual(self.task._last_failure, '')

    def test_exhaustion_honors_floor(self):
        calls = []
        def attempt(pose, *args, **kw):
            calls.append(pose[2])
            self.task._last_failure = 'PLANNING_FAILED | test'
            return False
        self.task._cartesian_to = attempt
        self.assertFalse(self.task._lift_with_recovery(1, 2, 'lift'))
        self.assertEqual(len(calls), 5)
        self.assertAlmostEqual(min(calls), .32)

    def test_execution_failure_and_stop_do_not_retry(self):
        for stopped in (False, True):
            with self.subTest(stopped=stopped):
                self.task._stop_requested.clear()
                def attempt(*args, **kw):
                    self.task._last_failure = 'EXECUTION_FAILED | test'
                    if stopped:
                        self.task._stop_requested.set()
                    return False
                self.task._cartesian_to = MagicMock(side_effect=attempt)
                self.assertFalse(self.task._lift_with_recovery(1, 2, 'lift'))
                self.assertEqual(self.task._cartesian_to.call_count, 1)

    def test_partial_lift_is_not_executed(self):
        t = self.task
        t.minimum_cartesian_fraction, t.cartesian_step, t.require_octomap = .90, .01, False
        t.arm = MagicMock()
        def compute(waypoints, step, avoid_collisions=True):
            return SimpleNamespace(), .958
        t.arm.compute_cartesian_path = compute
        self.assertFalse(t._cartesian_to(SimpleNamespace(pose=SimpleNamespace()), 'lift', True))
        t.arm.execute.assert_not_called()
        self.assertTrue(t._last_failure.startswith('PLANNING_FAILED |'))

if __name__ == '__main__':
    unittest.main()
