"""执行实际 Python 抓放方法，以假动作接口检查动作前校验和指令值。"""
import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

source = ast.parse((Path(__file__).parents[1]/'scripts/color_sorting_task.py').read_text())
original = next(n for n in source.body if isinstance(n, ast.ClassDef))
methods = [n for n in original.body if isinstance(n, ast.FunctionDef)
           and n.name in ('_pick_and_place', '_pick_and_place_impl')]
module = ast.Module(body=[ast.ClassDef(name='Task', bases=[], keywords=[], body=methods,
                                      decorator_list=[])], type_ignores=[])
env = dict(math=math, rospy=NS(loginfo=Mock(), logerr=Mock(), sleep=Mock()))
exec(compile(ast.fix_missing_locations(module), '<actual grasp methods>', 'exec'), env)

class GraspTest(unittest.TestCase):
    def setUp(self):
        self.t = t = env['Task']()
        for key, value in dict(height_mode='depth', use_detected_angle=True, use_detected_width=True,
                object_height=.04, table_z=.1, height_tolerance=.02, grasp_height_offset=.01,
                grasp_offset_x=0., grasp_offset_y=0., gripper_open=0., gripper_closed=.4,
                gripper_width_open=.08, gripper_width_closed=0., width_close_scale=.9,
                pregrasp_height=.22, preplace_height=.28, lift_min_height=.18,
                place_clearance=.02, place_frame='base_link', place_targets={'red':[.4, .2]},
                grasp_model_names={}, _active_grasp_angle=0.).items():
            setattr(t, key, value)
        t._pose = lambda x,y,z: (x,y,z,t._active_grasp_angle)
        t._xy_in_target_frame = lambda frame, xy: xy
        for name in ('_command_gripper', '_move_to_pose', '_cartesian_to', '_lift_with_recovery', '_set_grasp_attachment'):
            setattr(t, name, Mock(return_value=True))
        t._set_failure = Mock()
        self.d = NS(color='red', object_height=.04, depth_valid=True, grasp_geometry_valid=True,
                    grasp_width=.04, grasp_angle=.7, pose=NS(position=NS(x=.5,y=0.,z=.125)))

    def test_depth_width_angle_used_and_angle_reset(self):
        self.assertTrue(self.t._pick_and_place(self.d))
        self.assertAlmostEqual(self.t._cartesian_to.call_args_list[0][0][0][2], .135)
        self.assertAlmostEqual(self.t._cartesian_to.call_args_list[0][0][0][3], .7)
        self.assertAlmostEqual(self.t._command_gripper.call_args_list[1][0][0], .22)
        # 放置仍按目的桌面 + 半物高，不沿用有测量偏差的抓取 Z。
        self.assertAlmostEqual(self.t._cartesian_to.call_args_list[1][0][0][2], .15)
        self.assertEqual(self.t._active_grasp_angle, 0.)

    def test_invalid_geometry_cannot_move_robot(self):
        for field, value in [('depth_valid',False), ('grasp_geometry_valid',False),
                             ('grasp_width',.2), ('grasp_angle',float('nan'))]:
            old = getattr(self.d, field)
            setattr(self.d, field, value)
            self.assertFalse(self.t._pick_and_place(self.d))
            self.t._command_gripper.assert_not_called()
            self.t._move_to_pose.assert_not_called()
            setattr(self.d, field, old)
        self.d.pose.position.z = .4
        self.assertFalse(self.t._pick_and_place(self.d))
        self.t._command_gripper.assert_not_called()

if __name__ == '__main__':
    unittest.main()
