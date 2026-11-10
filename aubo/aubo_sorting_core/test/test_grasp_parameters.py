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
           and n.name in ('_category', '_pick_and_place', '_pick_and_place_impl')]
module = ast.Module(body=[ast.ClassDef(name='Task', bases=[], keywords=[], body=methods,
                                      decorator_list=[])], type_ignores=[])
env = dict(math=math, rospy=NS(loginfo=Mock(), logerr=Mock(), sleep=Mock()))
exec(compile(ast.fix_missing_locations(module), '<actual grasp methods>', 'exec'), env)

class GraspTest(unittest.TestCase):
    def setUp(self):
        self.t = t = env['Task']()
        for key, value in dict(height_mode='depth', allow_stacked_objects=False, use_detected_angle=True, use_detected_width=True,
                object_height=.04, table_z=.1, height_tolerance=.02, grasp_height_offset=.01,
                grasp_offset_x=0., grasp_offset_y=0., gripper_open=0., gripper_closed=.4,
                gripper_width_open=.08, gripper_width_closed=0., width_close_scale=.9,
                pregrasp_height=.22, preplace_height=.28, lift_min_height=.18,
                place_clearance=.02, minimum_place_z=None, place_frame='base_link', place_targets={'red':[.4, .2]},
                minimum_gripper_height=0.,
                grasp_height_reference='center', grasp_surface_inset=.01,
                stacked_grasp_offset_z=0., stacked_support_min_height=.02,
                grasp_model_names={}, _active_grasp_angle=0., place_object_yaw=None).items():
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

    def test_category_prefers_class_name_and_falls_back_to_color(self):
        self.d.class_name = 'can'
        self.assertEqual(self.t._category(self.d), 'can')
        self.d.class_name = ''
        self.assertEqual(self.t._category(self.d), 'red')

    def test_place_long_axis_aligns_x_after_lift_and_holds_through_release(self):
        self.t.place_object_yaw = 0.
        def lift(*args):
            self.assertAlmostEqual(self.t._active_grasp_angle, self.d.grasp_angle)
            return True
        self.t._lift_with_recovery.side_effect = lift
        self.assertTrue(self.t._pick_and_place(self.d))
        # Pickup follows the measured angle; pre-place, descent and retreat use Y closure.
        self.assertAlmostEqual(self.t._move_to_pose.call_args_list[0][0][0][3], .7)
        self.assertAlmostEqual(self.t._move_to_pose.call_args_list[1][0][0][3], math.pi/2)
        for call in self.t._cartesian_to.call_args_list[1:]:
            self.assertAlmostEqual(call[0][0][3], math.pi/2)
        self.assertEqual(self.t._active_grasp_angle, 0.)

    def test_place_uses_nearest_parallel_orientation_and_supports_y_axis(self):
        for pickup, target in ((-1.4, 0.), (1.4, math.pi/2), (.7, math.pi)):
            with self.subTest(pickup=pickup, target=target):
                self.d.grasp_angle = pickup
                self.t.place_object_yaw = target
                self.assertTrue(self.t._pick_and_place(self.d))
                angle = self.t._move_to_pose.call_args_list[-1][0][0][3]
                self.assertAlmostEqual(math.remainder(angle-target-math.pi/2, math.pi), 0.)
                self.assertLessEqual(abs(angle-pickup), math.pi/2)

    def test_lift_failure_does_not_rotate_for_placement_or_release(self):
        self.t.place_object_yaw = 0.
        self.t._lift_with_recovery.return_value = False
        self.assertFalse(self.t._pick_and_place(self.d))
        self.assertEqual(self.t._move_to_pose.call_count, 1)
        self.assertEqual(self.t._command_gripper.call_count, 2)  # Open then close; no release.
        self.assertEqual(self.t._active_grasp_angle, 0.)

    def test_unconfigured_place_keeps_pickup_angle(self):
        self.assertTrue(self.t._pick_and_place(self.d))
        self.assertAlmostEqual(self.t._move_to_pose.call_args_list[-1][0][0][3], .7)

    def test_minimum_place_z_does_not_change_grasp_height(self):
        self.t.minimum_place_z = .18
        self.assertTrue(self.t._pick_and_place(self.d))
        self.assertAlmostEqual(self.t._cartesian_to.call_args_list[0][0][0][2], .135)
        self.assertAlmostEqual(self.t._cartesian_to.call_args_list[1][0][0][2], .18)

    def test_floor_below_nominal_does_not_lower_release(self):
        self.t.minimum_place_z = .10
        self.assertTrue(self.t._pick_and_place(self.d))
        self.assertAlmostEqual(self.t._cartesian_to.call_args_list[1][0][0][2], .15)

    def test_minimum_gripper_height_clamps_both_descents_without_lowering_higher_targets(self):
        self.t.minimum_gripper_height=.06
        self.assertTrue(self.t._pick_and_place(self.d))
        self.assertAlmostEqual(self.t._cartesian_to.call_args_list[0][0][0][2], .16)
        self.assertAlmostEqual(self.t._cartesian_to.call_args_list[1][0][0][2], .16)
        self.assertEqual(self.t._command_gripper.call_args_list[0][0][0], self.t.gripper_open)
        self.assertEqual(self.t._command_gripper.call_args_list[-1][0][0], self.t.gripper_open)

    def test_gripper_floor_must_remain_below_the_approach(self):
        self.t.minimum_gripper_height=.3
        self.assertFalse(self.t._pick_and_place(self.d))
        self.t._command_gripper.assert_not_called()
        self.t._move_to_pose.assert_not_called()

    def test_measured_object_heights_and_six_centimeter_tcp_floor_are_independent(self):
        self.t.minimum_gripper_height = .06
        for height, expected in ((.065, .16), (.12, .17), (.20, .21)):
            with self.subTest(height=height):
                self.d.object_height = height
                self.d.pose.position.z = self.t.table_z + height/2
                self.t._cartesian_to.reset_mock()
                self.assertTrue(self.t._pick_and_place(self.d))
                self.assertAlmostEqual(self.t._cartesian_to.call_args_list[0][0][0][2], expected)

    def test_invalid_release_height_is_rejected_before_grasping(self):
        self.t.minimum_place_z = self.t.table_z + self.t.preplace_height
        self.assertFalse(self.t._pick_and_place(self.d))
        self.t._command_gripper.assert_not_called()
        self.t._move_to_pose.assert_not_called()
        self.assertEqual(self.t._set_failure.call_args[0][0], 'CONFIGURATION_FAILED')

    def test_elevated_support_changes_pick_height_but_not_destination_height(self):
        self.d.pose.position.z = .20
        self.assertFalse(self.t._pick_and_place(self.d))
        self.t._command_gripper.assert_not_called()
        self.t.allow_stacked_objects = True
        self.assertTrue(self.t._pick_and_place(self.d))
        self.assertAlmostEqual(self.t._cartesian_to.call_args_list[0][0][0][2], .21)
        self.assertAlmostEqual(self.t._cartesian_to.call_args_list[1][0][0][2], .15)

    def test_stack_mode_still_rejects_below_table_or_above_approach_before_motion(self):
        self.t.allow_stacked_objects = True
        for center in (.08, .4, float('nan')):
            self.d.pose.position.z = center
            self.assertFalse(self.t._pick_and_place(self.d))
            self.t._command_gripper.assert_not_called()
            self.t._move_to_pose.assert_not_called()

    def test_surface_reference_does_not_depend_on_the_thickness_prior_for_pick(self):
        self.t.allow_stacked_objects = True
        self.t.grasp_height_reference = 'surface'
        self.t.grasp_surface_inset = .01
        for thickness in (.04, .065):
            self.d.object_height = thickness
            self.d.pose.position.z = .24-thickness/2
            self.t._cartesian_to.reset_mock()
            self.assertTrue(self.t._pick_and_place(self.d))
            self.assertAlmostEqual(self.t._cartesian_to.call_args_list[0][0][0][2], .23)
            self.assertAlmostEqual(self.t._cartesian_to.call_args_list[1][0][0][2],
                                   self.t.table_z+thickness-.01+self.t.place_clearance)

    def test_upper_z_correction_raises_pick_only_for_elevated_objects_and_preserves_grip_offset(self):
        self.t.allow_stacked_objects=True
        self.t.grasp_height_reference='surface'
        self.t.stacked_grasp_offset_z=.01
        self.d.pose.position.z=.20  # Estimated bottom=.18, 8 cm above the table.
        self.assertTrue(self.t._pick_and_place(self.d))
        self.assertAlmostEqual(self.t._cartesian_to.call_args_list[0][0][0][2],.22)
        self.assertAlmostEqual(self.t._cartesian_to.call_args_list[1][0][0][2],.16)
        self.assertAlmostEqual(self.d.pose.position.z,.20)
        self.d.pose.position.z=.12
        self.t._cartesian_to.reset_mock()
        self.assertTrue(self.t._pick_and_place(self.d))
        self.assertAlmostEqual(self.t._cartesian_to.call_args_list[0][0][0][2],.13)

    def test_upper_z_correction_cannot_raise_tcp_past_approach(self):
        self.t.allow_stacked_objects=True
        self.d.pose.position.z=.27
        self.t.stacked_grasp_offset_z=.05
        self.assertFalse(self.t._pick_and_place(self.d))
        self.t._command_gripper.assert_not_called()
        self.t._move_to_pose.assert_not_called()

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
