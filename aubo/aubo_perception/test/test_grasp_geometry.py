"""几何回归：无相机、无 ROS master，不发送机器人动作。"""
import math
import unittest
import numpy as np
from aubo_perception.grasp_geometry import (center_height, object_height_from_depth, bounded_depth_center,
                                           rectangle_grasp, ray_plane, validate_modes)


class GeometryTest(unittest.TestCase):
    def setUp(self):
        self.K = [100., 0., 50., 0., 100., 50., 0., 0., 1.]
        self.R = np.diag([1., -1., -1.])
        self.t = [0., 0., 1.]

    def test_metric_short_side_and_camera_rotation(self):
        width, angle = rectangle_grasp(50, 50, 20, 10, 0, self.K, self.R, self.t, 0)
        self.assertAlmostEqual(width, .1)
        self.assertAlmostEqual(abs(angle), math.pi/2)
        rz = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
        width, angle = rectangle_grasp(50, 50, 20, 10, 0, self.K, rz.dot(self.R), self.t, 0)
        self.assertAlmostEqual(width, .1)
        self.assertAlmostEqual(angle, 0)

    def test_surface_height_is_not_object_center(self):
        self.assertAlmostEqual(center_height(.145, .10, .04, .02), .125)
        for z in (.10, .20, float('nan')):
            with self.assertRaises(ValueError):
                center_height(z, .10, .04, .02)

    def test_measured_height_is_surface_minus_table_with_bounded_noise_rejection(self):
        for height in (.005, .065, .12, .20, .30):
            with self.subTest(height=height):
                self.assertAlmostEqual(object_height_from_depth(.1+height, .1, .005, .30), height)
        for surface in (.1, .09, .41, float('nan'), float('inf')):
            with self.subTest(surface=surface), self.assertRaises(ValueError):
                object_height_from_depth(surface, .1, .005, .30)
        with self.assertRaisesRegex(ValueError, 'height=0.40000.*allowed='):
            object_height_from_depth(.5, .1, .005, .30)
        with self.assertRaisesRegex(ValueError, 'surface_z=0.20000.*expected_z=0.14000'):
            center_height(.2, .1, .04, .02)

    def test_table_projection_without_depth(self):
        point = ray_plane(60, 50, self.K, self.R, self.t, .14)
        np.testing.assert_allclose(point, [.086, 0, .14])
        with self.assertRaises(ValueError):
            ray_plane(50, 50, self.K, np.eye(3), self.t, .14)

    def test_stack_height_is_added_in_full_with_constant_own_thickness(self):
        table, thickness = .00285, .065
        base, height = bounded_depth_center(table+thickness, table, thickness, .005, .30)
        elevated, elevated_height = bounded_depth_center(table+thickness+.08, table, thickness, .005, .30)
        self.assertAlmostEqual(elevated-base, .08)
        self.assertAlmostEqual(height, thickness)
        self.assertAlmostEqual(elevated_height, thickness)

    def test_bounded_depth_keeps_short_table_objects_and_rejects_bad_returns(self):
        center, height = bounded_depth_center(.04285, .00285, .065, .005, .30)
        self.assertAlmostEqual(center, .02285)
        self.assertAlmostEqual(height, .04)
        for surface in (.00285, -.01, .4, float('nan')):
            with self.subTest(surface=surface), self.assertRaises(ValueError):
                bounded_depth_center(surface, .00285, .065, .005, .30)
        for limit in (0, .001, .4, float('inf')):
            with self.subTest(limit=limit), self.assertRaises(ValueError):
                bounded_depth_center(.1, .00285, limit, .005, .30)

    def test_servo_requires_explicit_class(self):
        validate_modes('sorting', 'table', 'eye_to_hand', '')
        with self.assertRaises(ValueError):
            validate_modes('servo', 'depth', 'eye_in_hand', '')
        with self.assertRaises(ValueError):
            validate_modes('sorting', 'fallback', 'eye_in_hand', '')

if __name__ == '__main__':
    unittest.main()
