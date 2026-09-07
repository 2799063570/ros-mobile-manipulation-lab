"""几何回归：无相机、无 ROS master，不发送机器人动作。"""
import math
import unittest
import numpy as np
from aubo_perception.grasp_geometry import center_height, rectangle_grasp, ray_plane, validate_modes


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

    def test_table_projection_without_depth(self):
        point = ray_plane(60, 50, self.K, self.R, self.t, .14)
        np.testing.assert_allclose(point, [.086, 0, .14])
        with self.assertRaises(ValueError):
            ray_plane(50, 50, self.K, np.eye(3), self.t, .14)

    def test_servo_requires_explicit_class(self):
        validate_modes('sorting', 'table', 'eye_to_hand', '')
        with self.assertRaises(ValueError):
            validate_modes('servo', 'depth', 'eye_in_hand', '')
        with self.assertRaises(ValueError):
            validate_modes('sorting', 'fallback', 'eye_in_hand', '')

if __name__ == '__main__':
    unittest.main()
