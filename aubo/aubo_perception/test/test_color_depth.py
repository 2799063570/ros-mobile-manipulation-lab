"""Depth surface localization and strict failure behavior for mobile sorting."""
import importlib.util
from pathlib import Path
from unittest import TestCase, main
from unittest.mock import Mock, patch

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo, Image

spec = importlib.util.spec_from_file_location(
    'color_depth', str(Path(__file__).parents[1] / 'scripts/color_object_detector.py'))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ColorDepthTest(TestCase):
    def setUp(self):
        rospy.rostime.set_rostime_initialized(True)
        params = {'~colors': {'red': {'ranges': [
            {'lower': [0, 100, 100], 'upper': [10, 255, 255]}]}},
            '~use_depth': True, '~require_depth': True,
            '~top_surface_tolerance': .004, '~max_depth_age': .06,
            '~projection_plane_z': .18, '~object_center_z': .16,
            '~workspace_min_x': 0., '~workspace_max_x': 2.,
            '~workspace_min_y': -2., '~workspace_max_y': 2.}
        with patch.object(rospy, 'get_param', side_effect=lambda k, d=None: params.get(k, d)), \
                patch.object(rospy, 'Publisher', side_effect=lambda *a, **k: Mock()), \
                patch.object(rospy, 'Subscriber'), patch.object(module.tf, 'TransformListener'):
            self.node = module.ColorObjectDetector()
        self.info = CameraInfo()
        self.info.header.frame_id = 'camera'
        self.info.K = [100., 0., 50., 0., 100., 50., 0., 0., 1.]
        self.node._camera_info_callback(self.info)
        self.transform = ([.5, 0., 1.], [1., 0., 0., 0.])
        self.node._listener.lookupTransform.return_value = self.transform

    def test_top_surface_rejects_side_points(self):
        mask = np.zeros((100, 100), np.uint8)
        mask[40:61, 40:71] = 255
        contour = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0][0]
        depth = np.full(mask.shape, .84, np.float32)  # side below top
        depth[40:61, 40:61] = .82  # symmetric top surface at z=.18
        self.node._depth_image = depth
        self.node._depth_stamp = rospy.Time(10)
        point = self.node._depth_top_center(contour, mask, self.info, self.transform, rospy.Time(10))
        np.testing.assert_allclose(point, [.5, 0., .16], atol=1e-6)
        # The whole color silhouette has a biased centroid, unlike its top.
        moments = cv2.moments(contour)
        legacy = self.node._pixel_to_table(moments['m10']/moments['m00'],
                                          moments['m01']/moments['m00'], self.info, self.transform)
        self.assertGreater(abs(legacy[0] - point[0]), .03)

    def test_missing_stale_and_invalid_depth_do_not_fall_back(self):
        rgb = np.zeros((100, 100, 3), np.uint8)
        rgb[40:61, 40:61, 2] = 255
        self.node._bridge = Mock()
        self.node._bridge.imgmsg_to_cv2.return_value = rgb
        self.node._pixel_to_table = Mock()
        msg = Image()
        msg.header.frame_id = 'camera'
        msg.header.stamp = rospy.Time(10)
        for depth, stamp in [(None, 10), (np.full((100, 100), .82), 9),
                             (np.full((100, 100), np.nan), 10)]:
            with self.subTest(stamp=stamp, missing=depth is None), patch.object(rospy, 'logwarn_throttle'):
                self.node._depth_image = depth
                self.node._depth_stamp = rospy.Time(stamp)
                self.node._image_callback(msg)
                self.assertEqual(self.node._detections_publisher.publish.call_args[0][0].objects, [])
        self.node._pixel_to_table.assert_not_called()

    def test_strict_depth_rejects_latest_tf_fallback(self):
        self.node.tf_wait_timeout = 0.0
        self.node._listener.lookupTransform.side_effect = module.tf.ExtrapolationException('late')
        with self.assertRaises(module.tf.ExtrapolationException):
            self.node._camera_transform('camera', rospy.Time(10))
        self.assertEqual(self.node._listener.lookupTransform.call_count, 1)

    def test_delayed_tf_retries_original_image_timestamp(self):
        self.node._listener.lookupTransform.side_effect = [
            module.tf.ExtrapolationException('TF is 10 ms behind'), self.transform]
        stamp = rospy.Time(10)
        with patch.object(module.time, 'sleep'):
            result = self.node._camera_transform('camera', stamp)
        self.assertEqual(result, self.transform)
        self.assertEqual(self.node._listener.lookupTransform.call_count, 2)
        for call in self.node._listener.lookupTransform.call_args_list:
            self.assertEqual(call.args[2], stamp)

    def test_tf_timeout_uses_wall_clock(self):
        self.node._listener.lookupTransform.side_effect = module.tf.ExtrapolationException('late')
        self.node.tf_wait_timeout = .2
        with patch.object(module.time, 'monotonic', side_effect=[0., .01, .21]), \
                patch.object(module.time, 'sleep'):
            with self.assertRaises(module.tf.ExtrapolationException):
                self.node._camera_transform('camera', rospy.Time(10))
        self.assertEqual(self.node._listener.lookupTransform.call_count, 2)


if __name__ == '__main__':
    main()
