#!/usr/bin/env python3
"""No camera, model inference or ROS master required."""
import importlib.util
from pathlib import Path
from collections import deque
from types import SimpleNamespace
import threading
import tempfile
import unittest
from unittest.mock import Mock

import numpy as np
import rospy
from sensor_msgs.msg import Image
from aubo_perception.msg import YoloDetectionArray, YoloDetection

spec = importlib.util.spec_from_file_location(
    'bridge', str(Path(__file__).resolve().parents[1] / 'scripts/yolo_rgbd_target_node.py'))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class DepthCacheTest(unittest.TestCase):
    def setUp(self):
        rospy.rostime.set_rostime_initialized(True)
        self.node = module.YoloRgbdTargetNode.__new__(module.YoloRgbdTargetNode)
        self.node._depth_lock = threading.Lock()
        self.node._depth_cache = deque(maxlen=3)
        self.node.backend = 'ultralytics'
        self.node.height_mode = 'depth'
        self.node.task_mode = 'sorting'
        self.node.target_frame = 'base_link'
        self.node.detections_publisher = Mock()
        self.node.maximum_depth_age = 0.05
        self.node._callback = Mock()

    def depth(self, stamp):
        image = Image()
        image.header.stamp = rospy.Time.from_sec(stamp)
        self.node._depth_callback(image)
        return image

    def boxes(self, stamp):
        boxes = YoloDetectionArray()
        boxes.header.stamp = rospy.Time.from_sec(stamp)
        box = YoloDetection()
        box.class_name = 'bottle'
        box.center_x, box.center_y = 40., 60.
        box.width, box.height = 20., 30.
        box.confidence = 0.9
        boxes.detections.append(box)
        return boxes

    def test_delayed_inference_uses_source_frame(self):
        original = self.depth(10.)
        self.depth(11.)
        self.depth(12.)
        self.node._boxes_callback(self.boxes(10.01))
        boxes, selected = self.node._callback.call_args[0]
        self.assertIs(selected, original)
        self.assertEqual(boxes.bounding_boxes[0].xmin, 30.)
        self.assertEqual(boxes.bounding_boxes[0].class_name, 'bottle')

    def test_unmatched_depth_is_rejected(self):
        self.depth(12.)
        self.node._boxes_callback(self.boxes(10.))
        self.node._callback.assert_not_called()

    def test_zero_timestamp_is_rejected(self):
        self.depth(12.)
        self.node._boxes_callback(self.boxes(0.))
        self.node._callback.assert_not_called()

    def test_cache_is_bounded_and_resets_with_clock(self):
        for stamp in (10., 11., 12., 13.):
            self.depth(stamp)
        self.assertEqual(len(self.node._depth_cache), 3)
        self.depth(1.)
        self.assertEqual(len(self.node._depth_cache), 1)

    def test_projection_uses_intrinsics(self):
        info = SimpleNamespace(K=[100., 0., 50., 0., 200., 60., 0., 0., 1.],
                               header=SimpleNamespace(frame_id='camera'))
        pose = self.node._camera_pose(info, rospy.Time(10), 2., 60., 80.)
        self.assertAlmostEqual(pose.pose.position.x, 0.2)
        self.assertAlmostEqual(pose.pose.position.y, 0.2)
        self.assertAlmostEqual(pose.pose.position.z, 2.)

    def test_saved_depth_preserves_raw_units_and_timestamps(self):
        from sensor_msgs.msg import CameraInfo
        from cv_bridge import CvBridge
        node = self.node
        node.bridge = CvBridge()
        node._info_lock = threading.Lock()
        info = CameraInfo()
        info.width = info.height = 10
        info.header.frame_id = 'camera'
        info.K = [100., 0., 5., 0., 100., 5., 0., 0., 1.]
        node._camera_info = info
        node.depth_scale_16u = 0.001
        node.target_frame = 'base_link'
        node.detections_publisher = Mock()
        depth = node.bridge.cv2_to_imgmsg(np.full((10, 10), 1200, dtype=np.uint16), '16UC1')
        depth.header.stamp = rospy.Time(10)
        boxes = SimpleNamespace(header=SimpleNamespace(stamp=rospy.Time(10, 100)), bounding_boxes=[])
        with tempfile.TemporaryDirectory() as directory:
            node.depth_save_directory = directory
            module.YoloRgbdTargetNode._callback(node, boxes, depth)
            with np.load(str(Path(directory) / '10000000000.npz')) as saved:
                self.assertEqual(saved['depth'].dtype, np.uint16)
                self.assertEqual(saved['depth'][0, 0], 1200)
                self.assertEqual(int(saved['image_stamp_ns']), 10000000100)
                np.testing.assert_array_equal(saved['K'], info.K)

    def test_depth_median_excludes_invalid_samples(self):
        self.node.minimum_depth = 0.1
        self.node.maximum_depth = 3.
        self.node.roi_scale = 1.
        depth = np.full((10, 10), 1.2)
        depth[0, :3] = [0., np.nan, np.inf]
        box = SimpleNamespace(xmin=0, xmax=10, ymin=0, ymax=10)
        self.assertAlmostEqual(self.node._depth_for_box(depth, box)[0], 1.2)


if __name__ == '__main__':
    unittest.main()
