"""Servo control epochs reject in-flight old images; no camera/ROS master."""
import importlib.util
from pathlib import Path
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import rospy
from std_msgs.msg import String
from aubo_perception.msg import YoloDetectionArray

spec = importlib.util.spec_from_file_location('servo_geometry', str(
    Path(__file__).resolve().parents[1]/'scripts/yolo_rgbd_target_node.py'))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ServoControlsTest(unittest.TestCase):
    def setUp(self):
        rospy.rostime.set_rostime_initialized(True)
        self.node = module.YoloRgbdTargetNode.__new__(module.YoloRgbdTargetNode)
        self.node._servo_controls = True
        self.node._vision_lock = threading.RLock()
        self.node._enabled = True
        self.node._vision_epoch = rospy.Time(0)
        self.node.selected_class = 'can'
        self.node._state_publisher = Mock()
        self.node.target_label_publisher = Mock()
        self.node.target_confidence_publisher = Mock()
        self.node._process_boxes = Mock()

    def boxes(self, stamp):
        result = YoloDetectionArray()
        result.header.stamp = rospy.Time.from_sec(stamp)
        return result

    def test_selection_requires_new_source_image_and_invalidates_label(self):
        with patch.object(rospy.Time, 'now', return_value=rospy.Time(20)):
            self.node._select_target(String(data='Bottle'))
        self.assertEqual(self.node.selected_class, 'bottle')
        self.assertEqual(self.node.target_label_publisher.publish.call_args[0][0].data, '')
        self.node._boxes_callback(self.boxes(19.9))
        self.node._boxes_callback(self.boxes(20.))
        self.node._process_boxes.assert_not_called()
        self.node._boxes_callback(self.boxes(20.1))
        self.node._process_boxes.assert_called_once()

    def test_disable_enable_and_reset_do_not_replay_old_targets(self):
        with patch.object(rospy.Time, 'now', return_value=rospy.Time(20)):
            self.assertTrue(self.node._set_enabled(NS(data=False)).success)
            self.node._boxes_callback(self.boxes(21))
            self.node._process_boxes.assert_not_called()
            self.assertTrue(self.node._set_enabled(NS(data=True)).success)
            self.node._boxes_callback(self.boxes(19))
            self.node._process_boxes.assert_not_called()
            self.node._boxes_callback(self.boxes(21))
            self.node._process_boxes.assert_called_once()
        with patch.object(rospy.Time, 'now', return_value=rospy.Time(22)):
            self.node._reset(NS())
        self.node._boxes_callback(self.boxes(21.5))
        self.node._process_boxes.assert_called_once()

    def test_bad_selection_is_ignored(self):
        for label in ('', 'can|DETECTED', 'a'*65): self.node._select_target(String(data=label))
        self.assertEqual(self.node.selected_class, 'can')
        self.node._state_publisher.publish.assert_not_called()


if __name__ == '__main__': unittest.main()
