"""不加载模型权重，验证输入模式、离线时间戳及 OBB 元数据。"""
import importlib.util
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch
import cv2
import numpy as np
import rospy
from aubo_perception.msg import YoloDetectionArray

spec = importlib.util.spec_from_file_location('inference', str(Path(__file__).parents[1]/'scripts/ultralytics_yolo_node.py'))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

class InputTest(unittest.TestCase):
    def test_empty_image_dimensions_are_rejected(self):
        for width, height in ((0, 8), (10, 0), (0, 0)):
            with self.subTest(width=width, height=height):
                message = module.Image()
                message.encoding = 'bgr8'
                message.width, message.height = width, height
                message.step = width * 3
                message.data = bytes(height * message.step)
                with self.assertRaisesRegex(ValueError, 'width and height'):
                    module.image_message_to_bgr(message)

    def test_rgb_row_padding_is_not_decoded_as_pixels(self):
        message = module.Image()
        message.encoding = 'rgb8'
        message.width, message.height, message.step = 1, 2, 4
        message.data = bytes([1, 2, 3, 255, 4, 5, 6, 255])
        np.testing.assert_array_equal(module.image_message_to_bgr(message),
                                      [[[3, 2, 1]], [[6, 5, 4]]])

    def test_topic_waits_for_camera_and_preserves_timestamp(self):
        rospy.rostime.set_rostime_initialized(True)
        with tempfile.TemporaryDirectory() as directory:
            model_path = Path(directory) / 'model.pt'
            model_path.touch()
            params = {'~project_path': directory, '~model_path': str(model_path),
                      '~input_mode': 'topic', '~image_topic': '/camera/color/image_raw',
                      '~image_path': '/must/not/read/test.jpg'}
            result = NS(obb=None, boxes=None, plot=lambda: np.zeros((8, 10, 3), np.uint8))
            model = Mock()
            model.predict.return_value = [result]
            old_path = list(sys.path)
            try:
                with patch.object(rospy, 'get_param', side_effect=lambda n,d=None: params.get(n,d)), \
                     patch.object(rospy, 'Publisher', side_effect=lambda *a,**k: Mock()) as publisher, \
                     patch.object(rospy, 'Subscriber') as subscriber, \
                     patch.object(cv2, 'imread') as imread, \
                     patch.dict(sys.modules, {'ultralytics': NS(YOLO=lambda path: model)}):
                    node = module.UltralyticsYoloNode()
                    model.predict.assert_not_called()
                    self.assertEqual(subscriber.call_args[0][0], '/camera/color/image_raw')
                    self.assertFalse(any(call[1]['latch'] for call in publisher.call_args_list))
                    source = module.Image()
                    source.header.stamp = rospy.Time(12, 345)
                    source.header.frame_id = 'camera_color_optical_frame'
                    message = module.bgr_to_image_message(np.zeros((8, 10, 3), np.uint8), source)
                    subscriber.call_args[0][2](message)
                    imread.assert_not_called()
                    model.predict.assert_called_once()
                    output = node.detections_publisher.publish.call_args[0][0]
                    self.assertEqual(output.header, source.header)
            finally:
                sys.path[:] = old_path

    def test_local_image_latched_once_and_never_subscribes(self):
        rospy.rostime.set_rostime_initialized(True)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/'model.pt').touch()
            cv2.imwrite(str(root/'image.png'), np.zeros((8, 10, 3), np.uint8))
            params = {'~project_path': directory, '~model_path': str(root/'model.pt'),
                      '~input_mode':'image', '~image_path':str(root/'image.png')}
            result = NS(obb=None, boxes=None, plot=lambda: np.zeros((8, 10, 3), np.uint8))
            model = Mock()
            model.predict.return_value = [result]
            old_path = list(sys.path)
            try:
                with patch.object(rospy, 'get_param', side_effect=lambda n,d=None: params.get(n,d)), \
                     patch.object(rospy, 'Publisher', side_effect=lambda *a,**k: Mock()) as publisher, \
                     patch.object(rospy, 'Subscriber') as subscriber, \
                     patch.dict(sys.modules, {'ultralytics':NS(YOLO=lambda path: model)}):
                    node = module.UltralyticsYoloNode()
                subscriber.assert_not_called()
                self.assertEqual(model.predict.call_count, 1)
                self.assertTrue(all(call[1]['latch'] for call in publisher.call_args_list))
                output = node.detections_publisher.publish.call_args[0][0]
                self.assertEqual(output.header.stamp, rospy.Time(0))
                self.assertEqual(node.annotated_publisher.publish.call_count, 1)
            finally:
                sys.path[:] = old_path

    def test_obb_orientation_is_explicit(self):
        node = module.UltralyticsYoloNode.__new__(module.UltralyticsYoloNode)
        result = NS(names={0:'bottle'})
        box = node._new_detection(result, [50,60,20,10], 0, .9)
        self.assertFalse(box.orientation_valid)
        box = node._new_detection(result, [50,60,20,10], 0, .9, .7, True)
        self.assertTrue(box.orientation_valid)
        self.assertAlmostEqual(box.angle, .7)

if __name__ == '__main__':
    unittest.main()
