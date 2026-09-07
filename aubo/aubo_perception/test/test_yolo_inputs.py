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
