"""Run actual callback bodies without ROS to test empty-frame safety semantics."""
import ast
import math
import time
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock


def callback_class(filename, names, environment):
    tree = ast.parse((Path(__file__).parents[1] / 'scripts' / filename).read_text(encoding='utf-8'))
    original = next(node for node in tree.body if isinstance(node, ast.ClassDef))
    methods = [node for node in original.body if isinstance(node, ast.FunctionDef) and node.name in names]
    node = ast.ClassDef(name='ActualCallbacks', bases=[], keywords=[], body=methods, decorator_list=[])
    module = ast.Module(body=[node], type_ignores=[])
    exec(compile(ast.fix_missing_locations(module), filename, 'exec'), environment)
    return environment['ActualCallbacks']


class ObservationContractTest(unittest.TestCase):
    def node(self):
        self.publisher = Mock()
        self.tf_error = type('TransformError', (Exception,), {})
        def array():
            return NS(header=NS(stamp=None, frame_id=''), objects=[], observation_valid=False, sensor_frame='')
        self.env = dict(DetectedObjectArray=array, tf=NS(Exception=self.tf_error),
                        rospy=NS(Duration=lambda n:n, logwarn_throttle=Mock()), math=math,
                        CvBridgeError=type('CvBridgeError', (Exception,), {}),
                        String=lambda **kw:NS(**kw), Float32=lambda **kw:NS(**kw))
        cls = callback_class('yolo_rgbd_target_node.py', {'_callback', '_publish_empty'}, self.env)
        node = cls()
        import threading
        node._info_lock = threading.Lock()
        node._camera_info = NS(header=NS(frame_id='camera'), width=100, height=100)
        node.target_frame = 'base_link'
        node.task_mode = 'sorting'
        node.minimum_probability = .5
        node.tf_listener = Mock()
        node.tf_listener.lookupTransform.return_value = ([], [])
        node.detections_publisher = self.publisher
        node._label = lambda box: 'red'
        return node

    def test_genuine_empty_frame_requires_source_time_tf(self):
        node = self.node()
        node._callback(NS(header=NS(stamp=42), bounding_boxes=[]), None)
        result = self.publisher.publish.call_args[0][0]
        self.assertTrue(result.observation_valid)
        self.assertEqual(result.sensor_frame, 'camera')
        self.assertEqual(result.header.stamp, 42)
        node.tf_listener.lookupTransform.assert_called_once_with('base_link', 'camera', 42)

    def test_tf_failure_is_not_empty_workspace(self):
        node = self.node()
        node.tf_listener.lookupTransform.side_effect = self.tf_error('missing')
        node._callback(NS(header=NS(stamp=42), bounding_boxes=[]), None)
        self.assertFalse(self.publisher.publish.call_args[0][0].observation_valid)

    def test_missing_depth_error_publication_cannot_complete_sorting(self):
        node = self.node()
        node._publish_empty(NS(stamp=42))
        result = self.publisher.publish.call_args[0][0]
        self.assertFalse(result.observation_valid)
        self.assertEqual(result.objects, [])

    def test_invalid_box_is_not_empty_workspace(self):
        node = self.node()
        box = NS(probability=.9, xmin=0, xmax=20, ymin=0, ymax=20, width=float('nan'), height=20)
        node._callback(NS(header=NS(stamp=42), bounding_boxes=[box]), None)
        self.assertFalse(self.publisher.publish.call_args[0][0].observation_valid)

    def test_color_table_mode_never_uses_latest_tf_for_old_image(self):
        error = type('TransformError', (Exception,), {})
        env = dict(time=time, rospy=NS(is_shutdown=lambda:False),
                   tf=NS(LookupException=error, ConnectivityException=error, ExtrapolationException=error))
        cls = callback_class('color_object_detector.py', {'_camera_transform'}, env)
        node = cls()
        node.require_depth = False
        node.tf_wait_timeout = 0
        node.target_frame = 'base_link'
        node._listener = Mock()
        node._listener.lookupTransform.side_effect = error('late')
        with self.assertRaises(error): node._camera_transform('camera', 42)
        node._listener.lookupTransform.assert_called_once_with('base_link', 'camera', 42)


if __name__ == '__main__':
    unittest.main()
