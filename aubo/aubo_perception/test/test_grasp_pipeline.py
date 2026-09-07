"""直接执行几何节点回调，覆盖模式隔离和无效深度拒绝。"""
import importlib.util
from pathlib import Path
from unittest import TestCase, main
from unittest.mock import Mock, patch
import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo, Image
from aubo_perception.msg import YoloDetection, YoloDetectionArray

spec = importlib.util.spec_from_file_location('rgbd', str(Path(__file__).parents[1]/'scripts/yolo_rgbd_target_node.py'))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class PipelineTest(TestCase):
    def node(self, mode='sorting', height='depth'):
        rospy.rostime.set_rostime_initialized(True)
        params = {'~task_mode': mode, '~height_mode': height, '~selected_class': 'red',
                  '~minimum_probability': .5, '~maximum_depth_age': .05}
        with patch.object(rospy, 'get_param', side_effect=lambda name, default=None: params.get(name, default)), \
             patch.object(rospy, 'Publisher', side_effect=lambda *a, **k: Mock()), \
             patch.object(rospy, 'Subscriber'), patch.object(module.tf, 'TransformListener'):
            node = module.YoloRgbdTargetNode()
        node.tf_listener.lookupTransform.return_value = ([.5, 0., 1.], [1., 0., 0., 0.])
        info = CameraInfo()
        info.header.frame_id = 'camera'
        info.width = info.height = 100
        info.K = [100., 0., 50., 0., 100., 50., 0., 0., 1.]
        node._camera_info_callback(info)
        return node

    def send(self, node, depth=.86):
        if depth is not None:
            image = node.bridge.cv2_to_imgmsg(np.full((100, 100), depth, np.float32), '32FC1')
            image.header.stamp = rospy.Time(10)
            node._depth_callback(image)
        boxes = YoloDetectionArray()
        boxes.header.stamp = rospy.Time(10)
        boxes.header.frame_id = 'camera'
        for label, score in [('red', .7), ('blue', .99), ('red', .9)]:
            box = YoloDetection()
            box.class_name, box.confidence = label, score
            box.center_x = box.center_y = 50
            box.width, box.height, box.orientation_valid = 20, 10, True
            boxes.detections.append(box)
        node._boxes_callback(boxes)

    def test_sorting_publishes_all_classes_and_validated_center(self):
        node = self.node()
        self.send(node)
        result = node.detections_publisher.publish.call_args[0][0]
        self.assertEqual(len(result.objects), 3)
        self.assertAlmostEqual(result.objects[0].pose.position.z, .12, places=5)
        self.assertTrue(result.objects[0].depth_valid)
        self.assertAlmostEqual(result.objects[0].grasp_width, .086, places=5)
        node.target_pose_publisher.publish.assert_not_called()

    def test_bad_surface_never_falls_back_in_depth_mode(self):
        node = self.node()
        self.send(node, depth=.9)  # 桌面，不能当作 4 cm 物体的顶面
        self.assertEqual(node.detections_publisher.publish.call_args[0][0].objects, [])

    def test_table_mode_does_not_require_depth(self):
        node = self.node(height='table')
        self.send(node, depth=None)
        result = node.detections_publisher.publish.call_args[0][0]
        self.assertEqual(len(result.objects), 3)
        self.assertFalse(result.objects[0].depth_valid)
        self.assertAlmostEqual(result.objects[0].pose.position.z, .12)

    def test_servo_filters_class_without_table_height_validation(self):
        node = self.node(mode='servo')
        self.send(node, depth=.5)
        node.detections_publisher.publish.assert_not_called()
        self.assertEqual(node.target_label_publisher.publish.call_args[0][0].data, 'red')
        self.assertEqual(node.target_pose_publisher.publish.call_count, 1)
        self.assertAlmostEqual(node.target_confidence_publisher.publish.call_args[0][0].data, .9, places=5)
        self.assertEqual(node.target_pose_publisher.publish.call_args[0][0].header.stamp, rospy.Time(10))

if __name__ == '__main__':
    main()
