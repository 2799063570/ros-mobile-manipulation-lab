"""Resolve the unified real entry without starting camera, SDK or Qt."""
from pathlib import Path
import json
import unittest

import roslaunch
import roslaunch.rlutil


def resolve(package='aubo_ros_control', name='eye_to_hand_visual_servo_real.launch', args=()):
    filename = roslaunch.rlutil.resolve_launch_arguments([package, name])[0]
    config = roslaunch.config.ROSLaunchConfig()
    roslaunch.xmlloader.XmlLoader().load(filename, config, argv=list(args), verbose=False)
    return config, {key: value.value for key, value in config.params.items()}


class UnifiedRealServoLaunchTest(unittest.TestCase):
    def test_default_is_yolo_eye_to_hand_sdk_with_ui_and_explicit_enable(self):
        config, params = resolve()
        nodes = {(node.package, node.type) for node in config.nodes}
        self.assertIn(('aubo_ros_control', 'aubo_visual_servo_node'), nodes)
        self.assertIn(('aubo_sorting_rqt', 'visual_servo_panel'), nodes)
        self.assertIn(('moveit_ros_move_group', 'move_group'), nodes)
        self.assertFalse(any(node.type == 'aubo_hw_node' or node.package in
                             ('aubo_sorting_core', 'inspire_gripper', 'gazebo_ros') for node in config.nodes))
        self.assertEqual(params['/aubo_visual_servo/servo_mode'], 'eye_to_hand')
        self.assertEqual(params['/aubo_visual_servo/backend'], 'sdk')
        self.assertFalse(params['/aubo_visual_servo/start_enabled'])
        self.assertFalse(params['/aubo_visual_servo/hybrid_enabled'])
        self.assertFalse(params['/move_group/allow_trajectory_execution'])
        self.assertEqual(params['/grasp_geometry/selected_class'], 'can')
        self.assertTrue(params['/grasp_geometry/servo_controls'])
        self.assertEqual(params['/aubo_visual_servo/max_linear_velocity'], .01)
        self.assertEqual(params['/aubo_visual_servo/target_offset'], [0., 0., .15])
        self.assertTrue(params['/aubo_visual_servo/use_orientation_control'])
        self.assertTrue(params['/aubo_visual_servo/initial_search_enabled'])
        self.assertEqual(params['/aubo_visual_servo/initial_search_posture'],params['/aubo_visual_servo/recovery_posture'])
        self.assertEqual(params['/aubo_visual_servo/loss_strategy'],'coast_then_open')
        reference = json.loads((Path(__file__).resolve().parents[1]/
            'config/visual_servo_eye_to_hand_observation_reference.json').read_text())
        self.assertEqual(params['/aubo_visual_servo/initial_search_posture'], reference['joint_positions'])
        self.assertEqual(params['/aubo_visual_servo/desired_target_rpy'], reference['tcp_rpy'])
        self.assertEqual(params['/eye_to_hand_calibration_tf/child_frame'],
                         params['/aubo_visual_servo/camera_link'])
        self.assertEqual(params['/grasp_geometry/camera_info_topic'], '/camera/color/camera_info')
        self.assertEqual(params['/grasp_geometry/minimum_probability'], .6)
        text = (Path(__file__).resolve().parents[1]/'launch/eye_to_hand_visual_servo_real.launch').read_text()
        self.assertNotIn('$(find aubo_sorting)', text)

    def test_components_can_be_reused_without_loading_another_owner(self):
        config, params = resolve(args=('connect_robot:=false', 'start_camera:=false',
                                      'start_perception:=false', 'rqt:=false', 'rviz:=false',
                                      'record_data:=false'))
        self.assertFalse(any(node.type in ('aubo_visual_servo_node', 'aubo_hw_node',
                                           'visual_servo_panel', 'record', 'rviz') or
                             node.package in ('aubo_perception', 'realsense2_camera')
                             for node in config.nodes))
        self.assertNotIn('/aubo_visual_servo/backend', params)

    def test_device_target_and_logging_arguments_are_forwarded(self):
        config, params = resolve(args=('robot_ip:=192.0.2.8', 'target_class:=bottle',
            'camera_serial_no:=335222076553', 'camera_initial_reset:=true', 'camera_fps:=15',
            'timing_trace_path:=/tmp/unified/timing.csv', 'bag_prefix:=/tmp/unified/servo', 'rviz:=true'))
        self.assertEqual(params['/aubo_visual_servo/robot_ip'], '192.0.2.8')
        self.assertEqual(params['/grasp_geometry/selected_class'], 'bottle')
        self.assertEqual(str(params['/camera/realsense2_camera/serial_no']), '335222076553')
        self.assertTrue(params['/camera/realsense2_camera/initial_reset'])
        self.assertEqual(params['/camera/realsense2_camera/color_fps'], 15)
        self.assertEqual(params['/aubo_visual_servo/timing_trace_path'], '/tmp/unified/timing.csv')
        self.assertTrue(any(node.type == 'rviz' for node in config.nodes))
        self.assertIn('/tmp/unified/servo', next(n.args for n in config.nodes if n.package == 'rosbag'))

    def test_old_scenario_is_a_parameter_compatible_wrapper(self):
        args = ('rviz:=false', 'rqt:=false', 'target_class:=box', 'camera_initial_reset:=true')
        new, new_params = resolve(args=args)
        old, old_params = resolve('aubo_sorting', 'eye_to_hand_yolo_servo_test_real.launch', args)
        self.assertEqual(new_params, old_params)
        signature = lambda config: sorted((n.package, n.type, n.namespace, n.name, n.args) for n in config.nodes)
        self.assertEqual(signature(new), signature(old))

    def test_hybrid_color_caller_keeps_its_frontend_and_queue_profile(self):
        config, params = resolve('aubo_sorting', 'eye_to_hand_hybrid_control_real.launch',
                                 ('target_label:=blue',))
        self.assertTrue(any(n.type == 'rgbd_visual_target_node.py' for n in config.nodes))
        self.assertFalse(any(n.type == 'ultralytics_yolo_node.sh' for n in config.nodes))
        self.assertEqual(params['/rgbd_visual_target/target_label'], 'blue')
        self.assertEqual(params['/rgbd_visual_target/camera_info_topic'], '/camera/color/camera_info')
        self.assertTrue(params['/aubo_visual_servo/hybrid_enabled'])
        self.assertEqual(params['/aubo_visual_servo/sdk_mac_buffer_target'], 60)


if __name__ == '__main__': unittest.main()
