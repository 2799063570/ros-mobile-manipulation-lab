"""Inspect the real servo launch graph without starting any device."""
import unittest

import roslaunch
import roslaunch.rlutil


def resolve(*args):
    path = roslaunch.rlutil.resolve_launch_arguments(
        ['aubo_sorting', 'eye_to_hand_yolo_servo_test_real.launch'])[0]
    config = roslaunch.config.ROSLaunchConfig()
    roslaunch.xmlloader.XmlLoader().load(path, config, argv=list(args), verbose=False)
    return config, {key: param.value for key, param in config.params.items()}


class RealServoLaunchTest(unittest.TestCase):
    def test_single_sdk_owner_with_explicit_enable_and_real_camera_frames(self):
        config, params = resolve('rviz:=false')
        nodes = {(n.package, n.type) for n in config.nodes}
        self.assertIn(('aubo_ros_control', 'aubo_visual_servo_node'), nodes)
        self.assertNotIn(('aubo_ros_control', 'aubo_hw_node'), nodes)
        self.assertFalse(any(n.package in ('inspire_gripper', 'aubo_sorting_core', 'gazebo_ros')
                             for n in config.nodes))
        self.assertFalse(params['/move_group/allow_trajectory_execution'])
        self.assertFalse(params['/aubo_visual_servo/start_enabled'])
        self.assertFalse(params['/aubo_visual_servo/hybrid_enabled'])
        self.assertTrue(params['/aubo_visual_servo/on_demand_output'])
        self.assertEqual(params['/aubo_visual_servo/max_linear_velocity'], .01)
        self.assertTrue(params['/aubo_visual_servo/use_orientation_control'])
        self.assertEqual(params['/aubo_visual_servo/target_offset'], [0., 0., .15])
        self.assertEqual(params['/aubo_visual_servo/camera_link'], 'camera_color_optical_frame')
        self.assertEqual(params['/eye_to_hand_calibration_tf/child_frame'], 'camera_color_optical_frame')
        self.assertEqual(params['/grasp_geometry/camera_info_topic'], '/camera/color/camera_info')
        self.assertEqual(params['/grasp_geometry/task_mode'], 'servo')
        self.assertEqual(params['/grasp_geometry/selected_class'], 'can')
        self.assertTrue(params['/grasp_geometry/servo_controls'])
        self.assertTrue(any(n.type == 'visual_servo_panel' for n in config.nodes))
        self.assertEqual(params['/ultralytics_yolo/annotated_image_topic'], '/visual_servo/debug_image')
        self.assertEqual(params['/aubo_visual_servo/planning_scene_topic'],
                         '/move_group/monitored_planning_scene')
        self.assertEqual(params['/aubo_visual_servo/initial_scene_service'], '/get_planning_scene')
        names = {n.name for n in config.nodes}
        self.assertTrue({'real_servo_table', 'publish_camera_stand', 'publish_sorting_containers'} <= names)
        self.assertTrue(any(n.package == 'rosbag' for n in config.nodes))

    def test_perception_only_does_not_load_any_robot_control_backend(self):
        config, params = resolve('connect_robot:=false', 'rviz:=false', 'record_data:=false')
        self.assertFalse(any(n.type in ('aubo_visual_servo_node', 'aubo_hw_node', 'record')
                             for n in config.nodes))
        self.assertNotIn('/aubo_visual_servo/backend', params)
        self.assertEqual(params['/grasp_geometry/task_mode'], 'servo')

    def test_custom_target_and_recording_paths_are_forwarded(self):
        config, params = resolve('target_class:=bottle', 'rviz:=false', 'start_camera:=false',
                                 'timing_trace_path:=/tmp/test_run/timing.csv',
                                 'bag_prefix:=/tmp/test_run/servo')
        self.assertFalse(any(n.package == 'realsense2_camera' for n in config.nodes))
        self.assertEqual(params['/grasp_geometry/selected_class'], 'bottle')
        self.assertEqual(params['/aubo_visual_servo/timing_trace_path'], '/tmp/test_run/timing.csv')
        recording = next(n for n in config.nodes if n.package == 'rosbag')
        self.assertIn('/tmp/test_run/servo', recording.args)
        self.assertIn('/visual_servo/sdk_joint_states', recording.args)

    def test_camera_recovery_options_are_explicit_and_forwarded(self):
        _, params = resolve('connect_robot:=false', 'rviz:=false',
                            'camera_serial_no:=335222076553',
                            'camera_initial_reset:=true', 'camera_fps:=15')
        prefix = '/camera/realsense2_camera/'
        self.assertEqual(str(params[prefix + 'serial_no']), '335222076553')
        self.assertTrue(params[prefix + 'initial_reset'])
        self.assertEqual(params[prefix + 'color_fps'], 15)
        self.assertEqual(params[prefix + 'depth_fps'], 15)
        self.assertFalse(params[prefix + 'enable_pointcloud'])
        self.assertFalse(params[prefix + 'enable_gyro'])
        self.assertFalse(params[prefix + 'enable_accel'])


if __name__ == '__main__':
    unittest.main()
