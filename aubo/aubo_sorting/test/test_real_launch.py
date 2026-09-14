"""Check the real launch graph without connecting to hardware."""
from pathlib import Path
import unittest

import roslaunch


ROOT = Path(__file__).resolve().parents[1]


def resolve(name='sorting_real.launch', *args):
    config = roslaunch.config.ROSLaunchConfig()
    roslaunch.xmlloader.XmlLoader().load(
        str(ROOT / 'launch' / name), config,
        argv=list(args), verbose=False)
    return config


class RealLaunchTest(unittest.TestCase):
    def test_default_is_real_and_waits_for_explicit_motion(self):
        config = resolve()
        nodes = {(node.package, node.type) for node in config.nodes}
        params = {key: value.value for key, value in config.params.items()}

        self.assertIn(('aubo_ros_control', 'aubo_hw_node'), nodes)
        self.assertIn(('inspire_gripper', 'inspire_gripper'), nodes)
        self.assertIn(('aubo_sorting_core', 'color_sorting_task_cpp'), nodes)
        self.assertFalse(any(node.package == 'gazebo_ros' for node in config.nodes))
        self.assertEqual(params['/color_sorting_task/gripper_backend'], 'inspire')
        self.assertFalse(params['/color_sorting_task/use_grasp_attachment'])
        self.assertFalse(params['/color_sorting_task/auto_start'])
        self.assertFalse(params['/color_sorting_task/auto_move_to_observation'])
        self.assertEqual(params['/color_sorting_task/velocity_scaling'], 0.1)
        self.assertEqual(params['/color_sorting_task/acceleration_scaling'], 0.1)
        self.assertEqual(params['/grasp_geometry/camera_info_topic'],
                         '/camera/color/camera_info')
        self.assertEqual(params['/color_sorting_task/sort_colors'],
                         ['red', 'green', 'blue'])

    def test_yolo_and_external_drivers(self):
        config = resolve('sorting_real.launch', 'detector:=yolo', 'start_camera:=false',
                         'start_gripper:=false', 'rviz:=false')
        params = {key: value.value for key, value in config.params.items()}
        self.assertEqual(params['/color_sorting_task/sort_colors'], ['can'])
        self.assertEqual(params['/ultralytics_yolo/input_mode'], 'topic')
        self.assertFalse(any(node.package == 'inspire_gripper' for node in config.nodes))
        self.assertFalse(any(node.package == 'realsense2_camera' for node in config.nodes))

    def test_four_d435i_modes_have_matching_topics_and_tf(self):
        for mount, camera in [('eye_in_hand', 'camera'),
                              ('eye_to_hand', 'workspace_camera')]:
            for detector, classes in [('color', ['red', 'green', 'blue']),
                                      ('yolo', ['can'])]:
                name = f'{mount}_{detector}_sorting_real.launch'
                with self.subTest(launch=name):
                    config = resolve(name, 'camera_serial_no:=D435I_TEST_SERIAL')
                    params = {key: value.value for key, value in config.params.items()}
                    node_names = {node.name for node in config.nodes}

                    self.assertEqual(params['/grasp_geometry/camera_mount'], mount)
                    self.assertEqual(params['/grasp_geometry/camera_info_topic'],
                                     f'/{camera}/color/camera_info')
                    self.assertEqual(params['/grasp_geometry/aligned_depth_topic'],
                                     f'/{camera}/aligned_depth_to_color/image_raw')
                    self.assertEqual(params['/color_sorting_task/sort_colors'], classes)
                    self.assertEqual(params[f'/{camera}/realsense2_camera/serial_no'],
                                     'D435I_TEST_SERIAL')
                    for stream in ('enable_color', 'enable_depth', 'align_depth'):
                        self.assertTrue(params[f'/{camera}/realsense2_camera/{stream}'])
                    self.assertFalse(params[f'/{camera}/realsense2_camera/publish_tf'])
                    self.assertEqual('eye_to_hand_calibration_tf' in node_names,
                                     mount == 'eye_to_hand')
                    self.assertEqual('hand_camera_link' in params['/robot_description'],
                                     mount == 'eye_in_hand')
                    for pose in ('observe', 'work_ready', 'down'):
                        self.assertIn(f'group_state name="{pose}"',
                                      params['/robot_description_semantic'])
                    if mount == 'eye_to_hand':
                        self.assertEqual(params['/eye_to_hand_calibration_tf/child_frame'],
                                         'workspace_camera_color_optical_frame')
                    self.assertEqual('ultralytics_yolo' in node_names, detector == 'yolo')


if __name__ == '__main__':
    unittest.main()
