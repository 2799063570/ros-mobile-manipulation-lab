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
    def test_external_yolo_continuously_sorts_the_real_cell(self):
        config = resolve('eye_to_hand_yolo_sorting_real.launch')
        params = {key: value.value for key, value in config.params.items()}
        self.assertTrue(params['/color_sorting_task/continuous_sorting'])
        self.assertEqual(params['/color_sorting_task/sort_classes'], ['bottle', 'can', 'box'])
        self.assertEqual(params['/color_sorting_task/observation_named_target'], 'real_observe')
        self.assertEqual(params['/color_sorting_task/finish_named_target'], 'real_observe')
        self.assertEqual(params['/color_sorting_task/table_center'], [-.0442, -.6707, -.09715])
        self.assertEqual(params['/color_sorting_task/minimum_place_z'], .18)
        self.assertEqual(params['/grasp_geometry/height_mode'], 'depth')
        self.assertEqual(params['/color_sorting_task/target_cache_min_observations'], 8)
        self.assertEqual(params['/color_sorting_task/queue_empty_min_frames'], 5)
        self.assertEqual(params['/color_sorting_task/queue_empty_confirmation'], 2.)
        self.assertLess(params['/color_sorting_task/queue_empty_confirmation'],
                        params['/color_sorting_task/detection_timeout'])
        self.assertGreaterEqual(params['/color_sorting_task/queue_reserved_max_age'],
                                params['/color_sorting_task/queue_max_age'])
        for category in ('bottle', 'can', 'box'):
            self.assertEqual(params['/color_sorting_task/place_targets/'+category],
                             params['/publish_sorting_containers/place_targets/'+category])
            size = params['/publish_sorting_containers/containers/'+category+'/outer_size']
            radius = (size[0]**2+size[1]**2)**.5/2
            self.assertGreater(params['/color_sorting_task/queue_place_exclusion_radius'], radius)
        names = {node.name for node in config.nodes}
        self.assertTrue({'publish_camera_stand', 'real_sorting_table', 'publish_sorting_containers'} <= names)
        self.assertFalse(params['/color_sorting_task/auto_start'])
        self.assertFalse(params['/color_sorting_task/auto_move_to_observation'])

    def test_external_yolo_forwards_driver_settings_when_reusing_hardware(self):
        config = resolve('eye_to_hand_yolo_sorting_real.launch',
                         'start_robot:=false', 'start_gripper:=false', 'rviz:=false',
                         'camera_serial_no:=D435I_TEST_SERIAL',
                         'velocity_scaling:=0.12', 'acceleration_scaling:=0.08')
        params = {key: value.value for key, value in config.params.items()}
        self.assertFalse(any(node.type in ('aubo_hw_node', 'move_group', 'robot_state_publisher')
                             for node in config.nodes))
        self.assertTrue(params['/color_sorting_task/continuous_sorting'])
        self.assertEqual(params['/camera/realsense2_camera/serial_no'], 'D435I_TEST_SERIAL')
        self.assertEqual(params['/color_sorting_task/velocity_scaling'], .12)
        self.assertEqual(params['/color_sorting_task/acceleration_scaling'], .08)

    def test_real_can_launch_accepts_no_arguments(self):
        config = resolve('eye_to_hand_can_sorting_real.launch')
        params = {key: value.value for key, value in config.params.items()}
        self.assertEqual(params['/publish_camera_stand/object_id'], 'camera_stand')
        self.assertEqual(len(params['/publish_camera_stand/parts']), 4)
        self.assertFalse(params['/color_sorting_task/auto_start'])

    def test_real_can_reuses_hardware_and_keeps_observation_and_obstacles(self):
        config = resolve('eye_to_hand_can_sorting_real.launch',
                         'start_robot:=false', 'start_camera:=false',
                         'start_gripper:=false', 'rviz:=false')
        params = {key: value.value for key, value in config.params.items()}
        self.assertFalse(any(node.type in ('aubo_hw_node', 'move_group', 'robot_state_publisher')
                             for node in config.nodes))
        self.assertEqual(params['/grasp_geometry/camera_info_topic'], '/camera/color/camera_info')
        self.assertEqual(params['/color_sorting_task/observation_named_target'], 'real_observe')
        self.assertEqual(params['/color_sorting_task/finish_named_target'], 'real_observe')
        self.assertEqual(params['/color_sorting_task/place_targets/can'], [-.42, -.3375])
        self.assertEqual(params['/color_sorting_task/sort_classes'], ['bottle', 'can', 'box'])
        self.assertEqual(params['/color_sorting_task/minimum_place_z'], .18)
        for category in ('bottle', 'can', 'box'):
            self.assertEqual(params['/color_sorting_task/place_targets/'+category],
                             params['/publish_sorting_containers/place_targets/'+category])
        self.assertTrue(any(node.type == 'publish_sorting_containers.py' for node in config.nodes))
        self.assertFalse(params['/color_sorting_task/continuous_sorting'])
        self.assertEqual(params['/color_sorting_task/minimum_cartesian_fraction'], 1.0)
        self.assertEqual(params['/real_sorting_table/object_id'], 'sorting_table')
        self.assertEqual(len(params['/publish_camera_stand/parts']), 4)
        table = params['/real_sorting_table/parts'][0]
        center = params['/color_sorting_task/table_center']
        size = params['/color_sorting_task/table_size']
        margin = params['/color_sorting_task/table_collision_margin']
        self.assertEqual(table['center'], center)
        for actual, expected in zip(table['size'], [size[0]+2*margin, size[1]+2*margin, size[2]]):
            self.assertAlmostEqual(actual, expected)
        self.assertAlmostEqual(center[2]+size[2]/2, params['/color_sorting_task/table_z'])

    def test_sorting_box_is_optional_and_passes_through_yolo_entry(self):
        default = resolve('sorting_real.launch', 'rviz:=false')
        self.assertFalse(any(node.type == 'publish_sorting_box.py' for node in default.nodes))
        configured = resolve('eye_to_hand_yolo_sorting_real.launch', 'rviz:=false',
                             'publish_sorting_box:=true', 'start_robot:=false')
        self.assertTrue(any(node.package == 'aubo_planning' and node.type == 'publish_sorting_box.py'
                            for node in configured.nodes))
        params = {key: value.value for key, value in configured.params.items()}
        self.assertEqual(params['/publish_sorting_box/object_id'], 'sorting_box')
        self.assertEqual(params['/publish_sorting_box/outer_size'], [.30, .24, .12])
        self.assertFalse(params['/color_sorting_task/auto_start'])

    def test_default_is_real_and_waits_for_explicit_motion(self):
        config = resolve()
        nodes = {(node.package, node.type) for node in config.nodes}
        params = {key: value.value for key, value in config.params.items()}

        self.assertIn(('aubo_ros_control', 'aubo_hw_node'), nodes)
        self.assertIn(('inspire_gripper', 'inspire_gripper'), nodes)
        self.assertIn(('aubo_sorting_core', 'sorting_task_cpp'), nodes)
        self.assertFalse(any(node.package == 'gazebo_ros' for node in config.nodes))
        self.assertEqual(params['/color_sorting_task/gripper_backend'], 'inspire')
        self.assertFalse(params['/color_sorting_task/use_grasp_attachment'])
        self.assertFalse(params['/color_sorting_task/auto_start'])
        self.assertFalse(params['/color_sorting_task/auto_move_to_observation'])
        self.assertEqual(params['/color_sorting_task/velocity_scaling'], 0.1)
        self.assertEqual(params['/color_sorting_task/acceleration_scaling'], 0.1)
        self.assertEqual(params['/grasp_geometry/camera_info_topic'],
                         '/camera/color/camera_info')
        self.assertEqual(params['/color_sorting_task/sort_classes'],
                         ['red', 'green', 'blue'])

    def test_yolo_and_external_drivers(self):
        config = resolve('sorting_real.launch', 'detector:=yolo', 'start_camera:=false',
                         'start_gripper:=false', 'rviz:=false')
        params = {key: value.value for key, value in config.params.items()}
        self.assertEqual(params['/color_sorting_task/sort_classes'], ['can'])
        self.assertEqual(params['/ultralytics_yolo/input_mode'], 'topic')
        self.assertFalse(any(node.package == 'inspire_gripper' for node in config.nodes))
        self.assertFalse(any(node.package == 'realsense2_camera' for node in config.nodes))

    def test_four_d435i_modes_have_matching_topics_and_tf(self):
        for mount, camera in [('eye_in_hand', 'camera'),
                              ('eye_to_hand', 'camera')]:
            for detector, classes in [('color', ['red', 'green', 'blue']),
                                      ('yolo', ['bottle', 'can', 'box'] if mount == 'eye_to_hand' else ['can'])]:
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
                    self.assertEqual(params['/color_sorting_task/sort_classes'], classes)
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
                                         'camera_color_optical_frame')
                    self.assertEqual('ultralytics_yolo' in node_names, detector == 'yolo')


if __name__ == '__main__':
    unittest.main()
