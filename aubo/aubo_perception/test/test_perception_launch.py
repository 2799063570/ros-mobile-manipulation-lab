"""Check full launch graphs without starting controllers or Gazebo."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import roslaunch

ROOT = Path(__file__).resolve().parents[1]


def resolve(name, *args):
    config = roslaunch.config.ROSLaunchConfig()
    roslaunch.xmlloader.XmlLoader().load(
        str(ROOT / 'launch' / name), config, argv=list(args), verbose=False)
    return config


class PerceptionLaunchTest(unittest.TestCase):
    def test_four_combinations(self):
        for detector in ('color', 'yolo'):
            for mount in ('eye_in_hand', 'eye_to_hand'):
                with self.subTest(detector=detector, mount=mount):
                    config = resolve(f'{detector}_{mount}_gazebo.launch', 'gui:=false')
                    names = {n.name for n in config.nodes}
                    params = {k: v.value for k, v in config.params.items()}
                    on_arm = mount == 'eye_in_hand'
                    self.assertEqual('move_to_observation' in names, on_arm)
                    self.assertEqual('move_group' in names, on_arm)
                    self.assertEqual('/robot_description' in params, on_arm)
                    self.assertEqual('spawn_workspace_camera' in names, not on_arm)
                    self.assertEqual('ultralytics_yolo' in names, detector == 'yolo')
                    self.assertEqual('color_object_detector' in names, detector == 'color')
                    self.assertNotIn('color_sorting_task', names)
                    self.assertEqual(params['/grasp_geometry/target_frame'], 'base_link' if on_arm else 'world')
                    prefix = '/camera' if on_arm else '/workspace_camera'
                    self.assertEqual(params['/grasp_geometry/aligned_depth_topic'], prefix + '/aligned_depth_to_color/image_raw')
                    if detector == 'yolo':
                        self.assertEqual(params['/ultralytics_yolo/input_mode'], 'topic')
                        self.assertEqual(params['/ultralytics_yolo/image_topic'], prefix + '/color/image_raw')

    def test_overrides(self):
        config = resolve('yolo_eye_to_hand_gazebo.launch', 'camera_z:=1.8', 'model_path:=/tmp/test.pt', 'height_mode:=table')
        self.assertEqual(config.params['/ultralytics_yolo/model_path'].value, '/tmp/test.pt')
        self.assertEqual(config.params['/grasp_geometry/height_mode'].value, 'table')
        nodes = {n.name: n for n in config.nodes}
        self.assertIn('-z 1.8', nodes['spawn_workspace_camera'].args)
        self.assertIn('1.8', nodes['workspace_camera_mount_tf'].args)
        config = resolve('color_eye_in_hand_gazebo.launch', 'auto_move_to_observation:=false')
        self.assertNotIn('move_to_observation', {n.name for n in config.nodes})

    def test_worlds_have_no_arm_plugins_or_extra_cameras(self):
        for detector in ('color', 'yolo'):
            world = ET.parse(ROOT / 'worlds' / f'{detector}_perception.world').getroot().find('world')
            self.assertEqual(world.findall('plugin'), [])
            self.assertEqual(world.findall('.//sensor'), [])
            self.assertIsNotNone(world.find("model[@name='sorting_table']"))
        camera = ET.parse(ROOT / 'models/workspace_camera/model.sdf').getroot()
        self.assertEqual(len(camera.findall('.//sensor')), 1)
        self.assertEqual(camera.findtext('.//plugin/frameName'), 'workspace_camera_color_optical_frame')


if __name__ == '__main__':
    unittest.main()
