"""Resolve the real include chain without starting Gazebo or moving the robot."""
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import roslaunch


ROOT = Path(__file__).resolve().parents[1]


def resolve(name, args=()):
    config = roslaunch.config.ROSLaunchConfig()
    roslaunch.xmlloader.XmlLoader().load(
        str(ROOT / 'launch' / name), config, argv=list(args), verbose=False)
    return config


class GazeboLaunchTest(unittest.TestCase):
    def test_yolo_uses_live_camera_and_lying_can(self):
        for name, args in [('yolo_sorting_gazebo.launch', []),
                           ('sorting_gazebo.launch', ['detector:=yolo'])]:
            with self.subTest(launch=name):
                config = resolve(name, args)
                params = {key: value.value for key, value in config.params.items()}
                self.assertEqual(params['/ultralytics_yolo/input_mode'], 'topic')
                self.assertEqual(params['/ultralytics_yolo/image_path'], '')
                self.assertEqual(params['/ultralytics_yolo/image_topic'], '/camera/color/image_raw')
                self.assertEqual(params['/ultralytics_yolo/detections_topic'],
                                 params['/grasp_geometry/boxes_topic'])
                self.assertEqual(params['/grasp_geometry/detections_topic'],
                                 params['/color_sorting_task/detections_topic'])
                self.assertTrue(params['/color_sorting_task/use_detected_angle'])
                self.assertEqual(params['/color_sorting_task/sort_colors'], ['can'])
                world = ET.parse(ROOT / 'worlds/yolo_sorting.world').getroot().find('world')
                model = world.find("model[@name='beverage_can']")
                pose = [float(v) for v in model.findtext('pose').split()]
                radius = float(model.findtext('link/collision/geometry/cylinder/radius'))
                self.assertAlmostEqual(pose[4], math.pi / 2)
                self.assertAlmostEqual(pose[2] - radius, params['/grasp_geometry/table_z'])
                for node in ['grasp_geometry', 'color_sorting_task']:
                    self.assertAlmostEqual(params['/' + node + '/object_height'], radius * 2)
                self.assertEqual(params['/color_sorting_task/grasp_model_names/can'], model.get('name'))
                self.assertEqual(world.findtext('plugin/object_link'), model.find('link').get('name'))
                gazebo = next(n for n in config.nodes if n.name == 'gazebo')
                self.assertIn('/worlds/yolo_sorting.world', gazebo.args)
                self.assertFalse(any(n.name == 'spawn_sorting_pads' for n in config.nodes))

    def test_color_keeps_blocks(self):
        config = resolve('sorting_gazebo.launch')
        self.assertFalse(any(n.name == 'ultralytics_yolo' for n in config.nodes))
        self.assertEqual(config.params['/color_sorting_task/sort_colors'].value, ['red', 'green', 'blue'])
        self.assertEqual(config.params['/grasp_geometry/object_height'].value, 0.04)
        self.assertIn('/worlds/sorting.world', next(n.args for n in config.nodes if n.name == 'gazebo'))

    def test_wrapper_forwards_overrides(self):
        config = resolve('yolo_sorting_gazebo.launch', ['gui:=false', 'object_height:=0.06'])
        for node in ['grasp_geometry', 'color_sorting_task']:
            self.assertEqual(config.params['/' + node + '/object_height'].value, 0.06)
        self.assertEqual(config.params['/ultralytics_yolo/input_mode'].value, 'topic')


if __name__ == '__main__':
    unittest.main()
