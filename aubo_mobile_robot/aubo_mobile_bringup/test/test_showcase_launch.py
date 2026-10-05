"""Expand real ROS launch chains; never start Gazebo or issue motion commands.

Run through catkin on Linux so xacro, ROS packages and plugin dependencies are
resolved from the actual workspace rather than simulated by a test parser.
"""
from pathlib import Path
import unittest

import roslaunch
import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def resolve(*arguments):
    config = roslaunch.config.ROSLaunchConfig()
    roslaunch.xmlloader.XmlLoader().load(
        str(PACKAGE / 'launch/showcase.launch'), config,
        argv=list(arguments), verbose=False)
    return config


class ShowcaseLaunchTest(unittest.TestCase):
    def test_scenarios_have_one_robot_and_one_matching_rviz(self):
        for scenario in ('perception', 'sorting', 'nav_sorting'):
            with self.subTest(scenario=scenario):
                config = resolve('scenario:=' + scenario)
                rviz_nodes = [node for node in config.nodes if node.type == 'rviz']
                self.assertEqual(len(rviz_nodes), 1)
                layout = PACKAGE / ('rviz/showcase_' + scenario + '.rviz')
                self.assertIn(str(layout), rviz_nodes[0].args)
                self.assertEqual(sum(node.type == 'spawn_model' for node in config.nodes), 1)
                # Load the actual layout: each active image display must subscribe
                # to either the detector output or the scene's correct RGB camera.
                data = yaml.safe_load(layout.read_text(encoding='utf-8'))
                manager = data['Visualization Manager']
                image_topics = {item['Image Topic'] for item in manager['Displays']
                                if item['Class'] == 'rviz/Image'}
                params = {name: param.value for name, param in config.params.items()}
                self.assertIn(params['/color_object_detector/image_topic'], image_topics)
                self.assertIn(params['/color_object_detector/debug_image_topic'], image_topics)
                self.assertEqual(manager['Global Options']['Fixed Frame'],
                                 'map' if scenario == 'nav_sorting' else 'odom')

    def test_perception_never_launches_motion_tasks(self):
        # Even an explicit auto_start must not turn a perception-only demo into
        # a grasping task. Controllers still run to provide joint states/TF.
        config = resolve('scenario:=perception', 'auto_start:=true')
        names = {node.name for node in config.nodes}
        self.assertNotIn('move_group', names)
        self.assertNotIn('color_sorting_task', names)
        self.assertNotIn('nav_sorting_mission', names)
        self.assertEqual(config.params['/color_object_detector/image_topic'].value,
                         '/workspace_camera/color/image_raw')

    def test_tasks_wait_for_operator_by_default(self):
        for scenario in ('sorting', 'nav_sorting'):
            with self.subTest(scenario=scenario):
                config = resolve('scenario:=' + scenario)
                self.assertFalse(config.params['/color_sorting_task/auto_start'].value)
                self.assertFalse(config.params['/color_sorting_task/auto_move_to_observation'].value)
                if scenario == 'nav_sorting':
                    self.assertFalse(config.params['/nav_sorting_mission/auto_start'].value)

    def test_headless_and_auto_start_forwarding(self):
        for scenario in ('sorting', 'nav_sorting'):
            with self.subTest(scenario=scenario):
                config = resolve('scenario:=' + scenario, 'rviz:=false',
                                 'gui:=false', 'auto_start:=true')
                self.assertFalse(any(node.type == 'rviz' for node in config.nodes))
                key = ('/nav_sorting_mission/auto_start' if scenario == 'nav_sorting'
                       else '/color_sorting_task/auto_start')
                self.assertTrue(config.params[key].value)

    def test_unknown_scenario_fails_even_with_custom_layout(self):
        # ROS releases wrap substitution errors in different exception classes;
        # require the error to identify our bad scenario, not an unrelated package.
        with self.assertRaises(Exception) as failure:
            resolve('scenario:=typo', 'rviz_config:=/tmp/custom.rviz')
        self.assertIn('typo', str(failure.exception))


if __name__ == '__main__':
    unittest.main()
