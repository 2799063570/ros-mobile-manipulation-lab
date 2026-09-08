"""Check task-to-SRDF contracts without a running robot or MoveIt."""
from pathlib import Path
import math
import unittest
import xml.etree.ElementTree as ET
import yaml
from roslib.packages import get_pkg_dir


class NamedTargetContractTest(unittest.TestCase):
    def test_navigation_sorting_targets_are_complete_srdf_arm_states(self):
        package = Path(__file__).resolve().parents[1]
        config = yaml.safe_load((package / 'config/sorting.yaml').read_text())
        srdf = ET.parse(str(Path(get_pkg_dir('aubo_mobile_moveit_config')) / 'config/aubo_mobile_robot.srdf'))
        group = config['planning_group']
        states = {state.get('name'): state for state in srdf.findall('group_state')
                  if state.get('group') == group}
        arm_joints = {'shoulder_joint', 'upperArm_joint', 'foreArm_joint',
                      'wrist1_joint', 'wrist2_joint', 'wrist3_joint'}
        for key in ('observation_named_target', 'work_ready_named_target', 'finish_named_target'):
            with self.subTest(parameter=key):
                self.assertIn(config[key], states)
                values = {joint.get('name'): float(joint.get('value'))
                          for joint in states[config[key]].findall('joint')}
                self.assertEqual(set(values), arm_joints)
                self.assertTrue(all(math.isfinite(value) for value in values.values()))
        self.assertEqual(config['finish_named_target'], 'transport')


if __name__ == '__main__':
    unittest.main()
