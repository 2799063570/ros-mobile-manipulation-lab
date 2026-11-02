"""Check the real-cell standby geometry without ROS master or robot commands."""
import json
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import xacro
from tf.transformations import euler_matrix, rotation_matrix

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = Path(__file__).resolve().parents[1]


class RealObservationPoseTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.reference = json.loads((PACKAGE / 'config/real_sort_ready_reference.json').read_text())
        cls.capture = json.loads((PACKAGE / 'config/real_observe_capture.json').read_text())
        model = ROOT / 'aubo_description/urdf/aubo_i5.xacro'
        cls.urdf = ET.fromstring(xacro.process_file(str(model)).toxml())
        children = {j.find('child').get('link'): j for j in cls.urdf.findall('joint')}
        cls.chain = []
        link = 'tcp_link'
        while link != 'base_link':
            joint = children[link]
            cls.chain.append(joint)
            link = joint.find('parent').get('link')
        cls.chain.reverse()

    def fk(self, positions):
        values = dict(zip(self.reference['joint_names'], positions))
        transform = np.eye(4)
        for joint in self.chain:
            origin = joint.find('origin')
            xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()] if origin is not None else [0, 0, 0]
            rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()] if origin is not None else [0, 0, 0]
            offset = euler_matrix(*rpy)
            offset[:3, 3] = xyz
            transform = transform @ offset
            if joint.get('type') in ('revolute', 'continuous'):
                value = values[joint.get('name')]
                limit = joint.find('limit')
                self.assertGreaterEqual(value, float(limit.get('lower')))
                self.assertLessEqual(value, float(limit.get('upper')))
                axis = [float(v) for v in joint.find('axis').get('xyz').split()]
                transform = transform @ rotation_matrix(value, axis)
        return transform

    def test_both_models_share_the_standby_and_preserve_the_measured_finish(self):
        for name in ('aubo_i5.srdf', 'aubo_i5_with_camera.srdf'):
            with self.subTest(model=name):
                srdf = ET.parse(str(ROOT / 'aubo_moveit_config/config' / name))
                for pose, values in [('real_sort_ready', self.reference['positions']),
                                     ('real_observe', self.capture['positions'])]:
                    states = srdf.findall("group_state[@name='{}'][@group='aubo_i5']".format(pose))
                    self.assertEqual(len(states), 1)
                    joints = {j.get('name'): float(j.get('value')) for j in states[0].findall('joint')}
                    self.assertEqual(set(joints), set(self.reference['joint_names']))
                    np.testing.assert_allclose([joints[n] for n in self.reference['joint_names']], values,
                                               atol=1e-11, rtol=0)

    def test_standby_tcp_matches_the_reference_and_faces_down(self):
        transform = self.fk(self.reference['positions'])
        np.testing.assert_allclose(transform[:3, 3], [-.25, -.45, .25285], atol=1e-8, rtol=0)
        np.testing.assert_allclose(transform[:3, :3], euler_matrix(math.pi, 0, math.pi/2)[:3, :3],
                                   atol=1e-8, rtol=0)
        self.assertLess(abs(self.reference['positions'][3]), 1.2)
        self.assertFalse(self.reference['measured_on_robot'])
        self.assertFalse(self.reference['live_execution_verified'])

    def test_same_orientation_first_target_requires_less_joint_travel(self):
        target = self.reference['reference_pregrasp']
        goal = np.array(target['positions'])
        np.testing.assert_allclose(self.fk(goal)[:3, 3], target['position'], atol=1e-8, rtol=0)
        np.testing.assert_allclose(self.fk(goal)[:3, :3], self.fk(self.reference['positions'])[:3, :3],
                                   atol=1e-8, rtol=0)
        old = np.array(self.capture['positions'])
        new = np.array(self.reference['positions'])
        self.assertLess(max(abs(goal-new)), .4 * max(abs(goal-old)))
        self.assertLess(np.linalg.norm(self.fk(new)[:3, 3]-target['position']),
                        np.linalg.norm(self.fk(old)[:3, 3]-target['position']))


if __name__ == '__main__':
    unittest.main()
