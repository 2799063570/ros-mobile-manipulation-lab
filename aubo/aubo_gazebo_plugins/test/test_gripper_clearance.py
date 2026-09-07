"""Measure actual finger STL clearance over the 40 mm cube's grasp band."""
import math
from pathlib import Path
import struct
import unittest
import xml.etree.ElementTree as ET
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[3]
DESCRIPTION = ROOT / 'aubo/aubo_description'

def rotation(rpy):
    r, p, y = rpy
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return rz @ ry @ rx

def clip(polygon, height, above):
    output = []
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        inside_a = a[2] >= height if above else a[2] <= height
        inside_b = b[2] >= height if above else b[2] <= height
        if inside_a:
            output.append(a)
        if inside_a != inside_b:
            output.append(a + (b-a) * ((height-a[2])/(b[2]-a[2])))
    return output

def finger_band(urdf, name, q, low, high):
    joint = urdf.find("joint[@name='{}']".format(name))
    origin = joint.find('origin')
    offset = np.array([float(x) for x in origin.get('xyz').split()])
    rpy = [float(x) for x in origin.get('rpy').split()]
    # Both fingers in this model rotate about local +Y.
    assert joint.find('axis').get('xyz') == '0 1 0'
    transform = rotation(rpy) @ rotation([0, q, 0])
    link = urdf.find("link[@name='{}']".format(joint.find('child').get('link')))
    collision = link.find('collision')
    assert collision.find('origin').get('xyz') == '0 0 0'
    assert collision.find('origin').get('rpy') == '0 0 0'
    mesh = collision.find('geometry/mesh').get('filename').split('package://aubo_description/')[1]
    data = (DESCRIPTION / mesh).read_bytes()
    count = struct.unpack_from('<I', data, 80)[0]
    points = []
    for index in range(count):
        record = struct.unpack_from('<12fH', data, 84+index*50)
        triangle = np.array(record[3:12]).reshape(3, 3) @ transform.T + offset
        polygon = clip(list(triangle), low, True)
        if polygon:
            points.extend(clip(polygon, high, False))
    return np.array(points)

class ClearanceTest(unittest.TestCase):
    def test_cube_fits_calibrated_gripper(self):
        config = yaml.safe_load((ROOT / 'aubo/aubo_mobile_nav_sorting/config/sorting.yaml').read_text())
        urdf = ET.parse(str(DESCRIPTION / 'urdf/jiazhua.urdf'))
        tcp_z = float(urdf.find("joint[@name='gripper_joint']/origin").get('xyz').split()[2])
        cube_centre = tcp_z + config['grasp_height_offset']
        half = config['object_height']/2
        def gap(q):
            left = finger_band(urdf, 'joint1', q, cube_centre-half, cube_centre+half)
            right = finger_band(urdf, 'joint2', q, cube_centre-half, cube_centre+half)
            return left[:, 1].max(), right[:, 1].min()
        left, right = gap(config['gripper_closed'])
        self.assertLessEqual(left, -half)
        self.assertGreaterEqual(right, half)
        self.assertLess(right-left, config['object_height'] + .006)
        old_left, old_right = gap(.28)
        self.assertLess(old_right-old_left, config['object_height'])
        print('Calibrated gap: {:.2f} mm; previous gap: {:.2f} mm'.format(
            1000*(right-left), 1000*(old_right-old_left)))

if __name__ == '__main__':
    unittest.main()
