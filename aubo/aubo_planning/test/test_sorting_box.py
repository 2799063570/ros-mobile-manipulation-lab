"""Open-container geometry and planning-scene publication without robot hardware."""
import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest import mock
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('publish_sorting_box', ROOT/'scripts/publish_sorting_box.py')
box = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(box)


def contains(part, point):
    return all(abs(p-c) <= s/2+1e-12 for p, c, s in zip(point, part['center'], part['size']))


class SortingBoxTest(unittest.TestCase):
    def test_open_interior_and_bottom_and_walls(self):
        parts = box.box_parts([.30, .24, .12], [.6, -.3, .1], .01, .008)
        self.assertEqual(len(parts), 5)
        self.assertFalse(any(contains(p, [.6, -.3, .16]) for p in parts))
        self.assertFalse(any(contains(p, [.6, -.3, .22]) for p in parts))
        self.assertTrue(any(contains(p, [.6, -.3, .104]) for p in parts))
        for point in ([.455, -.3, .16], [.745, -.3, .16],
                      [.6, -.415, .16], [.6, -.185, .16]):
            self.assertTrue(any(contains(p, point) for p in parts), point)
        self.assertAlmostEqual(max(p['center'][2]+p['size'][2]/2 for p in parts), .22)
        volume = sum(math.prod(p['size']) for p in parts)
        self.assertAlmostEqual(volume, .30*.24*.12 - .28*.22*.112)

    def test_yaw_rotates_all_parts_about_bottom_center(self):
        straight = box.box_parts([.30, .24, .12], [1., 2., .1], .01, .008)
        rotated = box.box_parts([.30, .24, .12], [1., 2., .1], .01, .008, math.pi/2)
        for a, b in zip(straight, rotated):
            self.assertAlmostEqual(b['center'][0], 1-(a['center'][1]-2))
            self.assertAlmostEqual(b['center'][1], 2+(a['center'][0]-1))
            self.assertAlmostEqual(b['center'][2], a['center'][2])
            self.assertAlmostEqual(b['quaternion'][2], math.sqrt(.5))
            self.assertAlmostEqual(b['quaternion'][3], math.sqrt(.5))

    def test_invalid_dimensions_cannot_create_a_container(self):
        cases = [([0., .2, .1], [0, 0, 0], .01, .008, 0.),
                 ([.3, .2, .1], [0, 0, float('nan')], .01, .008, 0.),
                 ([.3, .2, .1], [0, 0, 0], .10, .008, 0.),
                 ([.3, .2, .1], [0, 0, 0], .01, .1, 0.),
                 ([.3, .2, .1], [0, 0, 0], -.01, .008, 0.),
                 ([.3, .2, .1], [0, 0, 0], .01, .008, float('inf'))]
        for values in cases:
            with self.assertRaises(ValueError):
                box.box_parts(*values)

    def fake_modules(self, outcome='valid'):
        class CollisionObject:
            ADD = 0

            def __init__(self):
                self.header = NS(frame_id='')
                self.primitives, self.primitive_poses = [], []

        class SolidPrimitive:
            BOX = 1

        def pose():
            return NS(position=NS(x=0., y=0., z=0.), orientation=NS(x=0., y=0., z=0., w=0.))

        def scene():
            return NS(is_diff=False, robot_state=NS(is_diff=False), world=NS(collision_objects=[]))

        publishers, applied = [], []
        params = dict(frame_id='base_link', object_id='sorting_box', outer_size=[.3, .24, .12],
                      bottom_center=[.6, -.3, .1], wall_thickness=.01, bottom_thickness=.008, yaw=0.)

        def apply(request):
            applied.append(request.scene.world.collision_objects[0])
            return NS(success=outcome != 'apply_rejected')

        def get_scene(_query):
            objects = applied if outcome != 'missing_object' else []
            return NS(scene=NS(world=NS(collision_objects=objects)))

        def publisher(*_args, **_kwargs):
            result = mock.Mock()
            publishers.append(result)
            return result

        rospy = NS(init_node=mock.Mock(), Publisher=publisher, get_param=lambda k, *d: params.get(k[1:], *d),
                   wait_for_service=mock.Mock(), ServiceProxy=lambda name, _: apply if name == '/apply_planning_scene' else get_scene,
                   spin=mock.Mock(), loginfo=mock.Mock(), logfatal=mock.Mock(),
                   ROSException=type('ROSException', (Exception,), {}),
                   ServiceException=type('ServiceException', (Exception,), {}))
        modules = {'rospy': rospy, 'geometry_msgs': NS(), 'geometry_msgs.msg': NS(Pose=pose),
                   'moveit_msgs': NS(), 'moveit_msgs.msg': NS(CollisionObject=CollisionObject, PlanningScene=scene,
                                                            PlanningSceneComponents=NS(WORLD_OBJECT_GEOMETRY=16)),
                   'moveit_msgs.srv': NS(ApplyPlanningScene=object, ApplyPlanningSceneRequest=lambda: NS(scene=None),
                                         GetPlanningScene=object, GetPlanningSceneRequest=lambda: NS(components=NS(components=0))),
                   'shape_msgs': NS(), 'shape_msgs.msg': NS(SolidPrimitive=SolidPrimitive),
                   'std_msgs': NS(), 'std_msgs.msg': NS(Bool=object)}
        return modules, publishers, applied

    def test_publication_is_real_collision_geometry_and_confirms_scene_before_ready(self):
        modules, publishers, applied = self.fake_modules()
        with mock.patch.dict(sys.modules, modules):
            box.main()
        self.assertEqual(len(applied), 1)
        self.assertEqual(applied[0].header.frame_id, 'base_link')
        self.assertEqual(applied[0].id, 'sorting_box')
        self.assertEqual(len(applied[0].primitives), 5)
        self.assertTrue(all(p.type == 1 for p in applied[0].primitives))
        self.assertEqual(publishers[0].publish.call_args_list, [mock.call(False), mock.call(True)])
        modules['rospy'].spin.assert_called_once()

    def test_failed_apply_or_missing_scene_object_never_reports_ready(self):
        for outcome in ('apply_rejected', 'missing_object'):
            modules, publishers, _ = self.fake_modules(outcome)
            with mock.patch.dict(sys.modules, modules), self.assertRaises(RuntimeError):
                box.main()
            self.assertFalse(any(c.args == (True,) for c in publishers[0].publish.call_args_list))
            modules['rospy'].spin.assert_not_called()

    def test_bringup_and_real_sorting_keep_reference_disabled_and_forward_configuration(self):
        bringup = ET.parse(ROOT.parent/'aubo_ros_control/launch/aubo_real_bringup.launch')
        sorting = ET.parse(ROOT.parent/'aubo_sorting/launch/sorting_real.launch')
        for tree in (bringup, sorting):
            args = {a.get('name'): a.get('default') for a in tree.findall('arg')}
            self.assertEqual(args['publish_sorting_box'], 'false')
            self.assertIn('sorting_box_reference.yaml', args['sorting_box_config'])
        include = next(i for i in bringup.findall('include') if 'publish_sorting_box.launch' in i.get('file', ''))
        self.assertEqual(include.get('if'), '$(arg publish_sorting_box)')
        real = next(i for i in sorting.findall('include') if 'publish_sorting_box.launch' in i.get('file', ''))
        self.assertEqual(real.get('if'), '$(arg publish_sorting_box)')
        forwarded = {a.get('name'): a.get('value') for a in real.findall('arg')}
        self.assertEqual(forwarded['config'], '$(arg sorting_box_config)')


if __name__ == '__main__':
    unittest.main()
