"""Check the specified category bins, open interiors and shared destinations."""
import copy
import importlib.util
from pathlib import Path
import sys
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
SPEC = importlib.util.spec_from_file_location('containers', ROOT/'scripts/publish_sorting_containers.py')
module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(module)


class SortingContainersTest(unittest.TestCase):
    def setUp(self):
        self.config = yaml.safe_load((ROOT/'config/sorting_containers_real.yaml').read_text())

    def test_specified_centers_sizes_and_open_interiors(self):
        frame, objects = module.container_geometry(self.config)
        self.assertEqual(frame, 'base_link')
        self.assertEqual(len(objects), 3)
        expected = {'bottle': ([-.4, -.18], [.27, .15, .18]),
                    'can': ([-.42, -.3375], [.20, .165, .16]),
                    'box': ([-.42, -.5425], [.20, .245, .16])}
        for category, (xy, size) in expected.items():
            self.assertEqual(self.config['place_targets'][category], xy)
            self.assertEqual(self.config['containers'][category]['outer_size'], size)
            parts = dict(objects)['sorting_container_'+category]
            self.assertEqual(len(parts), 5)
            self.assertEqual(parts[0]['center'][:2], xy)
            point = [*xy, self.config['bottom_z']+size[2]/2]
            self.assertFalse(any(all(abs(p-c) <= s/2 for p, c, s in
                                     zip(point, part['center'], part['size'])) for part in parts))
            self.assertAlmostEqual(max(p['center'][2]+p['size'][2]/2 for p in parts),
                                   self.config['bottom_z']+size[2])
        self.assertEqual(self.config['minimum_place_z'], .18)

    def test_adjacent_y_boundaries_follow_the_supplied_formulas(self):
        self.assertAlmostEqual(-.18-(.15/2+.165/2), self.config['place_targets']['can'][1])
        self.assertAlmostEqual(-.18-(.15/2+.165+.245/2), self.config['place_targets']['box'][1])
        self.assertAlmostEqual(-.18-.15/2, -.3375+.165/2)
        self.assertAlmostEqual(-.3375-.165/2, -.5425+.245/2)

    def test_editing_destination_moves_the_collision_container(self):
        self.config['place_targets']['can'] = [.12, -.44]
        _, objects = module.container_geometry(self.config)
        parts = dict(objects)['sorting_container_can']
        self.assertEqual(parts[0]['center'][:2], [.12, -.44])

    def test_invalid_or_inconsistent_geometry_is_rejected(self):
        mutations = [lambda c: c['place_targets'].pop('can'),
                     lambda c: c['place_targets'].update(can=[0]),
                     lambda c: c['place_targets'].update(can=[float('nan'), 0]),
                     lambda c: c.update(minimum_place_z=float('inf')),
                     lambda c: c.update(wall_thickness=.20),
                     lambda c: c['containers']['can'].update(object_id='sorting_container_bottle')]
        for mutate in mutations:
            config = copy.deepcopy(self.config)
            mutate(config)
            with self.assertRaises(ValueError):
                module.container_geometry(config)


if __name__ == '__main__':
    unittest.main()
