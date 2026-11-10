import json
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from aubo_cloud_bridge.sorting_telemetry import SortingTelemetry


class SortingTelemetryTests(unittest.TestCase):
    def test_completion_snapshots_replace_without_summing_and_queue_clear_keeps_counts(self):
        telemetry = SortingTelemetry()
        self.assertFalse(telemetry.snapshot()['statistics_available'])
        stats = {'schema_version': 1, 'scope': 'node_session', 'session': 'first',
                 'counts': {'bottle': 1, 'can': 2, 'box': 3}, 'total': 6}
        for _ in range(2):
            telemetry.update('completed_counts', json.dumps(stats))
        telemetry.update('target_cache', '{"schema_version":2,"targets":{}}')
        self.assertEqual(telemetry.snapshot()['completed_total'], 6)
        for invalid in ({**stats, 'total': 7}, {**stats, 'counts': {'can': True}},
                        {**stats, 'scope': 'lifetime'}, {**stats, 'session': ''}):
            with self.assertRaises(ValueError):
                telemetry.update('completed_counts', json.dumps(invalid))
            self.assertEqual(telemetry.snapshot()['completed_total'], 6)
        stats.update(session='restarted', counts={'bottle': 0, 'can': 0, 'box': 0}, total=0)
        telemetry.update('completed_counts', json.dumps(stats))
        snapshot = telemetry.snapshot()
        self.assertTrue(snapshot['statistics_available'])
        self.assertEqual(snapshot['completed_total'], 0)
        self.assertEqual(json.loads(snapshot['completed_counts'])['session'], 'restarted')

    def test_projection_keeps_zero_coordinates_and_never_includes_images(self):
        telemetry = SortingTelemetry()
        telemetry.set_workcell([.7, 0, .1], [.7, 1, .05], 'base_link')
        target = {'id': 6, 'category': 'bottle', 'status': 'ready', 'position': [.54, 0, .12],
                  'grasp_angle': 0, 'grasp_width': .04, 'grasp_geometry_valid': True,
                  'observations': 12, 'age': .4, 'image': 'never-upload', 'point_cloud': 'never-upload'}
        telemetry.update('target_cache', json.dumps({'schema_version': 2, 'targets': {'6': target}}), now=10)
        snapshot = telemetry.snapshot(now=11)
        self.assertEqual(json.loads(snapshot['workcell'])['table_center'], [.7, 0])
        row = json.loads(snapshot['target_cache'])['targets'][0]
        self.assertEqual(row['position'], [.54, 0, .12])
        self.assertEqual(row['grasp_angle'], 0)
        self.assertNotIn('never-upload', snapshot['target_cache'])
        self.assertEqual(snapshot['target_cache_age_s'], 1)
        target['position'] = [None, 0, .12]
        telemetry.update('target_cache', json.dumps({'schema_version': 2, 'targets': {'6': target}}))
        self.assertNotIn('position', json.loads(telemetry.snapshot()['target_cache'])['targets'][0])
        telemetry.set_workcell(None, None, '')
        self.assertEqual(telemetry.snapshot()['workcell'], '{}')
