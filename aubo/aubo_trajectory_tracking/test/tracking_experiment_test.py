#!/usr/bin/env python3
"""Storage/statistical regressions; no ROS, robot, or plotting required."""
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from tracking_experiment_data import CaptureSession, analyze_run, compare_runs, normalize_metadata


def event(kind, data, t=0., start=10.):
    return {'kind': kind, 'data': data, 'ros_s': start+t, 'steady_s': start+t}


def frozen(run='10000000000', kp=2., origin=None):
    return dict(schema_version='1', run_start_ns=run,
                parameters=dict(shape='circle', plane='xy', radius='0.12', duration='2',
                                kp=str(kp), ki='0', kd='0', acceleration_ff_time='0.1'),
                origin_m=origin or ['0', '0', '0'], start_orientation_xyzw=['0', '0', '0', '1'])


def tcp(t, error=.001, settling=0):
    # A point on the circle at its start, with a signed outward radial offset.
    return [t, 0., 0., 0., error, 0., 0., error, .01, .01, .01, .01,
            .005, settling, 0., error]


class ExperimentDataTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.params = {'kp': 9., 'duration': 2.}
        self.session = CaptureSession(self.root, lambda: {'parameters': self.params.copy(),
                                      'environment': {'backend': 'fixture'}}, tail=.2)

    def tearDown(self):
        self.temp.cleanup()

    def begin(self, run='10000000000', metadata=True, start=10., kp=2.):
        if metadata:
            self.session.consume(event('metadata', frozen(run, kp), start=start))
        self.session.consume(event('start', run, start=start))
        self.session.consume(event('state', 'RUNNING: fixture', start=start))

    def fill(self, start=10.):
        for i in range(100):
            t=i*.02
            self.session.consume(event('tcp', tcp(t), t, start))
            self.session.consume(event('joints', [t]+[0.]*33, t, start))
            self.session.consume(event('servo', [t]+[0.]*11+[1., 2.], t, start))
        self.session.consume(event('tcp', tcp(2., .01, 1), 2., start))

    def end(self, state='COMPLETED: fixture', start=10.):
        self.session.consume(event('state', state, 2.01, start))
        self.session.consume(event('state', state, 2.1, start))
        self.session.tick(start+2.22)
        return self.session.closed[-1]

    def test_frozen_parameters_survive_server_edits_and_phase_statistics(self):
        self.begin(); self.params['kp'] = 100.; self.fill()
        result = self.end()
        self.assertEqual(result['parameters']['kp'], 2.)
        self.assertAlmostEqual(result['tcp']['running']['total_mm']['rms'], 1.)
        self.assertAlmostEqual(result['tcp']['all']['total_mm']['rms'], math.sqrt(200/101))
        self.assertAlmostEqual(result['tcp']['running']['radial_mm']['rms'], 1.)
        self.assertTrue(result['eligible_for_aggregate'], result['quality'])
        self.assertEqual(result['feedback_age_ms']['peak_abs'], 5.)

    def test_repeated_terminal_state_does_not_extend_tail(self):
        self.begin(); self.fill(); self.end()
        self.assertIsNone(self.session.active)

    def test_stop_after_completion_does_not_relabel_completed_trial(self):
        self.begin(); self.fill()
        self.session.consume(event('state', 'COMPLETED: fixture', 2.01))
        self.session.consume(event('state', 'STOPPED: next preparation', 2.1))
        self.session.tick(12.22)
        self.assertEqual(self.session.closed[-1]['state'], 'COMPLETED: fixture')

    def test_metadata_arriving_after_header_is_associated_by_exact_stamp(self):
        self.begin(metadata=False)
        self.session.consume(event('metadata', frozen()))
        self.fill(); result=self.end()
        self.assertEqual(result['parameter_source'], 'frozen')

    def test_old_latched_completed_run_is_partial_and_not_reopened(self):
        self.session.consume(event('start', '10000000000', t=20))
        self.session.consume(event('state', 'COMPLETED: old', t=20))
        self.session.tick(31.)
        self.assertIn('collector_joined_late', self.session.closed[0]['quality'])
        self.assertIsNone(self.session.closed[0]['tcp']['all']['total_mm']['rms'])
        self.session.consume(event('start', '10000000000', t=21))
        self.assertIsNone(self.session.active)

    def test_nonfinite_duplicate_and_old_run_sample_are_counted(self):
        self.begin()
        self.session.consume(event('tcp', tcp(0.)))
        self.session.consume(event('tcp', tcp(0.)))
        self.session.consume(event('tcp', tcp(40.)))
        self.session.consume(event('tcp', tcp(.01, float('nan')), .01))
        self.session.finish('INTERRUPTED: fixture', dropped=2)
        result=self.session.closed[0]
        self.assertIn('rejected_samples', result['quality'])
        self.assertIn('collector_queue_overflow', result['quality'])
        self.assertFalse(result['eligible_for_aggregate'])

    def test_preview_does_not_delete_data_and_new_run_interrupts_old(self):
        self.begin(); self.fill()
        self.session.consume(event('tcp', []))
        self.session.consume(event('start', '0', 2.))
        self.assertTrue((self.root/'run_10000000000/tcp.csv').exists())
        self.assertIn('tail_not_complete', self.session.closed[0]['quality'])
        self.begin('20000000000', start=20.)
        self.begin('30000000000', start=30.)
        self.assertTrue(self.session.closed[-1]['state'].startswith('INTERRUPTED:'))
        self.session.finish('INTERRUPTED: fixture')

    def test_fault_and_missing_metadata_visible_but_excluded_from_aggregates(self):
        self.begin(); self.fill(); self.end()
        self.begin('20000000000', start=20.); self.fill(20.); self.end('FAULT: test', 20.)
        self.begin('30000000000', start=30., metadata=False); self.fill(30.); self.end(start=30.)
        report=self.root/'report'
        result=compare_runs([self.root], report, plots=False)
        self.assertEqual(result['runs'], 3)
        data=json.loads((report/'comparison.json').read_text())
        self.assertEqual(len(data['aggregates']), 1)
        self.assertEqual(data['aggregates'][0]['repeats'], 1)
        self.assertIn('FAULT: test', (report/'comparison.csv').read_text())
        self.assertIn('parameters_not_frozen', (report/'comparison.csv').read_text())

    def test_start_pose_mismatch_separates_groups(self):
        self.begin(); self.fill(); self.end()
        run='20000000000'
        self.session.consume(event('metadata', frozen(run, origin=['.01', '0', '0']), start=20.))
        self.session.consume(event('start', run, start=20.))
        self.session.consume(event('state', 'RUNNING: fixture', start=20.))
        self.fill(20.); self.end(start=20.)
        compare_runs([self.root], self.root/'report', plots=False)
        data=json.loads((self.root/'report/comparison.json').read_text())
        self.assertEqual(len({r['comparison_group'] for r in data['runs']}), 2)

    def test_repeated_runs_aggregate_without_pooling_samples(self):
        for run,start in [('10000000000',10.),('20000000000',20.)]:
            self.begin(run,start=start);self.fill(start);self.end(start=start)
        compare_runs([self.root],self.root/'report',plots=False)
        groups=json.loads((self.root/'report/comparison.json').read_text())['aggregates']
        self.assertEqual(groups[0]['repeats'],2)
        self.assertAlmostEqual(groups[0]['rms_mean_mm'],1.)
        self.assertEqual(groups[0]['rms_std_mm'],0.)

    def test_invalid_frozen_number_rejected(self):
        data=frozen();data['parameters']['kp']='nan'
        with self.assertRaises(ValueError):normalize_metadata(data)

    def test_gui_float_roundoff_does_not_split_repeat_group(self):
        for run,start,kp in [('10000000000',10.,2.),('20000000000',20.,1.9999999999999991)]:
            self.begin(run,start=start,kp=kp);self.fill(start);self.end(start=start)
        compare_runs([self.root],self.root/'report',plots=False)
        groups=json.loads((self.root/'report/comparison.json').read_text())['aggregates']
        self.assertEqual(len(groups),1)
        self.assertEqual(groups[0]['repeats'],2)


if __name__ == '__main__':
    unittest.main()
