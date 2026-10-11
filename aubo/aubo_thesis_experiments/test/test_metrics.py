import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from aubo_thesis_experiments.metrics import localization, summarize_events
from make_experiment_matrix import make_matrix

class Measurements(unittest.TestCase):
    def sample(self):
        return dict(sample_id='p1',frame_id='base_link',true_frame_id='base_link',
                    true_class='bottle',pred_class='bottle',valid='1',
                    pred_x='0.003',pred_y='0.004',pred_z='0',true_x='0',true_y='0',true_z='0',
                    pred_yaw_rad=str(math.pi-math.radians(1)),true_yaw_rad='0',
                    pred_width_m='.041',true_width_m='.040')

    def test_units_pi_period_and_failed_detection_denominator(self):
        good=self.sample(); failed=dict(good,sample_id='p2',valid='0',pred_class='')
        result=localization([good,failed],5)
        self.assertAlmostEqual(result['position_distance']['rmse'],.005)
        self.assertAlmostEqual(result['yaw_abs_error_deg']['mean'],1)
        self.assertEqual(result['classification_rate_all'],.5)
        self.assertEqual(result['angle_correct_all'],.5)
        self.assertEqual(result['angle_correct_valid_only'],1)

    def test_mismatched_frame_and_nonfinite_are_rejected(self):
        with self.assertRaises(ValueError): localization([dict(self.sample(),true_frame_id='camera')],5)
        with self.assertRaises(ValueError): localization([dict(self.sample(),pred_x='nan')],5)
        with self.assertRaises(ValueError): localization([self.sample(),self.sample()],5)

    def test_missing_independent_labels_never_become_sort_success(self):
        events=[dict(kind='attempt_begin',attempt_id='a',condition='S4'),
                dict(kind='attempt_end',attempt_id='a',status='controller_success')]
        result=summarize_events(events)['conditions']['S4']
        self.assertEqual(result['controller_completion_rate'],1)
        self.assertIsNone(result['verified_sort_rate'])
        labels=[dict(attempt_id='a',class_correct='1',grasp_success='1',placed_correct='0')]
        self.assertEqual(summarize_events(events,labels)['conditions']['S4']['verified_sort_rate'],0)

    def test_incomplete_trials_are_retained(self):
        events=[dict(kind='attempt_begin',attempt_id='a',condition='S1')]
        result=summarize_events(events)
        self.assertEqual(result['conditions']['S1']['attempts'],1)
        self.assertEqual(result['diagnostics']['incomplete_attempts'],['a'])

    def test_exclusive_stage_time_does_not_double_count_children(self):
        common=dict(run_id='run',condition='S1',kind='stage_end')
        events=[dict(common,span_id=1,parent_span_id=0,stage='operation',elapsed_s=10),
                dict(common,span_id=2,parent_span_id=1,stage='execution',elapsed_s=6)]
        stages={r['stage']:r for r in summarize_events(events)['stages']}
        self.assertEqual(stages['operation']['exclusive_s']['mean'],4)
        self.assertEqual(stages['execution']['exclusive_s']['mean'],6)

    def test_duplicate_labels_rejected(self):
        label=dict(attempt_id='a',class_correct='1',grasp_success='1',placed_correct='1')
        with self.assertRaises(ValueError): summarize_events([], [label,label])

    def test_malformed_feedback_is_not_silently_truncated(self):
        with self.assertRaises(ValueError):
            summarize_events([dict(kind='joint_feedback',joint_names=['a','b'],error_positions=[.1])])

    def test_negative_stage_duration_is_rejected(self):
        with self.assertRaises(ValueError):
            summarize_events([dict(kind='stage_end',run_id='run',condition='S1',span_id=1,
                                   parent_span_id=0,stage='execution',elapsed_s=-1)])

    def test_balanced_paired_schedule_is_repeatable(self):
        rows=make_matrix(['easy01','hard01'],[42,43],2,42,'batch')
        self.assertEqual(len(rows),32)
        self.assertEqual(len({r['run_id'] for r in rows}),32)
        self.assertEqual(rows,make_matrix(['easy01','hard01'],[42,43],2,42,'batch'))
        groups={}
        for row in rows: groups.setdefault(row['pair_id'],set()).add(row['condition'])
        self.assertTrue(all(v=={'S1','S2','S3','S4'} for v in groups.values()))

if __name__=='__main__': unittest.main()
