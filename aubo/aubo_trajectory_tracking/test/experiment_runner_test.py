#!/usr/bin/env python3
"""Plan/profile checks only; imports no ROS and never executes motion."""
import sys
import unittest
import tempfile
import json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from run_tracking_experiments import build_plan,validate_profile,remaining_plan,load_resume_manifest


class RunnerPlanTest(unittest.TestCase):
    def setUp(self):
        self.p=dict(shape='circle',cycles=1,radius=.12,duration=40.,max_speed=.07,
                    kp=2.,ki=0.,kd=0.,acceleration_ff_time=.1,correction_fraction=.2,
                    plane='xy',max_extent=.25,check_feasibility=True)

    def test_two_balanced_repetitions(self):
        plan=build_plan(2)
        self.assertEqual(len(plan),12)
        self.assertEqual([r['case'] for r in plan[:6]],list(reversed([r['case'] for r in plan[6:]])))
        self.assertEqual(plan[0]['case'],'baseline')
        self.assertEqual(plan[-1]['case'],'baseline')
        self.assertLessEqual(max(r['ki'] for r in plan),.05)
        self.assertLessEqual(max(r['kd'] for r in plan),.02)

    def test_real_profile_must_match_approved_task(self):
        validate_profile(self.p,'sdk','sdk',False,True)
        for key,value in [('radius',.13),('radius',float('nan')),('duration',20.),('max_speed',.08),('check_feasibility',False),('shape','sine')]:
            p=dict(self.p,**{key:value})
            with self.assertRaises(RuntimeError):validate_profile(p,'sdk','sdk',False,True)

    def test_fake_backend_cannot_masquerade_as_real(self):
        with self.assertRaises(RuntimeError):validate_profile(self.p,'gazebo','sdk',False,True)
        with self.assertRaises(RuntimeError):validate_profile(self.p,'sdk','sdk',True,True)
        with self.assertRaises(RuntimeError):validate_profile(self.p,'sdk','sdk',False,False)

    def test_resume_excludes_completed_but_keeps_faulted_case(self):
        plan=build_plan(2)
        outcomes=[dict(row,state='COMPLETED: fixture',quality=[]) for row in plan[:7]]
        outcomes.append(dict(plan[7],state='FAULT: fixture',quality=['not_completed']))
        result=remaining_plan(dict(plan=plan,outcomes=outcomes))
        self.assertEqual(result,plan[7:])
        self.assertEqual(len(result),5)

    def test_resume_rejects_unapproved_tuning(self):
        plan=build_plan(1);plan[0]['ki']=1.
        with self.assertRaises(RuntimeError):remaining_plan(dict(plan=plan,outcomes=[]))

    def test_completed_recovery_manifest_cannot_restart_old_missing_cases(self):
        with tempfile.TemporaryDirectory() as temp:
            p=Path(temp)/'experiment.json';p.write_text(json.dumps(dict(plan=build_plan(1),outcomes=[])))
            self.assertEqual(len(load_resume_manifest(p)['plan']),6)
            (p.parent/'completion.json').write_text(json.dumps(dict(state='COMPLETE')))
            with self.assertRaises(RuntimeError):load_resume_manifest(p)


if __name__=='__main__':unittest.main()
