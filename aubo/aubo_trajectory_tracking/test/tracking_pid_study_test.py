#!/usr/bin/env python3
import csv
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from tracking_pid_study import curve_samples, velocity_window, replay, analyze_pid_inputs
from tracking_experiment_data import TCP_FIELDS


class PidStudyTest(unittest.TestCase):
    def setUp(self):
        self.p=dict(shape='circle',plane='xy',duration=4.,radius=.03,amplitude=.01,length=.02,
                    cycles=1.,waves=1.,kp=2.,ki=0.,kd=0.,acceleration_ff_time=.1,
                    max_speed=.1,correction_fraction=.2,settle_tolerance=.002,velocity_window=.08)

    def run_signals(self,error,ki=0.,kd=0.,velocity_error=None,ff=None,duration=100.):
        t=np.arange(1,len(error)+1)*.02
        if velocity_error is None:velocity_error=np.zeros_like(error)
        if ff is None:ff=np.zeros_like(error)
        return replay(t,ff,error,velocity_error,2.,ki,kd,.07,.2,.002,duration)

    def test_reference_derivatives_both_shapes_all_planes(self):
        for shape in ['circle','sine']:
            for plane in ['xy','xz','yz']:
                self.p.update(shape=shape,plane=plane)
                p,v,a=curve_samples(np.array([1.2-1e-5,1.2,1.2+1e-5]),self.p,[.1,.2,.3])
                np.testing.assert_allclose(v[1],(p[2]-p[0])/2e-5,rtol=1e-6,atol=1e-8)
                np.testing.assert_allclose(a[1],(v[2]-v[0])/2e-5,rtol=1e-6,atol=1e-8)

    def test_timestamped_velocity_on_irregular_samples(self):
        t=np.array([0,.019,.043,.067,.082,.109])
        v=np.array([.01,-.02,.03]);p=t[:,None]*v+[1.,2.,3.]
        np.testing.assert_allclose(velocity_window(t,p,.08)[1:],np.tile(v,(len(t)-1,1)),atol=1e-12)

    def test_integral_uses_signed_xyz_and_actual_dt(self):
        error=np.tile([.001,-.001,0.],(100,1))
        result=self.run_signals(error,ki=.05)
        np.testing.assert_allclose(result['integral'][-1],[.0001,-.0001,0.],atol=1e-14)
        np.testing.assert_allclose(result['command'][-1],[.0021,-.0021,0.],atol=1e-14)
        # Each candidate starts with its own zero integrator.
        zero=self.run_signals(np.zeros((5,3)),ki=.05)
        np.testing.assert_allclose(zero['command'],0.)

    def test_saturation_does_not_charge_integral(self):
        result=self.run_signals(np.tile([1.,0.,0.],(100,1)),ki=2.)
        np.testing.assert_allclose(result['integral'],0.)
        self.assertLessEqual(np.linalg.norm(result['command'],axis=1).max(),.014+1e-14)
        self.assertTrue(result['antiwindup'].all())

    def test_derivative_uses_vector_velocity_error(self):
        result=self.run_signals(np.zeros((10,3)),kd=.1,
                                velocity_error=np.tile([.01,-.02,.03],(10,1)))
        np.testing.assert_allclose(result['command'],np.tile([.001,-.002,.003],(10,1)),atol=1e-14)

    def test_nonincreasing_control_time_rejected(self):
        with self.assertRaises(ValueError):
            replay(np.array([.02,.01]),np.zeros((2,3)),np.zeros((2,3)),np.zeros((2,3)),
                   2.,0.,0.,.07,.2,.002,40.)

    def test_actual_statistics_are_not_replaced_by_candidate_error_predictions(self):
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp)/'source';folder.mkdir()
            self.p['radius']=.01  # Reference FF stays below both caps in this fixture.
            frozen=dict(parameters=self.p,origin_m=[0.,0.,0.],base_link='base',tool_link='tcp')
            (folder/'metadata.json').write_text(json.dumps(dict(run_id='1',state='COMPLETED: fixture',frozen=frozen)))
            t=np.arange(1,199)*.02
            p,v,a=curve_samples(t,self.p,[0.,0.,0.])
            # Exact observed reference motion; baseline I=D=0 has known FF norm.
            ff=v+.1*a
            with (folder/'tcp.csv').open('w',newline='') as f:
                w=csv.writer(f);w.writerow(TCP_FIELDS)
                for i,stamp in enumerate(t):
                    w.writerow([stamp]+p[i].tolist()+p[i].tolist()+[0.,np.linalg.norm(v[i]),
                        np.linalg.norm(v[i]),np.linalg.norm(ff[i]),np.linalg.norm(v[i]),0.,0.,0.,0.])
            out=Path(temp)/'out'
            result=analyze_pid_inputs(folder,[0.,.02],[0.,.02],out)
            self.assertEqual(result['candidates'],4)
            self.assertLess(result['baseline_reconstruction_rms_mm_s'],1e-9)
            data=json.loads((out/'id_statistics.json').read_text())
            self.assertEqual(data['observed_running_error_mm']['rms'],0.)
            with (out/'id_candidates.csv').open() as f:
                fields=csv.DictReader(f).fieldnames
            self.assertFalse(any('tracking_rms' in k or 'tracking_peak' in k for k in fields))
            candidate=json.loads((out/'candidate_D002.yaml').read_text())
            self.assertEqual(candidate['kp'],2.)
            self.assertEqual(candidate['kd'],.02)

    @unittest.skipUnless(os.environ.get('PID_REPLAY_REFERENCE'),'production C++ reference executable not configured')
    def test_velocity_window_agrees_at_absolute_ros_time_boundaries(self):
        t=1793608303.+np.arange(250)*.02
        p=np.column_stack([.3+.02*np.sin(np.arange(250)*.02),
                           .1+.01*np.cos(np.arange(250)*.01),np.zeros(250)])
        expected=velocity_window(t,p,.08)
        text='.08\n'+'\n'.join(' '.join('%.17g'%v for v in row) for row in np.column_stack([t,p]))+'\n'
        result=subprocess.run([os.environ['PID_REPLAY_REFERENCE'],'--velocity'],input=text,
                              text=True,capture_output=True,check=True)
        actual=np.array([[float(v) for v in line.split()] for line in result.stdout.splitlines()])
        np.testing.assert_allclose(expected,actual,atol=1e-12,rtol=1e-10)

    @unittest.skipUnless(os.environ.get('PID_REPLAY_REFERENCE'),'production C++ reference executable not configured')
    def test_agrees_with_production_cpp_under_limits_unwinding_and_settling(self):
        rng=np.random.RandomState(417)
        t=np.cumsum(np.r_[.02,rng.uniform(.015,.025,199)])
        error=rng.normal(0,.001,(200,3));error[40:80,0]+=.03
        error[170:]=.0001
        ve=rng.normal(0,.015,(200,3));ff=rng.normal(0,.01,(200,3));ff[150:]=0.
        for ki in [0.,.05,2.]:
            for kd in [0.,.02,.5]:
                expected=replay(t,ff,error,ve,2.,ki,kd,.07,.2,.002,3.)['command']
                text='2 %g %g .07 .2 .002 3\n'%(ki,kd)
                text+='\n'.join(' '.join('%.17g'%v for v in row) for row in np.column_stack([t,ff,error,ve]))+'\n'
                result=subprocess.run([os.environ['PID_REPLAY_REFERENCE']],input=text,text=True,
                                      capture_output=True,check=True)
                actual=np.array([[float(v) for v in line.split()] for line in result.stdout.splitlines()])
                np.testing.assert_allclose(expected,actual,atol=1e-12,rtol=1e-10)


if __name__=='__main__':
    unittest.main()
