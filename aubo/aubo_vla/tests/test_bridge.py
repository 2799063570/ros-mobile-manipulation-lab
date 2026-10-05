"""Exercise the real bridge callbacks/run loop with ROS and image IO doubles."""
import importlib.util
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))


class BridgeTest(unittest.TestCase):
    def setUp(self):
        self.params = {'/use_sim_time': False}
        self.rospy = NS(get_param=lambda key, default=None: self.params.get(key, default),
                        Publisher=Mock(), Subscriber=Mock(), logwarn=Mock(), logwarn_throttle=Mock(),
                        Time=NS(now=lambda: NS(to_sec=lambda: 100.2)),
                        is_shutdown=Mock(side_effect=[False, True]))
        modules = {'rospy': self.rospy, 'cv2': NS(imencode=lambda *a: (True, NS(tobytes=lambda: b'png'))),
                   'cv_bridge': NS(CvBridge=lambda: NS(imgmsg_to_cv2=lambda *a: None)),
                   'rosgraph_msgs.msg': NS(Clock=object), 'sensor_msgs.msg': NS(Image=object),
                   'std_msgs.msg': NS(String=lambda **kw: NS(**kw))}
        path = Path(__file__).resolve().parents[1]/'scripts/observe_bridge.py'
        spec = importlib.util.spec_from_file_location('tested_bridge', path)
        self.module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, modules):
            spec.loader.exec_module(self.module)
        self.bridge = self.module.Bridge()
        self.image = NS(header=NS(stamp=NS(to_sec=lambda: 100.), frame_id='camera'))

    def run_prediction(self, during=None):
        def open_request(request, timeout):
            payload = json.loads(request.data)
            if during:
                during()
            result = dict(payload, action=[0.]*7, mock=False, observation_only=True)
            response = Mock()
            response.__enter__ = Mock(return_value=NS(read=lambda _: json.dumps(result).encode()))
            response.__exit__ = Mock(return_value=False)
            return response
        self.bridge.http = NS(open=open_request)
        self.bridge.on_image(self.image)
        with patch.object(self.module.time, 'sleep', return_value=None):
            self.bridge.run()
        return json.loads(self.bridge.pub.publish.call_args.args[0].data)

    def test_real_camera_runs_without_clock(self):
        self.assertEqual(self.run_prediction()['status'], 'observed')

    def test_instruction_change_invalidates_inflight_response(self):
        result = self.run_prediction(lambda: self.bridge.on_instruction(NS(data='pick blue')))
        self.assertEqual(result['status'], 'discarded')
        self.assertEqual(self.bridge.instruction, 'pick blue')

    def test_pause_resume_invalidates_inflight_response(self):
        self.bridge.clock_guard.simulated = True
        self.bridge.on_clock(NS(clock=NS(to_sec=lambda: 100.)))
        def pause_resume():
            self.bridge.clock_guard.wall = time.monotonic()-3.
            self.bridge.on_clock(NS(clock=NS(to_sec=lambda: 100.1)))
        self.assertEqual(self.run_prediction(pause_resume)['status'], 'discarded')

    def test_old_source_timestamp_rejected_even_when_just_received(self):
        self.image.header.stamp = NS(to_sec=lambda: 10.)
        self.assertEqual(self.run_prediction()['status'], 'error')

    def test_response_that_ages_during_inference_is_discarded(self):
        self.bridge.max_age = 1.
        def expire():
            self.rospy.Time = NS(now=lambda: NS(to_sec=lambda: 102.))
        self.assertEqual(self.run_prediction(expire)['status'], 'discarded')

    def test_invalid_instruction_does_not_change_epoch(self):
        self.bridge.on_instruction(NS(data=' '))
        self.assertEqual(self.bridge.epoch, 0)
