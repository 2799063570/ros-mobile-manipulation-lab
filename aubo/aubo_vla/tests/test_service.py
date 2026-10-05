"""Transport boundary tests; never count mock as model acceptance."""
import argparse
import base64
import io
import json
from pathlib import Path
import sys
import threading
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from PIL import Image
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from model_service import Predictor, handler_for, validate_action


class ServiceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.predictor = Predictor(argparse.Namespace(mock=True, revision='test', unnorm_key='bridge_orig'))
        cls.server = ThreadingHTTPServer(('127.0.0.1', 0), handler_for(cls.predictor))
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()
        cls.url = 'http://127.0.0.1:%d' % cls.server.server_port
        cls.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        b = io.BytesIO()
        Image.new('RGB', (8, 8), (255, 0, 0)).save(b, 'PNG')
        cls.payload = dict(image_png=base64.b64encode(b.getvalue()).decode(), request_id='test-id',
                           stamp=1.5, frame_id='camera', instruction='pick up red block')

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join()

    def post(self, payload):
        r = urllib.request.Request(self.url+'/predict', json.dumps(payload).encode(),
                                   {'Content-Type': 'application/json'})
        return self.opener.open(r, timeout=2)

    def test_echo_and_mock_marker(self):
        with self.post(self.payload) as r:
            data = json.load(r)
        self.assertTrue(data['mock'])
        self.assertTrue(data['observation_only'])
        self.assertEqual(data['stamp'], 1.5)
        self.assertEqual(data['request_id'], 'test-id')
        self.assertEqual(len(data['action']), 7)

    def test_invalid_image(self):
        with self.assertRaises(urllib.error.HTTPError) as e:
            self.post(dict(self.payload, image_png='not base64!'))
        self.assertEqual(e.exception.code, 400)

    def test_busy_rejected(self):
        with self.predictor.lock:
            with self.assertRaises(urllib.error.HTTPError) as e:
                self.post(self.payload)
        self.assertEqual(e.exception.code, 409)

    def test_nonfinite_action_rejected(self):
        for action in ([0]*6, [0]*6+[float('nan')], [0]*6+[float('inf')]):
            with self.assertRaises(ValueError):
                validate_action(action)

    def test_invalid_stamp(self):
        with self.assertRaises(urllib.error.HTTPError) as e:
            self.post(dict(self.payload, stamp=float('nan')))
        self.assertEqual(e.exception.code, 400)

    def test_health_marks_action_semantics_unspecified(self):
        with self.opener.open(self.url+'/health', timeout=2) as response:
            data = json.load(response)
        self.assertEqual(data['action_schema'], 'unspecified')
        self.assertIsNone(data['action_dt'])

    def test_bridge_statistics_cannot_be_declared_aubo(self):
        with self.assertRaises(ValueError):
            Predictor(argparse.Namespace(mock=True, revision='test', unnorm_key='bridge_orig',
                                         action_schema='aubo_delta_pose_v1', action_dt=0.5))

    def test_invalid_metadata_rejected(self):
        for change in (dict(stamp=True), dict(stamp=-1.), dict(request_id=''), dict(frame_id=[])):
            with self.assertRaises(urllib.error.HTTPError) as error:
                self.post(dict(self.payload, **change))
            self.assertEqual(error.exception.code, 400)


if __name__ == '__main__':
    unittest.main()
