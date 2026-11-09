import base64
import copy
import json
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import app


class Source:
    def __init__(self, now):
        self.p = {'bridge_wall_ms': str(int(now * 1000)), 'bridge_session': 'a' * 32,
                  'commands_enabled': 1, 'sorting_available': 1, 'state': 'READY'}
        self.sent = []
        self.fail = False

    def snapshot(self):
        return dict(self.p)

    def command(self, body):
        self.sent.append(copy.deepcopy(body))
        if self.fail:
            raise TimeoutError('sensitive-provider-detail')
        return {'response': {'result_code': 0}}


class DashboardTests(unittest.TestCase):
    def setUp(self):
        self.env = patch.dict(os.environ, {'AUBO_DASHBOARD_MODE': 'cloud',
            'DASHBOARD_PASSWORD': 'test-password-long-enough', 'DASHBOARD_SESSION_KEY': 'x' * 40,
            'DASHBOARD_ENABLE_COMMANDS': 'true'})
        self.env.start()
        self.addCleanup(self.env.stop)
        self.now = [1794200000.0]
        self.source = Source(self.now[0])
        self.app = app.App(self.source, lambda: self.now[0])
        status, result = self.call('POST', '/api/login', {'password': 'test-password-long-enough'}, auth=False)
        self.assertEqual(status, 200)
        self.token = result['token']

    def call(self, method, path, data=None, auth=True):
        headers = {'Content-Type': 'application/json'}
        if auth:
            headers['Authorization'] = 'Bearer ' + self.token
        status, _, body = self.app.route(method, path, headers, app.dumps(data).encode() if data is not None else b'')
        return status, json.loads(body)

    def test_authorization_expiry_and_static_allowlist(self):
        self.assertEqual(self.call('GET', '/api/snapshot', auth=False)[0], 401)
        self.assertEqual(self.call('POST', '/api/login', {'password': 'wrong'}, auth=False)[0], 401)
        self.assertEqual(self.app.route('GET', '/../../app.py')[0], 404)
        status, headers, _ = self.app.route('GET', '/')
        self.assertEqual(status, 200)
        self.assertIn("script-src 'self'", headers['Content-Security-Policy'])
        self.now[0] += 7201
        self.assertEqual(self.call('GET', '/api/snapshot')[0], 401)
        self.assertEqual(self.source.sent, [])

    def test_readonly_stale_future_unavailable_and_session_change_never_dispatch(self):
        self.app.controls = False
        self.assertEqual(self.call('POST', '/api/prepare', {'method': 'start'})[0], 409)
        self.app.controls = True
        self.assertEqual(self.call('POST', '/api/prepare', {'method': 'open_gripper'})[0], 400)
        for key, val in [('bridge_wall_ms',str(int((self.now[0]-91)*1000))),
                         ('bridge_wall_ms',str(int((self.now[0]+1)*1000))),
                         ('commands_enabled',0), ('sorting_available',0), ('state','PICKING')]:
            before = dict(self.source.p)
            self.source.p[key] = val
            self.assertEqual(self.call('POST', '/api/prepare', {'method':'start'})[0],409)
            self.source.p = before
        status, ticket = self.call('POST','/api/prepare',{'method':'start'})
        self.assertEqual(status,200)
        self.source.p['bridge_session']='b'*32
        self.assertEqual(self.call('POST','/api/command',ticket)[0],409)
        self.assertEqual(self.source.sent,[])

    def test_signed_ticket_tamper_expiry_and_repeat_keep_same_command_id(self):
        _, ticket=self.call('POST','/api/prepare',{'method':'start'})
        bad={'ticket': ticket['ticket']+'x'}
        self.assertEqual(self.call('POST','/api/command',bad)[0],401)
        for _ in range(2):
            self.assertEqual(self.call('POST','/api/command',ticket)[0],200)
        self.assertEqual(len(self.source.sent),2)
        # Actual execution deduplication is the persistent ROS CommandJournal.
        self.assertEqual(self.source.sent[0],self.source.sent[1])
        self.now[0]+=26
        self.assertEqual(self.call('POST','/api/command',ticket)[0],401)
        self.assertEqual(len(self.source.sent),2)

    def test_unknown_is_not_retried_and_stop_during_picking(self):
        self.source.p['state']='PICKING'
        _, ticket=self.call('POST','/api/prepare',{'method':'stop'})
        self.source.fail=True
        status, result=self.call('POST','/api/command',ticket)
        self.assertEqual(status,200)
        self.assertEqual(result['status'],'unknown')
        self.assertNotIn('sensitive-provider-detail',json.dumps(result))
        self.assertEqual(len(self.source.sent),1)

    def test_functiongraph_base64_and_v2_events(self):
        with patch.object(app,'_application',self.app):
            event={'httpMethod':'POST','path':'/api/login','headers':{'content-type':'application/json'},
                   'body':base64.b64encode(b'{"password":"test-password-long-enough"}').decode(),
                   'isBase64Encoded':True}
            response=app.handler(event,None)
            self.assertEqual(response['statusCode'],200)
            response=app.handler({'rawPath':'/','requestContext':{'http':{'method':'GET'}}},None)
            self.assertEqual(response['statusCode'],200)
            self.assertIn('AUBO',response['body'])

    def test_demo_is_always_readonly_even_when_command_flag_true(self):
        with patch.dict(os.environ,{'AUBO_DASHBOARD_MODE':'demo'}):
            demo=app.App()
            self.assertFalse(demo.controls)
            self.assertEqual(demo.snapshot()['mode'],'demo')
            with self.assertRaises(app.APIError):
                demo.guard(demo.snapshot(),'start')


if __name__ == '__main__':
    unittest.main()
