import http.client
import json
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
ROOT = Path(__file__).resolve().parents[2]
for path in (ROOT / 'src', ROOT.parent / 'aubo_cloud_bridge/src', ROOT / 'dashboard'):
    sys.path.insert(0, str(path))
from aubo_huawei_bridge.web_tasks import JournalInUseError, WebTasks
from aubo_huawei_bridge.iot import make_command
from app import App
from http_server import BoundedServer


class LocalTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.calls = []
        def execute(method, guard):
            self.assertTrue(guard())
            self.calls.append(method)
            return True, 'test-only service accepted'
        self.tasks = WebTasks(self.tmp.name, execute, enabled=True)
        self.tasks.health(True)
        self.tasks.telemetry.update('state', 'READY|test fixture')
        self.addCleanup(self.clean_tasks)

    def clean_tasks(self):
        self.tasks.close()
        for _ in range(100):
            if not self.tasks.dispatcher.active:
                break
            time.sleep(.005)
        self.tasks.journal.db.close()
        self.tasks.owner.close()

    def command(self, method):
        return make_command(method, self.tasks.context()[0], int(time.time()*1000))

    def test_persisted_dedup_and_exclusive_owner(self):
        with self.assertRaises(ValueError):
            WebTasks(self.tmp.name, lambda *args: (True,''), True)
        cmd = self.command('start')
        result = self.tasks.command(cmd)
        self.assertEqual(result['response']['result_code'],0)
        duplicate = self.tasks.command(cmd)
        self.assertTrue(duplicate['response']['paras']['result']['duplicate'])
        self.assertEqual(self.calls,['start'])
        for _ in range(100):
            if not self.tasks.dispatcher.active:
                break
            time.sleep(.005)
        self.tasks.journal.db.close()
        self.tasks.journal = __import__('aubo_cloud_bridge.protocol',fromlist=['CommandJournal']).CommandJournal(str(Path(self.tmp.name)/'dashboard_commands.sqlite3'))
        self.tasks.dispatcher.journal = self.tasks.journal
        self.assertTrue(self.tasks.command(cmd)['response']['paras']['result']['duplicate'])
        self.assertEqual(self.calls,['start'])

    def test_restart_waits_for_old_journal_owner_and_never_bypasses_an_active_owner(self):
        ready = threading.Event()
        result = []
        def start_replacement():
            ready.set()
            result.append(WebTasks(self.tmp.name, lambda *args: (True, ''), owner_wait=1))
        worker = threading.Thread(target=start_replacement)
        worker.start()
        self.assertTrue(ready.wait(1))
        time.sleep(.08)
        self.assertEqual(result, [])
        self.tasks.owner.close()
        worker.join(2)
        self.assertFalse(worker.is_alive())
        self.assertEqual(len(result), 1)
        replacement = result[0]
        try:
            with self.assertRaises(JournalInUseError):
                WebTasks(self.tmp.name, lambda *args: (True, ''), owner_wait=.05)
        finally:
            replacement.close()
            replacement.journal.db.close()
            replacement.owner.close()
    def test_health_rotates_session_and_stale_ticket_cannot_execute(self):
        cmd = self.command('home')
        self.tasks.health(False)
        self.tasks.health(True)
        self.assertNotEqual(cmd['paras']['session'],self.tasks.context()[0])
        self.assertEqual(self.tasks.command(cmd)['response']['result_code'],1)
        self.tasks.checked -= 5
        self.assertEqual(self.tasks.snapshot()['sorting_available'],0)
        self.assertEqual(self.calls,[])

    def test_read_only_never_calls_executor(self):
        self.tasks.dispatcher.enabled = False
        result = self.tasks.command(self.command('start'))
        self.assertEqual(result['response']['result_code'],1)
        self.assertEqual(self.calls,[])

    def test_stop_executes_while_motion_is_waiting(self):
        entered, stopped = threading.Event(), threading.Event()
        def execute(method, guard):
            self.assertTrue(guard())
            self.calls.append(method)
            if method=='start':
                entered.set()
                self.assertTrue(stopped.wait(2))
            else:
                stopped.set()
            return True,'test'
        self.tasks.dispatcher.execute = execute
        worker = threading.Thread(target=lambda:self.tasks.command(self.command('start')))
        worker.start()
        self.assertTrue(entered.wait(1))
        result = self.tasks.command(self.command('stop'))
        worker.join(2)
        self.assertFalse(worker.is_alive())
        self.assertEqual(result['response']['result_code'],0)
        self.assertEqual(self.calls,['start','stop'])

    def test_real_http_auth_ticket_dispatch_and_ambiguous_length_rejection(self):
        with patch.dict(os.environ,{'AUBO_DASHBOARD_MODE':'ros','DASHBOARD_PASSWORD':'local-test-password-long',
                                    'DASHBOARD_SESSION_KEY':'k'*40,'DASHBOARD_ENABLE_COMMANDS':'true'}):
            application = App(self.tasks)
        server = BoundedServer(('127.0.0.1',0),application)
        thread = threading.Thread(target=server.serve_forever,daemon=True)
        thread.start()
        def request(path, data=None, token=None):
            connection=http.client.HTTPConnection(*server.server_address,timeout=3)
            headers={'Content-Type':'application/json'}
            if token:
                headers['Authorization']='Bearer '+token
            connection.request('POST' if data is not None else 'GET',path,json.dumps(data) if data is not None else None,headers)
            response=connection.getresponse()
            result=response.status,json.loads(response.read())
            connection.close()
            return result
        try:
            self.assertEqual(request('/api/snapshot')[0],401)
            _, login=request('/api/login',{'password':'local-test-password-long'})
            _, data=request('/api/snapshot',token=login['token'])
            self.assertEqual(data['mode'],'ros')
            self.assertEqual(data['poll_seconds'],2)
            _, ticket=request('/api/prepare',{'method':'start'},login['token'])
            status, result=request('/api/command',ticket,login['token'])
            self.assertEqual(status,200)
            self.assertEqual(result['response']['response']['result_code'],0)
            self.assertEqual(self.calls,['start'])
            request('/api/command',ticket,login['token'])
            self.assertEqual(self.calls,['start'])
            connection=http.client.HTTPConnection(*server.server_address,timeout=3)
            connection.putrequest('POST','/api/login')
            connection.putheader('Content-Length','0')
            connection.putheader('Content-Length','1')
            connection.endheaders()
            response=connection.getresponse()
            self.assertEqual(response.status,400)
            response.read()
            connection.close()
        finally:
            server.shutdown()
            server.server_close()
            thread.join(1)


if __name__=='__main__':
    unittest.main()
