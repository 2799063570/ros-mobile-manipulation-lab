import json
from pathlib import Path
import sys
import uuid
import threading
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from aubo_cloud_bridge.protocol import CommandJournal, Telemetry, validate_rpc, decode_json
from aubo_cloud_bridge.commands import Dispatcher


def request(method='start', command_id='0123456789abcdef'):
    now = int(time.time() * 1000)
    return {'method': method, 'params': {'command_id': command_id, 'session': 'session-1',
                                        'issued_at_ms': now, 'expires_at_ms': now + 10000}}


class ProtocolTests(unittest.TestCase):
    def test_core_schema_two_queue_and_counts(self):
        telemetry = Telemetry()
        targets = {str(i): {'id': i, 'category': '饮料罐', 'status': status,
                           'detection_confidence': .85} for i, status in enumerate(('ready', 'done', 'reserved'))}
        telemetry.update('target_cache', json.dumps({'schema_version': 2, 'targets': targets}), now=10)
        telemetry.update('state', 'READY | waiting for panel command', now=12)
        telemetry.update('progress', '{"phase":"PREGRASP","description":"接近目标"}', now=13)
        snapshot = telemetry.snapshot(now=15)
        self.assertEqual((snapshot['queue_total'], snapshot['queue_done'], snapshot['queue_ready']), (3, 1, 1))
        self.assertEqual(snapshot['target_cache_age_s'], 5)
        self.assertEqual(snapshot['state'], 'READY')
        self.assertEqual(json.loads(snapshot['target_cache'])['targets'][0]['category'], '饮料罐')

    def test_invalid_snapshot_does_not_erase_last_good_data(self):
        telemetry = Telemetry()
        telemetry.update('target_cache', '{"schema_version":2,"targets":{}}', now=1)
        for raw in ('{}', '{"schema_version":2,"targets":[]}', 'NaN'):
            with self.assertRaises(ValueError):
                telemetry.update('target_cache', raw)
        self.assertEqual(telemetry.snapshot(now=2)['target_cache_age_s'], 1)
        with self.assertRaises(ValueError):
            decode_json('{"value":Infinity}')

    def test_expiry_session_and_method_validation(self):
        now = int(time.time() * 1000)
        self.assertEqual(validate_rpc(request(), 'session-1', now)[0], 'start')
        for mutate in (lambda r: r.update(method='open_gripper'),
                       lambda r: r['params'].update(session='old-session'),
                       lambda r: r['params'].update(expires_at_ms=now - 1),
                       lambda r: r['params'].update(issued_at_ms=now + 6000),
                       lambda r: r['params'].update(expires_at_ms=now + 60000),
                       lambda r: r['params'].update(issued_at_ms=True),
                       lambda r: r['params'].update(command_id='short'),
                       lambda r: r['params'].update(service='/arbitrary_service')):
            document = request()
            mutate(document)
            with self.assertRaises(ValueError):
                validate_rpc(document, 'session-1', now)

    def test_durable_dedup_and_crash_unknown(self):
        path = str(Path(__file__).resolve().parent / (uuid.uuid4().hex + '.sqlite3'))
        try:
            original = request()
            journal = CommandJournal(path)
            self.assertTrue(journal.reserve(original)[0])
            # A fresh process must never replay a possibly executed command.
            journal.db.close()
            journal = CommandJournal(path)
            self.assertEqual(journal.reserve(original)[1]['status'], 'unknown')
            journal.finish(original['params']['command_id'], {'status': 'accepted'})
            self.assertEqual(journal.reserve(original)[1]['status'], 'accepted')
            conflict = json.loads(json.dumps(original))
            conflict['method'] = 'home'
            self.assertEqual(journal.reserve(conflict)[1]['reason'], 'command_id_conflict')
            journal.db.close()
        finally:
            Path(path).unlink(missing_ok=True)


class DispatcherTests(unittest.TestCase):
    def setUp(self):
        self.journal = CommandJournal(':memory:')
        self.results, self.calls = [], []
        self.session = 'session-1'
        self.available = True
        self.completed = threading.Event()
        self.gate = threading.Event()
        self.started = threading.Event()

    def tearDown(self):
        self.gate.set()

    def dispatcher(self, execute=None, enabled=True):
        def reply(rid, result):
            self.results.append((rid, result))
            self.completed.set()

        def default_execute(method, guard):
            self.calls.append(method)
            return True, 'command accepted'

        return Dispatcher(self.journal, execute or default_execute, reply, lambda result: None,
                          lambda: (self.session, self.available), enabled=enabled)

    def wait_for(self, rid):
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            matches = [result for request_id, result in self.results if request_id == rid]
            if matches:
                return matches[-1]
            self.completed.wait(.01)
            self.completed.clear()
        self.fail('worker did not reply')

    def test_disabled_retained_unavailable_never_call_ros(self):
        self.dispatcher(enabled=False).submit('1', request())
        self.dispatcher().submit('2', request(), retained=True)
        self.available = False
        self.dispatcher().submit('3', request())
        self.assertEqual(len(self.results), 3)
        self.assertFalse(self.calls)

    def test_duplicate_only_calls_service_once_and_accepted_is_not_done(self):
        dispatcher = self.dispatcher()
        document = request()
        dispatcher.submit('1', document)
        self.assertEqual(self.wait_for('1')['status'], 'accepted')
        dispatcher.submit('2', document)
        self.assertTrue(self.wait_for('2')['duplicate'])
        self.assertEqual(self.calls, ['start'])

    def test_blocked_motion_does_not_block_stop_or_accept_unbounded_workers(self):
        def execute(method, guard):
            self.calls.append(method)
            if method == 'start':
                self.started.set()
                self.gate.wait(2)
            return True, 'requested'

        dispatcher = self.dispatcher(execute)
        dispatcher.submit('1', request())
        self.assertTrue(self.started.wait(1))
        dispatcher.submit('2', request('home', '1123456789abcdef'))
        self.assertEqual(self.wait_for('2')['reason'], 'bridge_busy')
        dispatcher.submit('3', request('stop', '2123456789abcdef'))
        self.assertEqual(self.wait_for('3')['status'], 'accepted')
        self.assertEqual(self.calls, ['start', 'stop'])
        self.gate.set()
        self.wait_for('1')

    def test_connection_change_invalidates_waiting_motion(self):
        def execute(method, guard):
            self.started.set()
            self.gate.wait(2)
            if guard():
                self.calls.append(method)
                return True, 'accepted'
            return False, 'disconnected'

        dispatcher = self.dispatcher(execute)
        dispatcher.submit('1', request())
        self.assertTrue(self.started.wait(1))
        self.session = 'new-session'
        self.gate.set()
        self.assertEqual(self.wait_for('1')['status'], 'rejected')
        self.assertFalse(self.calls)

    def test_service_transport_failure_is_unknown_and_not_retried(self):
        def execute(method, guard):
            self.calls.append(method)
            raise IOError('service disconnected after request')

        dispatcher = self.dispatcher(execute)
        document = request()
        dispatcher.submit('1', document)
        self.assertEqual(self.wait_for('1')['status'], 'unknown')
        dispatcher.submit('2', document)
        self.assertEqual(self.wait_for('2')['status'], 'unknown')
        self.assertEqual(len(self.calls), 1)


if __name__ == '__main__':
    unittest.main()
