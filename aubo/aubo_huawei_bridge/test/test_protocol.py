import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest

PACKAGE = Path(__file__).resolve().parents[1]
for path in (PACKAGE / 'src', PACKAGE.parent / 'aubo_cloud_bridge/src'):
    sys.path.insert(0, str(path))
from aubo_cloud_bridge.protocol import Telemetry, encode_json, decode_json, validate_rpc
from aubo_huawei_bridge.iot import (credentials, property_report, decode_command, make_command,
                                    command_reply, topics, request_id, MAX_REPORT_BYTES)
from aubo_huawei_bridge.budget import PublishBudget


class ProtocolTests(unittest.TestCase):
    def test_official_authentication_vector(self):
        client_id, username, password = credentials('test_device', '12345678', '2025041401')
        self.assertEqual(client_id, 'test_device_0_0_2025041401')
        self.assertEqual(username, 'test_device')
        self.assertEqual(password, 'c75150e6cb841417396819e4d2ee4358a416344a03a083e3a8567074ddec820a')

    def test_commands_expiry_and_topics(self):
        now = 1794191363000
        command = make_command('start', 'a' * 32, now)
        document = decode_command(command)
        self.assertEqual(validate_rpc(document, 'a' * 32, now)[0], 'start')
        with self.assertRaises(ValueError):
            validate_rpc(document, 'a' * 32, now + 25001)
        with self.assertRaises(ValueError):
            validate_rpc(document, 'b' * 32, now)
        for mutation in ({'service_id': 'SDK'}, {'command_name': 'open_gripper'},
                         {'object_device_id': 'other-device'}, {'paras': []}):
            with self.assertRaises(ValueError):
                decode_command({**command, **mutation})
        for value in (True, now, '1e13', '-1794191363000'):
            bad = {**command, 'paras': {**command['paras'], 'issued_at_ms': value}}
            with self.assertRaises(ValueError):
                decode_command(bad)
        t = topics('test_device')
        self.assertEqual(request_id(t['request_prefix'] + 'abc-123', t['request_prefix']), 'abc-123')
        with self.assertRaises(ValueError):
            request_id(t['reply_prefix'] + 'abc-123', t['request_prefix'])
        self.assertEqual(command_reply({'status': 'unknown'})['result_code'], 2)

    def test_report_is_one_unit_valid_json_and_matches_product_model(self):
        spec = importlib.util.spec_from_file_location('huawei_model', PACKAGE / 'config/generate_model.py')
        model = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(model)
        telemetry = Telemetry()
        targets = {str(i): {'id': i, 'category': '饮料罐' * 8, 'status': 'ready',
                           'detection_confidence': .9} for i in range(200)}
        telemetry.update('target_cache', encode_json({'schema_version': 2, 'targets': targets}))
        for kind in ('failure', 'detection_summary'):
            telemetry.update(kind, '\x00"\\测试' * 200)
        report = property_report(telemetry.snapshot(), 'a' * 32, True, False,
            {'status': 'accepted', 'command_id': 'c' * 80, 'method': 'move_to_observation',
             'reason': '原因' * 100, 'message': '"\\成功' * 100, 'wall_stamp_ms': 1794191363000}, 1794191363000)
        self.assertLessEqual(len(encode_json(report).encode()), MAX_REPORT_BYTES)
        values = report['services'][0]['properties']
        definitions = {p['property_name']: p for p in model.service()['properties']}
        self.assertEqual(set(values), set(definitions))
        for name, definition in definitions.items():
            if definition['data_type'] == 'string':
                self.assertLessEqual(len(values[name].encode()), definition['max_length'], name)
        self.assertEqual(values['queue_total'], 200)
        self.assertEqual(values['target_cache_truncated'], 1)
        self.assertTrue(decode_json(values['target_cache'])['targets'])
        self.assertEqual(decode_json(values['last_command'])['status'], 'accepted')


class BudgetTests(unittest.TestCase):
    def test_restart_and_rolling_window_keep_response_reserve(self):
        now = [100000.0]
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / 'budget.sqlite3')
            budget = PublishBudget(path, limit=200, reply_reserve=10, clock=lambda: now[0])
            self.assertTrue(budget.take(190))
            self.assertFalse(budget.take())
            self.assertTrue(budget.take(10, reply=True))
            self.assertFalse(budget.take(reply=True))
            budget.db.close()
            budget = PublishBudget(path, limit=200, reply_reserve=10, clock=lambda: now[0])
            self.assertEqual(budget.used(), 200)
            self.assertFalse(budget.take())
            now[0] += 86399
            self.assertFalse(budget.take())
            now[0] += 2
            self.assertTrue(budget.take())
            self.assertEqual(budget.used(), 1)
            budget.db.close()


if __name__ == '__main__':
    unittest.main()
