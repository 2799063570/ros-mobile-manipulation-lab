import importlib.util
from pathlib import Path
import sys
import unittest

PACKAGE = Path(__file__).resolve().parents[1]
for path in (PACKAGE / 'src', PACKAGE.parent / 'aubo_cloud_bridge/src'):
    sys.path.insert(0, str(path))
from aubo_cloud_bridge.protocol import Telemetry, decode_json, encode_json
from aubo_tencent_bridge.iot import properties, action_request, action_reply, topics


class IoTTests(unittest.TestCase):
    def test_properties_match_model_and_bound_unicode_without_breaking_json(self):
        spec = importlib.util.spec_from_file_location('thing_model', PACKAGE / 'config/generate_thing_model.py')
        model = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(model)
        telemetry = Telemetry()
        targets = {str(i): {'id': i, 'category': '饮料罐' * 10,
                   'status': 'ready', 'detection_confidence': .9} for i in range(200)}
        telemetry.update('target_cache', encode_json({'schema_version': 2, 'targets': targets}))
        telemetry.update('failure', '测试故障' * 500)
        result = properties(telemetry.snapshot(), 'a' * 32, True, False,
            {'command_id': 'b' * 32, 'status': 'accepted', 'message': '成功' * 1000}, 1794191363000)
        definitions = {p['id']: p['define'] for p in model.build()['properties']}
        for prop in model.build()['properties']:
            self.assertLessEqual(len(prop['name']), 20)
            self.assertLessEqual(len(prop['id']), 32)
        self.assertEqual(set(result), set(definitions))
        for key, value in result.items():
            define = definitions[key]
            if define['type'] == 'string':
                self.assertLessEqual(len(value.encode('utf-8')), int(define['max']), key)
            elif define['type'] == 'bool':
                self.assertIn(value, (0, 1))
            else:
                self.assertGreaterEqual(value, float(define['min']))
                self.assertLessEqual(value, float(define['max']))
        self.assertEqual(result['queue_total'], 200)
        self.assertEqual(result['target_cache_truncated'], 1)
        self.assertEqual(result['target_cache_shown'], len(decode_json(result['target_cache'])['targets']))
        self.assertEqual(decode_json(result['last_command'])['status'], 'accepted')
        self.assertFalse(decode_json(result['bridge_context'])['enabled'])

    def test_action_envelopes_and_rejections(self):
        document = {'method': 'start', 'params': {}}
        envelope = {'method': 'action', 'actionId': 'task_command', 'clientToken': 'req::123',
                    'params': {'request': encode_json(document)}}
        self.assertEqual(action_request(envelope), ('req::123', document))
        reply = action_reply('req::123', {'status': 'accepted', 'message': 'ok'})
        self.assertEqual(decode_json(reply['response']['result'])['status'], 'accepted')
        self.assertNotIn('params', reply)
        for mutation in ({'actionId': 'arbitrary_service'}, {'clientToken': '../bad'},
                         {'params': {'request': '[]'}}, {'params': {'request': 'x' * 1025}},
                         {'params': {'request': '{}', 'extra': True}}):
            with self.assertRaises(ValueError):
                action_request({**envelope, **mutation})
        self.assertEqual(topics('PRODUCT', 'robot')['request'], '$thing/down/action/PRODUCT/robot')


if __name__ == '__main__':
    unittest.main()
