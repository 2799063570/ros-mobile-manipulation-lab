from pathlib import Path
import runpy
import sys
import tempfile
import time
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

PACKAGE = Path(__file__).resolve().parents[1]
for path in (PACKAGE / 'src', PACKAGE.parent / 'aubo_cloud_bridge/src'):
    sys.path.insert(0, str(path))
from aubo_cloud_bridge.protocol import encode_json, decode_json


class AdapterTests(unittest.TestCase):
    def exercise(self, version_two, enabled):
        with tempfile.TemporaryDirectory() as tmp:
            credentials = Path(tmp) / 'device.json'
            credentials.write_text(encode_json(dict(product_id='PRODUCT', device_name='robot',
                host='localhost', ca_file='local-ca', cert_file='local-cert', key_file='local-key')))
            params = {'~credentials_file': str(credentials), '~journal_path': str(Path(tmp) / 'journal.sqlite3'),
                      '~enable_commands': enabled}
            rospy = ModuleType('rospy')
            rospy.get_param = lambda name, default=None: params.get(name, default)
            rospy.Subscriber = Mock()
            rospy.loginfo, rospy.logwarn, rospy.logwarn_throttle = Mock(), Mock(), Mock()
            rospy.get_name = lambda: '/aubo_tencent_bridge'
            rospy.is_shutdown = lambda: False
            rospy.wait_for_service = Mock()
            service = Mock(return_value=SimpleNamespace(success=True, message='requested'))
            rospy.ServiceProxy = Mock(return_value=service)
            rosgraph = ModuleType('rosgraph')
            rosgraph.Master = Mock(return_value=SimpleNamespace(getSystemState=lambda: (
                [('/sorting/state', ['/task'])], [], [('/sorting/' + name, ['/task'])
                for name in ('start', 'stop', 'home', 'move_to_observation') ])))
            client = Mock()
            client.is_connected.return_value = True
            mqtt = ModuleType('paho.mqtt.client')
            mqtt.Client = Mock(return_value=client)
            if version_two:
                mqtt.CallbackAPIVersion = SimpleNamespace(VERSION2=2)
            std_msgs, std_srvs = ModuleType('std_msgs.msg'), ModuleType('std_srvs.srv')
            std_msgs.String, std_srvs.Trigger = object, object
            modules = {'rospy': rospy, 'rosgraph': rosgraph, 'paho': ModuleType('paho'),
                'paho.mqtt': ModuleType('paho.mqtt'), 'paho.mqtt.client': mqtt,
                'std_msgs': ModuleType('std_msgs'), 'std_msgs.msg': std_msgs,
                'std_srvs': ModuleType('std_srvs'), 'std_srvs.srv': std_srvs}
            with patch.dict(sys.modules, modules), patch('ssl.create_default_context') as tls:
                adapter = runpy.run_path(str(PACKAGE / 'scripts/tencent_bridge'))
                bridge = adapter['Bridge']()
                try:
                    tls.assert_called_once_with(cafile='local-ca')
                    tls.return_value.load_cert_chain.assert_called_once_with('local-cert', 'local-key')
                    client.connect_async.assert_called_once_with('localhost', 8883, 30)
                    self.assertEqual(mqtt.Client.call_args.kwargs['client_id'], 'PRODUCTrobot')
                    bridge.on_connect(client, None, {}, 0)
                    session = bridge.session
                    bridge.ros_message(SimpleNamespace(data='READY | test'), 'state')
                    with patch.object(bridge.shutdown, 'wait', side_effect=lambda _: bridge.shutdown.set()):
                        bridge.run()
                    bridge.shutdown.clear()
                    reports = [decode_json(c.args[1]) for c in client.publish.call_args_list
                               if c.args[0] == bridge.topics['report']]
                    self.assertEqual(reports[0]['params']['state'], 'READY')
                    self.assertEqual(reports[0]['params']['sorting_available'], 1)
                    self.assertEqual(len(bridge.subscribers), 5)
                    now = int(time.time() * 1000)
                    document = {'method': 'start', 'params': {'command_id': 'a' * 32, 'session': session,
                        'issued_at_ms': now, 'expires_at_ms': now + 10000}}
                    envelope = {'method': 'action', 'actionId': 'task_command', 'clientToken': 'cloud::1',
                                'params': {'request': encode_json(document)}}
                    def submit(data, retained=False):
                        bridge.on_message(client, None, SimpleNamespace(topic=bridge.topics['request'],
                            payload=encode_json(data).encode(), retain=retained))
                    def reply():
                        deadline = time.monotonic() + 2
                        while time.monotonic() < deadline:
                            replies = [decode_json(c.args[1]) for c in client.publish.call_args_list
                                       if c.args[0] == bridge.topics['reply']]
                            if replies:
                                return decode_json(replies[-1]['response']['result'])
                            time.sleep(.005)
                        self.fail('no action reply')
                    submit(envelope)
                    self.assertEqual(reply()['status'], 'accepted' if enabled else 'rejected')
                    self.assertEqual(service.call_count, int(enabled))
                    client.publish.reset_mock()
                    submit(envelope, retained=True)
                    self.assertEqual(reply()['status'], 'rejected')
                    self.assertEqual(service.call_count, int(enabled))
                    if version_two:
                        bridge.on_disconnect_v2(client, None, {}, 1, None)
                    else:
                        bridge.on_disconnect_v1(client, None, 1)
                    self.assertEqual(bridge.session, '')
                    bridge.on_connect(client, None, {}, 0)
                    self.assertNotEqual(session, bridge.session)
                    client.publish.reset_mock()
                    submit(envelope)
                    self.assertEqual(reply()['status'], 'rejected')
                    self.assertEqual(service.call_count, int(enabled))
                finally:
                    bridge.close()
                    bridge.journal.db.close()

    def test_paho_v1_read_only(self):
        self.exercise(False, False)

    def test_paho_v2_command_and_session_guards(self):
        self.exercise(True, True)


if __name__ == '__main__':
    unittest.main()
