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
from aubo_huawei_bridge.iot import make_command


class AdapterTests(unittest.TestCase):
    def exercise(self, version_two, enabled):
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp) / 'device.json'
            config.write_text(encode_json(dict(device_id='local_robot', device_secret='test-secret',
                                               host='localhost', ca_file='')))
            params = {'~credentials_file': str(config), '~enable_commands': enabled,
                      '~journal_path': str(Path(tmp) / 'commands.sqlite3'),
                      '~budget_path': str(Path(tmp) / 'quota.sqlite3')}
            rospy = ModuleType('rospy')
            rospy.get_param = lambda key, default=None: params.get(key, default)
            rospy.Subscriber, rospy.Publisher = Mock(), Mock()
            rospy.loginfo, rospy.logwarn, rospy.logwarn_throttle = Mock(), Mock(), Mock()
            rospy.get_name = lambda: '/aubo_huawei_bridge'
            rospy.is_shutdown = lambda: False
            rospy.wait_for_service = Mock()
            service = Mock(return_value=SimpleNamespace(success=True, message='accepted locally'))
            rospy.ServiceProxy = Mock(return_value=service)
            rosgraph = ModuleType('rosgraph')
            rosgraph.Master = Mock(return_value=SimpleNamespace(getSystemState=lambda: (
                [('/sorting/state', ['/task'])], [], [('/sorting/' + method, ['/task']) for method
                    in ('start', 'stop', 'home', 'move_to_observation')])))
            mqtt = ModuleType('paho.mqtt.client')
            mqtt.MQTT_ERR_SUCCESS = 0
            client = Mock()
            client.is_connected.return_value = True
            client.publish.return_value.rc = 0
            mqtt.Client = Mock(return_value=client)
            if version_two:
                mqtt.CallbackAPIVersion = SimpleNamespace(VERSION2=2)
            std_msgs, std_srvs = ModuleType('std_msgs.msg'), ModuleType('std_srvs.srv')
            std_msgs.String = SimpleNamespace
            std_srvs.Trigger = object
            modules = {'rospy': rospy, 'rosgraph': rosgraph, 'paho': ModuleType('paho'),
                'paho.mqtt': ModuleType('paho.mqtt'), 'paho.mqtt.client': mqtt,
                'std_msgs': ModuleType('std_msgs'), 'std_msgs.msg': std_msgs,
                'std_srvs': ModuleType('std_srvs'), 'std_srvs.srv': std_srvs}
            with patch.dict(sys.modules, modules), patch('ssl.create_default_context') as tls:
                script = runpy.run_path(str(PACKAGE / 'scripts/huawei_bridge'))
                bridge = script['Bridge']()
                try:
                    tls.assert_called_once_with(cafile=None)
                    self.assertEqual(client.username_pw_set.call_args.args[0], 'local_robot')
                    self.assertNotEqual(client.username_pw_set.call_args.args[1], 'test-secret')
                    bridge.on_connect(client, None, {}, 0)
                    session = bridge.session
                    bridge.ros_message(SimpleNamespace(data='READY | test'), 'state')
                    with patch.object(bridge.shutdown, 'wait', side_effect=lambda _: bridge.shutdown.set()) as wait:
                        bridge.run()
                    wait.assert_called_once_with(1)  # Local health remains fast despite 30s cloud reports.
                    bridge.shutdown.clear()
                    reports = [decode_json(call.args[1]) for call in client.publish.call_args_list
                               if call.args[0] == bridge.topics['report']]
                    self.assertEqual(reports[0]['services'][0]['properties']['state'], 'READY')
                    status = decode_json(bridge.status_publisher.publish.call_args.args[0].data)
                    self.assertEqual(status['publish_interval'], 30)
                    self.assertEqual(status['session'], session)
                    self.assertEqual(bridge.budget.used(), 1)
                    command = make_command('start', session, int(time.time() * 1000))

                    def submit(command, retained=False):
                        bridge.on_message(client, None, SimpleNamespace(
                            topic=bridge.topics['request_prefix'] + 'cloud-request-1',
                            payload=encode_json(command).encode(), retain=retained))

                    def reply():
                        deadline = time.monotonic() + 2
                        while time.monotonic() < deadline:
                            replies = [decode_json(call.args[1]) for call in client.publish.call_args_list
                                       if call.args[0].startswith(bridge.topics['reply_prefix'])]
                            if replies:
                                return replies[-1]
                            time.sleep(.005)
                        self.fail('no command reply')

                    client.publish.reset_mock()
                    submit(command)
                    first = reply()
                    self.assertEqual(first['result_code'], 0 if enabled else 1)
                    self.assertEqual(service.call_count, int(enabled))
                    client.publish.reset_mock()
                    submit(command)
                    duplicate = reply()
                    if enabled:
                        self.assertTrue(decode_json(duplicate['paras']['result'])['duplicate'])
                    self.assertEqual(service.call_count, int(enabled))
                    client.publish.reset_mock()
                    submit(command, retained=True)
                    self.assertEqual(reply()['result_code'], 1)
                    if version_two:
                        bridge.on_disconnect_v2(client, None, {}, 1, None)
                    else:
                        bridge.on_disconnect_v1(client, None, 1)
                    self.assertEqual(bridge.session, '')
                    bridge.on_connect(client, None, {}, 0)
                    self.assertNotEqual(session, bridge.session)
                    client.publish.reset_mock()
                    submit(command)
                    self.assertEqual(reply()['result_code'], 1)
                    self.assertEqual(service.call_count, int(enabled))
                    client.publish.reset_mock()
                    with patch.object(bridge.budget, 'take', return_value=False):
                        submit(make_command('home', bridge.session, int(time.time() * 1000)))
                    client.publish.assert_not_called()
                    self.assertEqual(service.call_count, int(enabled))
                finally:
                    bridge.close()
                    bridge.journal.db.close()
                    bridge.budget.db.close()

    def test_paho_v1_read_only(self):
        self.exercise(False, False)

    def test_paho_v2_commands_budget_and_connection_guards(self):
        self.exercise(True, True)


if __name__ == '__main__':
    unittest.main()
