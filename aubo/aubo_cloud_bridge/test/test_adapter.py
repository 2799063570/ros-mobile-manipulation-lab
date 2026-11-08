"""Run the actual ROS/MQTT adapter with synthetic endpoints, not a robot."""
from pathlib import Path
import runpy
import sys
import time
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from aubo_cloud_bridge.protocol import decode_json, RPC_PREFIX, RESPONSE_PREFIX, TELEMETRY_TOPIC


class AdapterTests(unittest.TestCase):
    def exercise_adapter(self, version_two):
        root = Path(__file__).resolve().parent
        token = root / (uuid.uuid4().hex + '.token')
        journal = root / (uuid.uuid4().hex + '.sqlite3')
        token.write_text('local-test-device-token', encoding='utf-8')
        client = Mock()
        client.is_connected.return_value = True
        mqtt = ModuleType('paho.mqtt.client')
        mqtt.Client = Mock(return_value=client)
        if version_two:
            mqtt.CallbackAPIVersion = SimpleNamespace(VERSION2=2)
        rospy = ModuleType('rospy')
        params = {'~journal_path': str(journal), '~token_file': str(token),
                  '~host': 'mqtt.thingsboard.cloud', '~enable_commands': True}
        rospy.get_param = lambda name, default=None: params.get(name, default)
        rospy.Subscriber = Mock()
        rospy.loginfo, rospy.logwarn, rospy.logwarn_throttle = Mock(), Mock(), Mock()
        rospy.get_name = lambda: '/aubo_cloud_bridge'
        rospy.is_shutdown = lambda: False
        rospy.wait_for_service = Mock()
        service = Mock(return_value=SimpleNamespace(success=True, message='command accepted'))
        rospy.ServiceProxy = Mock(return_value=service)
        rosgraph = ModuleType('rosgraph')
        services = [('/sorting/' + name, ['/task']) for name in ('start', 'stop', 'home', 'move_to_observation')]
        rosgraph.Master = Mock(return_value=SimpleNamespace(getSystemState=lambda: (
            [('/sorting/state', ['/task'])], [], services)))
        std_msgs = ModuleType('std_msgs.msg')
        std_msgs.String = object
        std_srvs = ModuleType('std_srvs.srv')
        std_srvs.Trigger = object
        modules = {'paho': ModuleType('paho'), 'paho.mqtt': ModuleType('paho.mqtt'),
                   'paho.mqtt.client': mqtt, 'rospy': rospy, 'rosgraph': rosgraph,
                   'std_msgs': ModuleType('std_msgs'), 'std_msgs.msg': std_msgs,
                   'std_srvs': ModuleType('std_srvs'), 'std_srvs.srv': std_srvs}
        bridge = None
        try:
            with patch.dict(sys.modules, modules):
                adapter = runpy.run_path(str(root.parent / 'scripts' / 'thingsboard_bridge'))
                bridge = adapter['Bridge']()
                client.tls_set.assert_called_once()
                self.assertNotEqual(client.tls_set.call_args.kwargs['cert_reqs'], 0)
                client.connect_async.assert_called_with('mqtt.thingsboard.cloud', 8883, 30)
                bridge.on_connect(client, None, {}, 0)
                first_session = bridge.session
                self.assertTrue(first_session)
                client.subscribe.assert_called_with(RPC_PREFIX + '+', qos=1)
                bridge.ros_message(SimpleNamespace(data='READY | waiting'), 'state')
                # Run one wall-clock telemetry cycle with ROS interfaces registered.
                with patch.object(bridge.shutdown, 'wait', side_effect=lambda _: bridge.shutdown.set()):
                    bridge.run()
                payloads = [decode_json(c.args[1]) for c in client.publish.call_args_list if c.args[0] == TELEMETRY_TOPIC]
                self.assertEqual(payloads[0]['state'], 'READY')
                self.assertTrue(payloads[0]['sorting_available'])
                bridge.shutdown.clear()
                now = int(time.time() * 1000)
                document = {'method': 'start', 'params': {'command_id': uuid.uuid4().hex,
                            'session': first_session, 'issued_at_ms': now, 'expires_at_ms': now + 10000}}
                from aubo_cloud_bridge.protocol import encode_json
                bridge.on_message(client, None, SimpleNamespace(topic=RPC_PREFIX + '123',
                    payload=encode_json(document).encode(), retain=False))
                deadline = time.monotonic() + 2
                while 'motion' in bridge.dispatcher.active and time.monotonic() < deadline:
                    time.sleep(.005)
                replies = [decode_json(c.args[1]) for c in client.publish.call_args_list if c.args[0] == RESPONSE_PREFIX + '123']
                self.assertEqual(replies[-1]['status'], 'accepted')
                service.assert_called_once()
                rospy.ServiceProxy.assert_called_with('/sorting/start', object)
                if version_two:
                    bridge.on_disconnect_v2(client, None, {}, 1, None)
                else:
                    bridge.on_disconnect_v1(client, None, 1)
                self.assertEqual(bridge.session, '')
                bridge.on_connect(client, None, {}, 0)
                self.assertNotEqual(first_session, bridge.session)
                bridge.close()
                bridge.dispatcher.journal.db.close()
        finally:
            token.unlink(missing_ok=True)
            journal.unlink(missing_ok=True)

    def test_paho_one_adapter(self):
        self.exercise_adapter(False)

    def test_paho_two_adapter(self):
        self.exercise_adapter(True)


if __name__ == '__main__':
    unittest.main()
