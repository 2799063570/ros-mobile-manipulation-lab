"""No ROS master or robot. Exercise preview, units, staleness and explicit IO."""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import math
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from python_qt_binding.QtWidgets import QApplication
from aubo_sorting_rqt import servo_model as model
from aubo_sorting_rqt.servo_widget import ServoWidget
from aubo_sorting_rqt.servo_bridge import ServoBridge

APP = QApplication.instance() or QApplication([])


class GeometryTest(unittest.TestCase):
    def test_3d_segments_and_missing_geometry(self):
        self.assertAlmostEqual(model.segment_distance((0, 0, 0), (1, 0, 0),
                                                     (.5, -1, 0), (.5, 1, 0)), 0.)
        self.assertAlmostEqual(model.segment_distance((0, 0, 0), (1, 0, 0),
                                                     (0, 0, .2), (1, 0, .2)), .2)
        self.assertAlmostEqual(model.segment_distance((0, 0, 0), (0, 0, 0),
                                                     (1, 0, 0), (1, 0, 0)), 1.)
        with self.assertRaises(ValueError): model.sight_clearance((0, 0, 1), (0, 0, 0), [])
        with self.assertRaises(ValueError): model.vector([float('nan'), 0, 0])

    def test_offset_is_away_from_camera_and_yaw_aligns_long_axis(self):
        camera = (-.029, -.652, .909)
        for target in ((.1, -.8, .05), (-.2, -.4, .04)):
            draft = model.visibility_draft(camera, target)
            dx, dy = camera[0]-target[0], camera[1]-target[1]
            self.assertLess(dx*draft['target_offset_x']+dy*draft['target_offset_y'], 0)
            self.assertAlmostEqual(math.hypot(draft['target_offset_x'], draft['target_offset_y']), .08)
            self.assertAlmostEqual(model.tool_axis((draft['desired_roll'], draft['desired_pitch'], draft['desired_yaw']))[2], -1.)
            y = draft['desired_yaw']
            alignment = (math.sin(y)*dx-math.cos(y)*dy)/math.hypot(dx, dy)
            self.assertAlmostEqual(abs(alignment), 1.)
        with self.assertRaises(ValueError): model.visibility_draft((0, 0, 1), (0, 0, 0))

    def test_selection_pending_and_source_ages_block_new_start(self):
        now = time.monotonic()
        data = dict(connected=True, state='DISABLED', status_at=now, config={'target_timeout': .2},
                    status={'feedback_valid': True, 'target_valid': True, 'feedback_age': .01,
                            'target_age': .19}, services={'set_enabled'})
        self.assertTrue(model.action_gates(data, now)['start'])
        self.assertFalse(model.action_gates(data, now+.03)['start'])
        data['selection_pending'] = True
        self.assertFalse(model.action_gates(data, now)['start'])

    def test_saved_observation_can_be_enabled_without_a_visible_target(self):
        now = time.monotonic()
        data = dict(connected=True, state='DISABLED', status_at=now, config={'target_timeout': .2},
                    status={'feedback_valid': True, 'target_valid': False},
                    services={'set_enabled'}, cell={'initial_search_enabled': True})
        self.assertTrue(model.action_gates(data, now)['start'])
        data['status']['feedback_valid'] = False
        self.assertFalse(model.action_gates(data, now)['start'])


class ServoPanelTest(unittest.TestCase):
    def setUp(self):
        self.widget = ServoWidget()
        self.config = {key: 0. for key, *_ in model.PARAMETERS}
        self.config.update(linear_gain=.3, max_linear_velocity=.01, target_offset_z=.15,
                           desired_roll=math.pi, desired_yaw=math.pi/2,
                           use_orientation_control=False, target_timeout=.2, dls_lambda=.04)
        self.widget.configuration(self.config, force=True)

    def tearDown(self):
        self.widget.close(); self.widget.deleteLater(); APP.processEvents()

    def data(self, state='DISABLED'):
        return dict(connected=True, state=state, status_at=time.monotonic(), config=self.config,
                    services={'set_enabled', 'reset'}, perception_controls=True, pending=False,
                    status=dict(feedback_valid=True, target_valid=True, error_valid=True,
                                feedback_age=.01, target_age=.08, tcp=(.1, -.7, .2),
                                goal=(.12, -.68, .21), error=(.02, .02, .01)),
                    visibility=dict(camera=(-.03, -.65, .91), target=(.12, -.68, .06),
                                    links=[(0, 0, 0), (.1, -.7, .2)], rotation=(1., 0., 0., 0.),
                                    clearance=.04, at=time.monotonic()))

    def test_stop_survives_pending_and_stale_telemetry_disables_new_motion(self):
        data = self.data(); self.widget.render(data)
        self.assertTrue(self.widget.buttons['start'].isEnabled())
        data.update(state='TRACKING', pending=True)
        self.widget.render(data)
        self.assertFalse(self.widget.buttons['start'].isEnabled())
        self.assertFalse(self.widget.pose_apply.isEnabled())
        self.assertTrue(self.widget.buttons['stop'].isEnabled())
        data.update(status_at=time.monotonic()-2, pending=False)
        self.widget.render(data)
        self.assertFalse(self.widget.control_apply.isEnabled())
        self.assertIn('无效', self.widget.health_label.text())
        self.assertIsNone(self.widget.plot.samples[-1][1])

    def test_preview_and_readback_never_apply_and_only_dirty_group_is_sent(self):
        writes = []
        self.widget.parameters_requested.connect(writes.append)
        self.widget.render(self.data())
        self.widget._visibility_draft()
        self.assertEqual(writes, [])
        self.assertIn('尚未发送', self.widget.parameter_note.text())
        self.widget.editors['max_linear_velocity'].setValue(12.)
        self.widget.control_apply.click()
        self.assertEqual(writes[-1], {'max_linear_velocity': .012})
        self.widget.pose_apply.click()
        self.assertIn('desired_yaw', writes[-1])
        self.assertNotIn('max_linear_velocity', writes[-1])
        draft_yaw = self.widget.editors['desired_yaw'].value()
        self.widget.configuration(dict(self.config, desired_yaw=0.))
        self.assertEqual(self.widget.editors['desired_yaw'].value(), draft_yaw)
        self.assertEqual(self.widget.actual['desired_yaw'].text(), '0.0')
        self.widget.configuration(self.config, force=True)
        self.assertEqual(self.widget._dirty, set())

    def test_out_of_range_server_values_are_displayed_without_hidden_clamping(self):
        self.widget.configuration(dict(self.config, max_linear_velocity=.2), force=True)
        self.assertEqual(self.widget.editors['max_linear_velocity'].value(), 200.)
        self.assertEqual(self.widget._dirty, set())

    def test_real_widget_preview_is_bounded_and_grabs_offscreen(self):
        for i in range(650): self.widget.render(self.data())
        self.assertLessEqual(len(self.widget.plot.samples), 600)
        self.widget.show(); APP.processEvents()
        self.assertFalse(self.widget.grab().isNull())
        self.widget.grab().save('/tmp/aubo_visual_servo_rqt_preview.png')


class BridgeTest(unittest.TestCase):
    def bridge(self):
        patches = [mock.patch('aubo_sorting_rqt.servo_bridge.rospy.Subscriber'),
                   mock.patch('aubo_sorting_rqt.servo_bridge.rospy.Publisher'),
                   mock.patch('aubo_sorting_rqt.servo_bridge.tf2_ros.Buffer'),
                   mock.patch('aubo_sorting_rqt.servo_bridge.tf2_ros.TransformListener'),
                   mock.patch.object(ServoBridge, '_thread')]
        for patch in patches: patch.start(); self.addCleanup(patch.stop)
        return ServoBridge('/servo', '/visual_servo', '/image', '/geometry')

    def test_open_close_does_not_send_motion_parameters_or_selection(self):
        with mock.patch('aubo_sorting_rqt.servo_bridge.rospy.ServiceProxy') as rpc:
            bridge = self.bridge()
            bridge.close()
            rpc.assert_not_called()
            bridge._selection.publish.assert_not_called()

    def test_stop_has_worker_independent_of_blocked_parameter_request(self):
        bridge = self.bridge()
        bridge._data.update(state='DISABLED', status_at=time.monotonic(), config={'linear_gain': .3},
                            status={'feedback_valid': True, 'target_valid': True},
                            services={'set_enabled', 'reset'})
        for sub in bridge._subscribers: sub.get_num_connections.return_value = 1
        entered, release, stopped = threading.Event(), threading.Event(), threading.Event()
        def update(values):
            entered.set(); release.wait(3); return values
        bridge._client = mock.Mock()
        bridge._client.update_configuration.side_effect = update
        bridge._thread = lambda call: threading.Thread(target=call, daemon=True).start()
        def rpc(service, kind):
            def call(*args):
                if service.endswith('/set_enabled') and args == (False,): stopped.set()
                return NS(success=True, message='accepted')
            return call
        with mock.patch('aubo_sorting_rqt.servo_bridge.rospy.wait_for_service'), \
             mock.patch('aubo_sorting_rqt.servo_bridge.rospy.ServiceProxy', side_effect=rpc):
            try:
                bridge.apply({'linear_gain': .4})
                self.assertTrue(entered.wait(1))
                bridge.command('stop')
                self.assertTrue(stopped.wait(1), 'stop was blocked by parameter RPC')
            finally:
                release.set(); bridge.close()

    def test_class_change_waits_for_matching_new_source_and_telemetry(self):
        import rospy
        from aubo_ros_control.msg import VisualServoStatus
        from std_msgs.msg import String
        bridge = self.bridge()
        for sub in bridge._subscribers: sub.get_num_connections.return_value = 1
        bridge._data.update(state='DISABLED', status_at=time.monotonic(), config={'target_timeout': .2},
                            status={'feedback_valid': True, 'target_valid': True}, perception_controls=True)
        with mock.patch.object(rospy.Time, 'now', return_value=rospy.Time(20)):
            bridge.select_class('Bottle')
            self.assertEqual(bridge._selection.publish.call_args[0][0].data, 'bottle')
            bridge._perception(String(data='DETECTED|bottle|'))
            bridge._target_label(String(data='bottle'))
            status = VisualServoStatus()
            status.header.stamp = rospy.Time(20)
            status.target_valid = status.feedback_valid = True
            status.target_age = .1  # Source still predates the switch.
            bridge._status(status)
            self.assertTrue(bridge.snapshot()['selection_pending'])
        with mock.patch.object(rospy.Time, 'now', return_value=rospy.Time(21)):
            status.header.stamp = rospy.Time(21)
            bridge._status(status)
            self.assertFalse(bridge.snapshot()['selection_pending'])
        bridge.close()


if __name__ == '__main__': unittest.main()
