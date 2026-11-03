"""Offline UI/ROS-boundary checks. No master, robot connection or motion RPC."""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import json
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from python_qt_binding.QtWidgets import QApplication
from aubo_sorting_rqt import model
from aubo_sorting_rqt.widget import SortingWidget
from aubo_sorting_rqt.bridge import RosBridge

APP = QApplication.instance() or QApplication([])


class PanelTest(unittest.TestCase):
    def setUp(self):
        self.widget = SortingWidget()

    def tearDown(self):
        self.widget.close()
        self.widget.deleteLater()
        APP.processEvents()

    def data(self, state='READY'):
        return dict(state=state, connected=True, services={key for key, _ in model.ACTIONS},
                    rows=[], config={'velocity_scaling': .1, 'acceleration_scaling': .1},
                    image_at=0, hardware_at=0, cell={})

    def test_motion_requires_connection_and_ready_start_but_stop_stays_available(self):
        for state in ('INITIALIZING', 'PICKING', 'OBSERVING', 'DETECTING'):
            self.widget.render(self.data(state))
            self.assertFalse(self.widget.buttons['start'].isEnabled())
            self.assertFalse(self.widget.buttons['move_to_observation'].isEnabled())
            self.assertTrue(self.widget.buttons['stop'].isEnabled())
        data = self.data('IDLE')
        self.widget.render(data)
        self.assertTrue(self.widget.buttons['move_to_observation'].isEnabled())
        self.assertFalse(self.widget.buttons['start'].isEnabled())
        data['state'] = 'READY'
        self.widget.render(data)
        self.assertTrue(self.widget.buttons['start'].isEnabled())
        data['cell'] = {'continuous_sorting': False}
        self.widget.render(data)
        self.assertFalse(self.widget.buttons['start'].isEnabled())
        data['cell'] = {'continuous_sorting': True}
        data.update(pending=True)
        self.widget.render(data)
        self.assertFalse(self.widget.buttons['start'].isEnabled())
        self.assertTrue(self.widget.buttons['stop'].isEnabled())
        data.update(pending=False, hardware_at=time.monotonic(),
                    hardware={'feedback_timeout_latched': 'true'})
        self.widget.render(data)
        self.assertFalse(self.widget.buttons['start'].isEnabled())
        data['connected'] = False
        self.widget.render(data)
        self.assertTrue(all(not button.isEnabled() for button in self.widget.buttons.values()))

    def test_edit_draft_survives_updates_and_server_values_are_restored(self):
        self.widget.configuration({'velocity_scaling': .1}, force=True)
        self.widget.editors['velocity_scaling'].setValue(.2)
        self.widget.configuration({'velocity_scaling': .12})
        self.assertEqual(self.widget.editors['velocity_scaling'].value(), .2)
        self.assertEqual(self.widget.actual['velocity_scaling'].text(), '0.12')
        self.widget.configuration({'velocity_scaling': .12}, force=True)
        self.assertEqual(self.widget.editors['velocity_scaling'].value(), .12)
        self.widget.parameter_note.setText('服务器未接受部分编辑值')
        self.widget.render(self.data())
        self.assertEqual(self.widget.parameter_note.text(), '服务器未接受部分编辑值')
        self.assertEqual(model.parameter_changes({'velocity_scaling': .2}, {'velocity_scaling': .12}), ['velocity_scaling'])

    def test_queue_renders_numeric_order_with_multiple_instances_and_process_phase(self):
        document = {'frame_id': 'base_link', 'targets': {
            str(i): {'id': i, 'category': category, 'position': [-.2, -.6, .04],
                     'status': status, 'observations': 8, 'age': .1, 'detection_confidence': .85}
            for i, category, status in [(10, 'can', 'ready'), (2, 'bottle', 'reserved'), (3, 'can', 'candidate')]}}
        frame, rows = model.queue_rows(json.dumps(document))
        self.assertEqual([row['id'] for row in rows], [2, 3, 10])
        data = self.data('PICKING | bottle #2')
        data.update(rows=rows, frame=frame, progress={'phase': 'RELEASE', 'description': 'bottle'})
        self.widget.render(data)
        self.assertEqual(self.widget.table.rowCount(), 3)
        self.assertEqual(self.widget.table.item(0, 2).text(), '执行中')
        self.assertIn('释放目标', self.widget.action_detail.text())
        self.assertEqual(model.process_step('PICKING', data['progress']), 3)
        data.update(cell={'continuous_sorting': True}, vision_phase='approach',
                    target_occluded=True, merged_detections_total=12)
        self.widget.render(data)
        self.assertIn('遮挡保护', self.widget.mode.text())
        self.assertIn('12', self.widget.mode.text())
        data.update(vision_phase='masked')
        self.widget.render(data)
        self.assertIn('抓放阶段屏蔽', self.widget.mode.text())
        self.assertIn('检测置信度', self.widget.table.horizontalHeaderItem(8).text())
        self.assertEqual(self.widget.table.item(0,8).text(),'0.85')
        self.assertEqual(self.widget.table.item(0,6).text(), '—')
        self.assertEqual(self.widget.table.item(0,7).text(), '—')
        self.widget.show()
        APP.processEvents()
        self.assertFalse(self.widget.grab().isNull())

    def test_bad_queue_geometry_is_rejected(self):
        for document in ({}, {'targets': []}, {'targets': {'1': {'position': [float('nan'), 0., 0.]}}}):
            with self.assertRaises((ValueError, TypeError)):
                model.queue_rows(document)

    def test_planner_selection_applies_strings_and_preserves_draft_until_reply(self):
        data = self.data()
        data.update(planners={'ompl': ['RRTConnect', 'RRTstar'], 'nsg': ['NSG']}, default_pipeline='ompl')
        data['config'].update(planning_pipeline='', planner_id='')
        self.widget.render(data)
        self.assertTrue(self.widget.pipeline_combo.isEnabled())
        self.assertGreaterEqual(self.widget.planner_combo.findData('RRTConnect'), 0)
        with mock.patch.object(self.widget, '_fill_planners', wraps=self.widget._fill_planners) as fill:
            self.widget.render(data)
            fill.assert_not_called()  # Periodic rendering must not reset an open popup.
        self.widget.pipeline_combo.setCurrentIndex(self.widget.pipeline_combo.findData('nsg'))
        self.widget.planner_combo.setCurrentIndex(self.widget.planner_combo.findData('NSG'))
        self.widget.render(data)
        self.assertEqual(self.widget.pipeline_combo.currentData(), 'nsg')
        self.assertEqual(self.widget.planner_combo.currentData(), 'NSG')
        self.assertIn('MoveIt 默认', self.widget.planner_actual.text())
        requests = []
        self.widget.parameters_requested.connect(requests.append)
        self.widget.apply_button.click()
        self.assertEqual(requests[-1]['planning_pipeline'], 'nsg')
        self.assertEqual(requests[-1]['planner_id'], 'NSG')
        self.widget.configuration(data['config'], force=True)
        self.assertEqual(self.widget.pipeline_combo.currentData(), '')
        self.assertEqual(self.widget.planner_combo.currentData(), '')
        self.assertEqual(model.parameter_changes({'planner_id': 'NSG'}, {'planner_id': ''}), ['planner_id'])
        self.assertEqual(model.parameter_changes({'planner_id': ''}, {'planner_id': ''}), [])

    def test_algorithm_controls_require_loaded_planners_idle_and_new_backend(self):
        data = self.data()
        data.update(planners={'ompl': ['RRTConnect']}, default_pipeline='ompl')
        self.widget.render(data)
        self.assertFalse(self.widget.pipeline_combo.isEnabled())
        data['config'].update(planning_pipeline='ompl', planner_id='RRTConnect')
        self.widget.render(data)
        self.assertTrue(self.widget.pipeline_combo.isEnabled())
        for update in ({'state': 'PICKING'}, {'pending': True}, {'connected': False},
                       {'planners': {}}, {'external_planning_service': '/external'},
                       {'hardware_at': time.monotonic(), 'hardware': {'stream_fault_latched': 'true'}}):
            self.widget.render(dict(data, **update))
            self.assertFalse(self.widget.pipeline_combo.isEnabled(), update)
            self.assertFalse(self.widget.planner_combo.isEnabled(), update)

    def test_planner_options_filter_other_groups_and_use_short_ids(self):
        options = model.planner_options([
            NS(pipeline_id='ompl', planner_ids=['aubo_i5', 'aubo_i5[RRTConnect]',
                                               'gripper[RRT]', 'aubo_i5[RRTstar]']),
            NS(pipeline_id='nsg', planner_ids=['NSG'])], 'aubo_i5')
        self.assertEqual(options, {'ompl': ['RRTConnect', 'RRTstar'], 'nsg': ['NSG']})

    def test_cached_grasp_geometry_uses_degrees_and_millimeters_with_unknown_values(self):
        cases = [
            ({'grasp_geometry_valid': True, 'grasp_angle': math.pi/6, 'grasp_width': .064}, ('30.0', '64.0')),
            ({'grasp_geometry_valid': True, 'grasp_angle': 0., 'grasp_width': .01}, ('0.0', '10.0')),
            ({'grasp_geometry_valid': True, 'grasp_angle': -math.pi/4, 'grasp_width': .035}, ('-45.0', '35.0')),
            ({'grasp_geometry_valid': False, 'grasp_angle': 0., 'grasp_width': .064}, ('—', '—')),
            ({'grasp_geometry_valid': True, 'grasp_angle': float('nan'), 'grasp_width': -1.}, ('—', '—')),
            ({'grasp_geometry_valid': True, 'grasp_angle': None, 'grasp_width': float('inf')}, ('—', '—')),
            ({'grasp_geometry_valid': True, 'grasp_angle': 'bad', 'grasp_width': 0.}, ('—', '—')),
            ({}, ('—', '—')),
        ]
        for geometry, expected in cases:
            with self.subTest(geometry=geometry):
                row = dict(id=1, category='can', position=[-.06, -.67, .035], status='ready', **geometry)
                _, rows = model.queue_rows({'targets': {'1': row}})
                data = self.data()
                data['rows'] = rows
                self.widget.render(data)
                self.assertEqual(self.widget.table.item(0, 6).text(), expected[0])
                self.assertEqual(self.widget.table.item(0, 7).text(), expected[1])
        self.assertIn('°', self.widget.table.horizontalHeaderItem(6).text())
        self.assertIn('mm', self.widget.table.horizontalHeaderItem(7).text())


class BridgeTest(unittest.TestCase):
    def setUp(self):
        self.subscribe = mock.patch('aubo_sorting_rqt.bridge.rospy.Subscriber',
                                    side_effect=lambda *a, **k: NS(unregister=lambda: None, get_num_connections=lambda: 1))
        self.thread = mock.patch.object(RosBridge, '_thread')
        self.subscribe.start()
        self.thread.start()
        self.bridge = RosBridge('/test_task', '/test_sorting', '/test_image', '/test_hardware')

    def tearDown(self):
        self.bridge.close()
        self.thread.stop()
        self.subscribe.stop()

    def test_construction_and_close_never_send_commands_or_update_parameters(self):
        with mock.patch('aubo_sorting_rqt.bridge.rospy.ServiceProxy') as proxy:
            self.bridge.snapshot()
            self.bridge.close()
            self.assertFalse(self.bridge.snapshot()['connected'])
            proxy.assert_not_called()

    def test_stop_is_not_serialized_behind_pending_start_rpc(self):
        self.bridge._jobs.add('start')
        jobs = []
        self.bridge._thread = jobs.append
        self.bridge.command('stop')
        self.assertEqual(len(jobs), 1)
        with mock.patch('aubo_sorting_rqt.bridge.rospy.wait_for_service'), \
                mock.patch('aubo_sorting_rqt.bridge.rospy.ServiceProxy') as proxy:
            proxy.return_value.return_value = NS(success=True, message='stopped')
            jobs[0]()
            self.assertEqual(proxy.call_args.args[0], '/test_sorting/stop')

    def test_rejected_parameters_use_server_reply_and_report_rejection(self):
        jobs, events = [], []
        self.bridge._thread = jobs.append
        self.bridge._client = mock.Mock()
        self.bridge._client.update_configuration.return_value = {'velocity_scaling': .1}
        self.bridge.event.connect(lambda kind, value: events.append((kind, value)))
        self.bridge.apply({'velocity_scaling': .3})
        jobs[0]()
        applied = next(value for kind, value in events if kind == 'applied')
        self.assertEqual(applied['rejected'], ['velocity_scaling'])
        self.assertEqual(applied['accepted']['velocity_scaling'], .1)

    def test_planner_discovery_is_read_only_and_clears_stale_options_on_failure(self):
        with mock.patch('aubo_sorting_rqt.bridge.rospy.get_param',
                        side_effect=lambda name, default: {'/test_task/planning_group': 'aubo_i5',
                                                          '/move_group/default_planning_pipeline': 'nsg'}.get(name, default)), \
                mock.patch('aubo_sorting_rqt.bridge.rospy.wait_for_service'), \
                mock.patch('aubo_sorting_rqt.bridge.rospy.ServiceProxy') as proxy:
            proxy.return_value.return_value = NS(planner_interfaces=[NS(pipeline_id='nsg', planner_ids=['NSG'])])
            self.bridge._discover_planners()
            self.assertEqual(proxy.call_args.args[0], '/query_planner_interface')
            self.assertEqual(self.bridge.snapshot()['planners'], {'nsg': ['NSG']})
            self.assertEqual(self.bridge.snapshot()['default_pipeline'], 'nsg')
            proxy.return_value.side_effect = RuntimeError('offline')
            self.bridge._discover_planners()
            self.assertEqual(self.bridge.snapshot()['planners'], {})
            self.assertIn('offline', self.bridge.snapshot()['planner_error'])


if __name__ == '__main__':
    unittest.main()
