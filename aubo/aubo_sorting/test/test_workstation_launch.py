"""Resolve optional channels without starting a server, tunnel or robot."""
import itertools
import unittest
import roslaunch
import roslaunch.rlutil


def resolve(*args):
    path = roslaunch.rlutil.resolve_launch_arguments(
        ['aubo_sorting', 'sorting_workstation.launch'])[0]
    config = roslaunch.config.ROSLaunchConfig()
    # Explicit baseline: the operator may edit the launch's preferred defaults.
    baseline = ['mode:=gazebo', 'start_thingsboard:=false', 'start_web:=false',
                'start_cpolar:=false', 'start_rqt:=false', 'enable_commands:=false']
    roslaunch.xmlloader.XmlLoader().load(path, config, argv=baseline + list(args), verbose=False)
    return config, {node.name: node for node in config.nodes}


class WorkstationLaunchTests(unittest.TestCase):
    def test_local_gazebo_without_automatic_motion_or_remote_channels(self):
        config, nodes = resolve('rviz:=false', 'gui:=false')
        self.assertIn('gazebo', nodes)
        self.assertIn('color_sorting_task', nodes)
        self.assertTrue(config.params['/use_sim_time'].value)
        self.assertFalse(config.params['/color_sorting_task/auto_start'].value)
        self.assertFalse(config.params['/color_sorting_task/auto_move_to_observation'].value)
        self.assertFalse({'aubo_cloud_bridge', 'aubo_huawei_dashboard', 'aubo_huawei_tunnel',
                          'aubo_huawei_bridge', 'sorting_panel'} & set(nodes))

    def test_optional_channels_are_independent_and_cpolar_brings_up_just_one_web_server(self):
        for cloud, web, tunnel in itertools.product((False, True), repeat=3):
            with self.subTest(cloud=cloud, web=web, tunnel=tunnel):
                config, nodes = resolve('start_sorting:=false',
                    'start_thingsboard:=' + str(cloud).lower(),
                    'start_web:=' + str(web).lower(),
                    'start_cpolar:=' + str(tunnel).lower())
                self.assertEqual('aubo_cloud_bridge' in nodes, cloud)
                self.assertEqual('aubo_huawei_dashboard' in nodes, web or tunnel)
                self.assertEqual('aubo_huawei_tunnel' in nodes, tunnel)
                self.assertNotIn('aubo_huawei_bridge', nodes)
                self.assertNotIn('gazebo', nodes)
                self.assertNotIn('move_group', nodes)
                self.assertNotIn('color_sorting_task', nodes)
                self.assertEqual(len(config.nodes), int(cloud) + int(web or tunnel) + int(tunnel))
                for name in ('aubo_cloud_bridge', 'aubo_huawei_dashboard', 'aubo_huawei_tunnel'):
                    if name in nodes:
                        self.assertFalse(nodes[name].required)
                        if name != 'aubo_huawei_tunnel':
                            self.assertFalse(config.params['/' + name + '/enable_commands'].value)

    def test_control_paths_port_and_interface_overrides_reach_each_consumer(self):
        config, nodes = resolve('start_sorting:=false', 'start_thingsboard:=true',
            'start_cpolar:=true', 'start_rqt:=true', 'enable_commands:=true',
            'thingsboard_host:=example.invalid', 'thingsboard_token_file:=/tmp/private-device-token',
            'web_auth_file:=/tmp/private-web-auth', 'web_port:=8101', 'cpolar_region:=cn_top',
            'sorting_ns:=/custom_sorting', 'task_node:=/custom_task')
        params = {name: value.value for name, value in config.params.items()}
        self.assertEqual(params['/aubo_cloud_bridge/host'], 'example.invalid')
        self.assertEqual(params['/aubo_cloud_bridge/token_file'], '/tmp/private-device-token')
        self.assertEqual(params['/aubo_huawei_dashboard/auth_file'], '/tmp/private-web-auth')
        self.assertEqual(params['/aubo_huawei_dashboard/port'], 8101)
        for name in ('aubo_cloud_bridge', 'aubo_huawei_dashboard'):
            self.assertTrue(params['/' + name + '/enable_commands'])
            self.assertEqual(params['/' + name + '/sorting_ns'], '/custom_sorting')
            self.assertEqual(params['/' + name + '/task_node'], '/custom_task')
        self.assertIn('--port 8101 --region cn_top', nodes['aubo_huawei_tunnel'].args)
        self.assertIn('--task-node /custom_task --sorting-ns /custom_sorting', nodes['sorting_panel'].args)

    def test_existing_simulation_overrides_still_reach_yolo_and_task(self):
        config, nodes = resolve('detector:=yolo', 'camera_mount:=eye_to_hand',
            'model_path:=/tmp/test-model.pt', 'object_height:=0.06', 'rviz:=false', 'gui:=false')
        params = {name: value.value for name, value in config.params.items()}
        self.assertEqual(params['/grasp_geometry/camera_mount'], 'eye_to_hand')
        self.assertEqual(params['/color_sorting_task/object_height'], .06)
        self.assertEqual(params['/color_sorting_task/sort_classes'], ['can'])
        self.assertEqual(params['/ultralytics_yolo/model_path'], '/tmp/test-model.pt')
        self.assertIn('workspace_camera_mount_tf', nodes)
        self.assertIn('/worlds/yolo_sorting.world', nodes['gazebo'].args)

    def test_real_mode_keeps_calibrated_cell_and_does_not_start_gazebo_or_duplicate_rqt(self):
        for panel in (False, True):
            with self.subTest(panel=panel):
                config, nodes = resolve('mode:=real', 'start_rqt:=' + str(panel).lower(),
                    'start_thingsboard:=true', 'start_cpolar:=true', 'rviz:=false')
                params = {name: value.value for name, value in config.params.items()}
                self.assertNotIn('gazebo', nodes)
                self.assertNotIn('spawn_urdf', nodes)
                self.assertIs(params['/use_sim_time'], False)
                self.assertEqual(params['/color_sorting_task/sort_classes'], ['bottle', 'can', 'box'])
                self.assertEqual(params['/color_sorting_task/table_center'], [-.0442, -.6707, -.09715])
                self.assertEqual(params['/color_sorting_task/observation_named_target'], 'real_sort_ready')
                self.assertTrue(params['/color_sorting_task/continuous_sorting'])
                self.assertEqual(params['/grasp_geometry/camera_mount'], 'eye_to_hand')
                self.assertEqual(params['/grasp_geometry/camera_info_topic'], '/camera/color/camera_info')
                self.assertFalse(params['/color_sorting_task/auto_start'])
                self.assertFalse(params['/color_sorting_task/auto_move_to_observation'])
                self.assertIn('publish_sorting_containers', nodes)
                self.assertIn('real_sorting_table', nodes)
                self.assertIn('publish_camera_stand', nodes)
                self.assertIn('aubo_cloud_bridge', nodes)
                self.assertIn('aubo_huawei_tunnel', nodes)
                self.assertEqual(sum(n.name == 'sorting_panel' for n in config.nodes), int(panel))

    def test_real_hardware_and_motion_overrides_reach_the_original_entry(self):
        config, nodes = resolve('mode:=real', 'start_robot:=false', 'start_camera:=false',
            'start_gripper:=false', 'rviz:=false', 'velocity_scaling:=0.12', 'acceleration_scaling:=0.08',
            'model_path:=/tmp/real-model.pt', 'observation_named_target:=real_observe')
        params = {name: value.value for name, value in config.params.items()}
        self.assertFalse(any(n.type in ('aubo_hw_node', 'move_group', 'realsense2_camera_node',
                                       'inspire_gripper_node') for n in config.nodes))
        self.assertEqual(params['/color_sorting_task/velocity_scaling'], .12)
        self.assertEqual(params['/color_sorting_task/acceleration_scaling'], .08)
        self.assertEqual(params['/color_sorting_task/observation_named_target'], 'real_observe')
        self.assertEqual(params['/ultralytics_yolo/model_path'], '/tmp/real-model.pt')
        self.assertIn('color_sorting_task', nodes)

    def test_existing_mode_never_starts_or_replaces_hardware_or_tasks(self):
        for args in ([], ['start_sorting:=true'], ['mode:=real', 'start_sorting:=false']):
            with self.subTest(args=args):
                config, nodes = resolve('mode:=existing', 'start_cpolar:=true', *args)
                self.assertEqual(set(nodes), {'aubo_huawei_dashboard', 'aubo_huawei_tunnel'})
                self.assertFalse(any(name.startswith('/color_sorting_task/') for name in config.params))
                self.assertNotIn('/use_sim_time', config.params)


if __name__ == '__main__':
    unittest.main()
