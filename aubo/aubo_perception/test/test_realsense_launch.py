"""Resolve real camera graphs without opening hardware or starting controllers."""
from pathlib import Path
import unittest

import roslaunch


def resolve(name, *args):
    config = roslaunch.config.ROSLaunchConfig()
    roslaunch.xmlloader.XmlLoader().load(
        str(Path(__file__).resolve().parents[1] / 'launch' / name),
        config, argv=list(args), verbose=False)
    return config


class RealSenseLaunchTest(unittest.TestCase):
    def test_streams_and_namespace(self):
        config = resolve('realsense_d435i.launch', 'camera_name:=probe',
                         'serial_no:=TEST_SERIAL', 'publish_tf:=false')
        params = {k: v.value for k, v in config.params.items()}
        prefix = '/probe/realsense2_camera/'
        for key in ('enable_color', 'enable_depth', 'align_depth', 'enable_sync'):
            self.assertTrue(params[prefix + key])
        for key in ('enable_gyro', 'enable_accel', 'publish_tf'):
            self.assertFalse(params[prefix + key])
        self.assertEqual(params[prefix + 'serial_no'], 'TEST_SERIAL')
        self.assertEqual(params[prefix + 'color_width'], 640)
        self.assertEqual(params[prefix + 'color_fps'], 30)
        self.assertEqual(params[prefix + 'color_optical_frame_id'], 'probe_color_optical_frame')

    def test_imu_and_reset_are_explicit(self):
        config = resolve('realsense_debug.launch', 'enable_imu:=true', 'initial_reset:=true')
        params = {k: v.value for k, v in config.params.items()}
        prefix = '/camera/realsense2_camera/'
        for key in ('enable_gyro', 'enable_accel', 'initial_reset'):
            self.assertTrue(params[prefix + key])
        self.assertEqual(params[prefix + 'gyro_fps'], 400)
        self.assertEqual(params[prefix + 'accel_fps'], 200)

    def test_debug_has_only_perception_and_isolated_outputs(self):
        for detector, label in (('color', 'red'), ('yolo', 'can')):
            config = resolve('realsense_debug.launch', 'start_camera:=false',
                             'camera_name:=probe', 'detector:=' + detector)
            self.assertEqual(len(config.nodes), 2)
            self.assertTrue(all(n.package == 'aubo_perception' for n in config.nodes))
            params = {k: v.value for k, v in config.params.items()}
            prefix = '/camera_debug/grasp_geometry/'
            self.assertEqual(params[prefix + 'task_mode'], 'servo')
            self.assertEqual(params[prefix + 'camera_mount'], 'eye_to_hand')
            self.assertEqual(params[prefix + 'selected_class'], label)
            self.assertEqual(params[prefix + 'aligned_depth_topic'],
                             '/probe/aligned_depth_to_color/image_raw')
            geometry = next(n for n in config.nodes if n.name == 'grasp_geometry')
            self.assertIn(['/visual_servo/target_pose', '/camera_debug/target_pose'],
                          geometry.remap_args)


if __name__ == '__main__':
    unittest.main()
