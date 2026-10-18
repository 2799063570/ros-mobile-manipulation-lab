"""Resolve complete launch graphs when ROS1 is installed; never start hardware."""
from pathlib import Path
import unittest

try:
    import roslaunch
except ImportError:
    roslaunch = None


ROOT = Path(__file__).resolve().parents[1]


def resolve(name, *args):
    config = roslaunch.config.ROSLaunchConfig()
    roslaunch.xmlloader.XmlLoader().load(
        str(ROOT/'launch'/name), config, argv=list(args), verbose=False)
    return config


@unittest.skipIf(roslaunch is None, 'ROS1 roslaunch is not installed')
class LaunchGraphTest(unittest.TestCase):
    def test_aruco_launch_only_starts_read_only_collector(self):
        config = resolve('validate_handeye.launch', 'output_dir:=/tmp/handeye_launch_check')
        self.assertEqual([(n.package, n.type) for n in config.nodes],
                         [('aubo_handeye_validation', 'validate_handeye.py')])

    def test_yolo_verify_has_slow_motion_and_no_sorting_or_gripper_actions(self):
        config = resolve('handeye_yolo_verify_real.launch', 'rviz:=false', 'table_z:=0.20')
        nodes = {(node.package, node.type) for node in config.nodes}
        params = {key: value.value for key, value in config.params.items()}
        self.assertIn(('aubo_handeye_validation', 'handeye_yolo_verify.py'), nodes)
        self.assertIn(('aubo_ros_control', 'aubo_hw_node'), nodes)
        self.assertFalse(any(n.package in ('aubo_sorting', 'aubo_sorting_core', 'gazebo_ros')
                             for n in config.nodes))
        self.assertNotIn(('inspire_gripper', 'inspire_gripper'), nodes)
        self.assertEqual(params['/handeye_yolo_verify/velocity_scaling'], .03)
        self.assertEqual(params['/handeye_yolo_verify/acceleration_scaling'], .02)
        self.assertEqual(params['/handeye_yolo_verify/hover_clearance'], .10)
        self.assertEqual(params['/grasp_geometry/height_mode'], 'depth')
        self.assertEqual(params['/grasp_geometry/camera_mount'], 'eye_to_hand')
        self.assertEqual(params['/eye_to_hand_calibration_tf/child_frame'], 'camera_color_optical_frame')
        self.assertEqual(params['/grasp_geometry/camera_info_topic'], '/camera/color/camera_info')
        self.assertAlmostEqual(params['/handeye_verify_table/table_center'][2], .10)
        self.assertEqual(params['/handeye_yolo_verify/table_z'], .20)
        self.assertNotIn('/color_sorting_task/auto_start', params)

    def test_external_camera_and_detector_can_be_reused(self):
        config = resolve('handeye_yolo_verify_real.launch', 'rviz:=false',
                         'start_camera:=false', 'start_perception:=false')
        self.assertFalse(any(n.package in ('realsense2_camera', 'aubo_perception')
                             for n in config.nodes))
        self.assertTrue(any(n.type == 'handeye_yolo_verify.py' for n in config.nodes))


if __name__ == '__main__':
    unittest.main()
