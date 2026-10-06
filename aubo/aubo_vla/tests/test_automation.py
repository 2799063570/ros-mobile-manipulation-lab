from pathlib import Path
import signal
import subprocess
import sys
import unittest
from unittest.mock import Mock, call, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from run_grasp_demo import stop_process


class AutomationTest(unittest.TestCase):
    def test_graceful_shutdown_signals_only_owned_group(self):
        process = Mock(pid=12345)
        with patch('run_grasp_demo.os.killpg') as kill:
            stop_process(process)
        kill.assert_called_once_with(12345, signal.SIGINT)
        process.wait.assert_called_once_with(timeout=15)

    def test_shutdown_escalates_when_process_does_not_exit(self):
        process = Mock(pid=12345)
        process.wait.side_effect = [subprocess.TimeoutExpired('test', 15), 0]
        with patch('run_grasp_demo.os.killpg') as kill:
            stop_process(process)
        self.assertEqual(kill.call_args_list, [call(12345, signal.SIGINT),
                                              call(12345, signal.SIGTERM)])

    def test_already_exited_process_is_ignored(self):
        process = Mock(pid=12345)
        with patch('run_grasp_demo.os.killpg', side_effect=ProcessLookupError):
            stop_process(process)
        process.wait.assert_not_called()

    def test_cleanup_can_fit_outer_roslaunch_shutdown_deadline(self):
        process = Mock(pid=12345)
        process.wait.side_effect = [subprocess.TimeoutExpired('test', 2), 0]
        with patch('run_grasp_demo.os.killpg'):
            stop_process(process, interrupt_timeout=2, terminate_timeout=1, kill_timeout=1)
        self.assertEqual(process.wait.call_args_list, [call(timeout=2), call(timeout=1)])
