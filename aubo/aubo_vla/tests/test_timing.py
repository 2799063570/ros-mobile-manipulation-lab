from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from aubo_vla.timing import ClockGuard, positive, source_age


class TimingTest(unittest.TestCase):
    def test_real_camera_does_not_need_clock(self):
        self.assertFalse(ClockGuard(False).paused(100.))

    def test_pause_resume_and_rollback_invalidate_epoch(self):
        guard = ClockGuard(True, 2.)
        self.assertTrue(guard.paused(0.))
        self.assertFalse(guard.update(100., 1.))
        self.assertFalse(guard.paused(2.))
        guard.update(100., 2.)
        self.assertTrue(guard.paused(3.1))
        self.assertTrue(guard.update(101., 4.))
        self.assertEqual(guard.epoch, 1)
        self.assertFalse(guard.paused(4.))
        self.assertTrue(guard.update(10., 4.1))
        self.assertEqual(guard.epoch, 2)

    def test_source_timestamp_and_parameter_validation(self):
        self.assertAlmostEqual(source_age(100., 100.2, 1.), 0.2)
        for stamp in (0., float('nan'), 99., 102.):
            with self.assertRaises(ValueError):
                source_age(stamp, 100.1, 1.)
        for value in (0., -1., float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                positive(value, 'timeout')
