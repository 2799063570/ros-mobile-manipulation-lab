#!/usr/bin/env python3

import unittest

import numpy as np
from std_msgs.msg import Header

from aubo_mobile_follower.image_message import bgr8_to_imgmsg


class Bgr8ImageMessageTest(unittest.TestCase):
    def test_builds_contiguous_bgr_message_and_preserves_header(self):
        source = np.arange(4 * 6 * 3, dtype=np.uint8).reshape(4, 6, 3)
        non_contiguous = source[:, ::2, :]
        header = Header(seq=7, frame_id="hand_camera")

        message = bgr8_to_imgmsg(non_contiguous, header=header)

        self.assertEqual(message.header, header)
        self.assertEqual((message.height, message.width), (4, 3))
        self.assertEqual(message.encoding, "bgr8")
        self.assertEqual(message.step, 9)
        self.assertEqual(message.data, np.ascontiguousarray(non_contiguous).tobytes())

    def test_rejects_non_bgr_layout(self):
        with self.assertRaises(ValueError):
            bgr8_to_imgmsg(np.zeros((4, 6), dtype=np.uint8))

    def test_rejects_non_uint8_data(self):
        with self.assertRaises(ValueError):
            bgr8_to_imgmsg(np.zeros((4, 6, 3), dtype=np.float32))


if __name__ == "__main__":
    unittest.main()
