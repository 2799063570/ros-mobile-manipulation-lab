#!/usr/bin/env python3
"""PTY模拟STM32，穿过真实串口节点验证点动/取消/重连；不连接硬件。"""
import binascii
import os
import pty
import select
import signal
import struct
import subprocess
import threading
import time
import unittest

import rospy
import rostest
from aubo_ros_control.msg import MoveHomeActionGoal
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String


def packet(kind, sequence, payload):
    body = struct.pack('<BBHB', 2, kind, sequence, len(payload)) + payload
    return b'\xa5\x5a' + body + struct.pack('<H', binascii.crc_hqx(body, 0xffff))


class CartesianSerialTest(unittest.TestCase):
    def test_serial_control_lifecycle(self):
        lock = threading.RLock()
        q = [0.] * 6
        command = [7, 3, 3, 0, 100, 0, 0, 0]  # 缓存的HOME，重连不能执行
        feedback = {}
        motion = ['']
        twists = []
        goals = []
        paused = [False]
        stop = threading.Event()
        pub = rospy.Publisher('/joint_states', JointState, queue_size=1)

        def output(m, i):
            with lock:
                q[i] = m.data

        subs = [rospy.Subscriber('/remote_test/q%d' % i, Float64, output, callback_args=i)
                for i in range(6)]
        subs += [rospy.Subscriber('/cartesian_jog/state', String,
                                 lambda m: motion.__setitem__(0, m.data)),
                 rospy.Subscriber('/cartesian_jog/command', TwistStamped,
                                  lambda m: twists.append(m)),
                 rospy.Subscriber('/cartesian_jog/move_home/goal', MoveHomeActionGoal,
                                  lambda m: goals.append(m))]

        def tick(_):
            with lock:
                positions = list(q)
            m = JointState()
            m.header.stamp = rospy.Time.now()
            m.name = ['shoulder_joint', 'upperArm_joint', 'foreArm_joint',
                      'wrist1_joint', 'wrist2_joint', 'wrist3_joint']
            m.position = positions
            pub.publish(m)

        timer = rospy.Timer(rospy.Duration(.02), tick)
        master, slave = pty.openpty()
        port = os.ttyname(slave)
        log = open('/tmp/cartesian_serial_test_node.log', 'w')
        proc = subprocess.Popen(['rosrun', 'aubo_remote_bridge', 'cartesian_serial_bridge',
                                 '_port:=' + port, '_joint_states_topic:=/joint_states'],
                                stdout=log, stderr=subprocess.STDOUT)

        def mcu():
            buffer = bytearray()
            while not stop.is_set():
                ready, _, _ = select.select([master], [], [], .02)
                if not ready:
                    continue
                try:
                    buffer.extend(os.read(master, 4096))
                except OSError:
                    continue
                while len(buffer) >= 9:
                    if buffer[:2] != b'\xa5\x5a':
                        del buffer[0]
                        continue
                    size = buffer[6] + 9
                    if len(buffer) < size:
                        break
                    frame = bytes(buffer[:size])
                    del buffer[:size]
                    if binascii.crc_hqx(frame[2:-2], 0xffff) != struct.unpack('<H', frame[-2:])[0]:
                        continue
                    sequence = struct.unpack('<H', frame[4:6])[0]
                    if frame[3] == 0x11 and not paused[0]:
                        session = struct.unpack('<I', frame[7:11])[0]
                        with lock:
                            rid, cmd, flags, basis, speed, x, y, z = command
                        payload = struct.pack('<IHBBBBHhhh', session, rid, cmd, flags,
                                              basis, 0, speed, x, y, z)
                        os.write(master, packet(0x12, sequence, payload))
                    elif frame[3] == 0x13:
                        values = struct.unpack('<IHBBBBBB6hHH', frame[7:-2])
                        with lock:
                            feedback.update(session=values[0], rid=values[1], cmd=values[2],
                                            ack=values[3], state=values[4], home=values[5],
                                            flags=values[6], mask=values[7], joints=values[8:14])

        thread = threading.Thread(target=mcu, daemon=True)
        thread.start()

        def wait(predicate, timeout=5):
            end = time.monotonic() + timeout
            while time.monotonic() < end and not rospy.is_shutdown():
                if predicate():
                    return True
                time.sleep(.01)
            self.fail('等待超时: feedback=%r state=%r log=/tmp/cartesian_serial_test_node.log' %
                      (feedback, motion[0]))

        def request(rid, cmd):
            with lock:
                command[:] = [rid, cmd, 3, 0, 100, 0, 0, 0]

        def ack(rid, status, timeout=5):
            wait(lambda: feedback.get('rid') == rid and feedback.get('ack') == status, timeout)

        try:
            wait(lambda: feedback.get('rid') == 7)
            self.assertEqual(feedback['ack'], 8)
            self.assertEqual(len(goals), 0)
            self.assertLess(max(abs(v) for v in q), 1e-6)
            wait(lambda: feedback.get('state') == 2 and not (feedback.get('flags', 8) & 8))
            request(8, 1)
            ack(8, 4)
            self.assertTrue(feedback['flags'] & 2)
            self.assertEqual(feedback['state'], 3)  # DONE必须与ENABLED_IDLE同帧出现。
            self.assertFalse(feedback['flags'] & 8)
            with lock:
                command[2] = 7
                command[4] = 1000  # 100 mm/s
                command[5:8] = [1000, 1000, 0]
            wait(lambda: q[0] > .005 and q[1] > .005)
            m = next(m for m in reversed(twists) if m.twist.linear.x > .001)
            self.assertAlmostEqual((m.twist.linear.x**2 + m.twist.linear.y**2)**.5, .1, places=5)
            with lock:
                command[2] = 3  # 松开deadman后即使轴未回中也必须发送零速度
            wait(lambda: motion[0] == 'ENABLED_IDLE')
            held = list(q)
            time.sleep(.15)
            self.assertLess(max(abs(a-b) for a, b in zip(q, held)), 1e-5)

            # 断链停用，恢复时旧使能请求和仍偏转的摇杆都不能恢复运动。
            paused[0] = True
            wait(lambda: motion[0] == 'DISABLED')
            time.sleep(.12)
            old_session = feedback['session']
            with lock:
                command[2] = 7
            paused[0] = False
            wait(lambda: feedback.get('session') != old_session)
            self.assertFalse(feedback['flags'] & 2)
            time.sleep(.2)
            self.assertEqual(motion[0], 'DISABLED')
            with lock:
                command[:] = [0, 0, 3, 0, 100, 0, 0, 0]
            time.sleep(.15)
            request(9, 1)
            ack(9, 4)
            request(10, 2)
            ack(10, 4)
            wait(lambda: motion[0] == 'DISABLED')

            request(11, 3)
            wait(lambda: len(goals) == 1 and motion[0] == 'HOME_MOVING')
            time.sleep(.15)
            request(12, 4)
            ack(12, 7)
            wait(lambda: motion[0] == 'DISABLED')
            held = list(q)
            time.sleep(.2)
            self.assertLess(max(abs(a-b) for a, b in zip(q, held)), 1e-5)
            request(13, 3)
            ack(13, 4, 15)
            self.assertAlmostEqual(q[0], .12, delta=.005)
            self.assertLess(abs(q[1]), .005)
            self.assertEqual(len(goals), 2)
            time.sleep(.2)
            self.assertEqual(len(goals), 2)  # latched请求重发不生成第二个goal
            self.assertEqual(feedback['state'], 2)
            self.assertEqual(feedback['mask'], 63)

            # 同编号换命令被拒绝，不得意外使能。
            request(13, 1)
            ack(13, 8)
            self.assertFalse(feedback['flags'] & 2)

            # home过程中断链也必须取消；恢复后缓存HOME不会再次执行。
            wait(lambda: feedback.get('state') == 2 and not (feedback.get('flags', 8) & 8))
            request(14, 1)
            ack(14, 4)
            with lock:
                command[2], command[4], command[5] = 7, 1000, -1000
            wait(lambda: q[0] < .10)
            request(15, 2)
            ack(15, 4)
            request(16, 3)
            wait(lambda: len(goals) == 3 and motion[0] == 'HOME_MOVING')
            paused[0] = True
            wait(lambda: motion[0] == 'DISABLED')
            time.sleep(.15)
            old_session = feedback['session']
            paused[0] = False
            wait(lambda: feedback.get('session') != old_session)
            wait(lambda: feedback.get('home') in (2, 8))
            self.assertEqual(len(goals), 3)

            # ADC/校准失效即停止；输入恢复本身不能重新使能。
            with lock:
                command[:] = [0, 0, 3, 0, 100, 0, 0, 0]
            wait(lambda: feedback.get('state') == 2 and not (feedback.get('flags', 8) & 8))
            request(17, 1)
            ack(17, 4)
            with lock:
                command[2] = 0
                command[5] = 500
            wait(lambda: motion[0] == 'DISABLED' and not (feedback.get('flags', 3) & 3))
            with lock:
                command[2] = 7
            time.sleep(.2)
            self.assertFalse(feedback['flags'] & 2)
            self.assertEqual(motion[0], 'DISABLED')

            # MCU快速复位仍持续回复POLL，不能靠100ms串口超时才能恢复编号。
            old_session = feedback['session']
            with lock:
                command[:] = [0, 0, 0, 0, 100, 0, 0, 0]
            wait(lambda: feedback.get('session') != old_session)
            time.sleep(.16)  # 校准期间无效INPUT持续正常回复。
            reset_session = feedback['session']
            time.sleep(.08)
            self.assertEqual(feedback['session'], reset_session)
            with lock:
                command[2] = 3
            wait(lambda: feedback.get('state') == 2 and not (feedback.get('flags', 8) & 8))
            self.assertFalse(feedback['flags'] & 2)
            request(1, 1)
            ack(1, 4)
            self.assertEqual(feedback['state'], 3)
            self.assertTrue(feedback['flags'] & 2)
            request(2, 2)
            ack(2, 4)
            wait(lambda: motion[0] == 'DISABLED')
        finally:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
            stop.set()
            thread.join(timeout=1)
            timer.shutdown()
            for sub in subs:
                sub.unregister()
            os.close(master)
            os.close(slave)
            log.close()


if __name__ == '__main__':
    rospy.init_node('cartesian_serial_test')
    rostest.rosrun('aubo_remote_bridge', 'cartesian_serial', CartesianSerialTest)
