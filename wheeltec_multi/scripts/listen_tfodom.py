#!/usr/bin/env python
# coding=utf-8
"""接收主车 UDP 状态并发布到 multfodom。"""

import socket
import struct

import rospy
from std_msgs.msg import Float32MultiArray


def frame_listener():
    rospy.init_node('listen_tfodom')
    publisher = rospy.Publisher('multfodom', Float32MultiArray, queue_size=1)

    udp_port = rospy.get_param('~udp_port', 10000)
    packet_format = '<6f'
    packet_size = struct.calcsize(packet_format)
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
    udp_socket.settimeout(0.2)
    udp_socket.bind(('', udp_port))

    try:
        while not rospy.is_shutdown():
            try:
                data, _ = udp_socket.recvfrom(1024)
            except socket.timeout:
                continue

            if len(data) != packet_size:
                rospy.logwarn_throttle(
                    1.0, 'invalid leader packet: expected %d bytes, received %d',
                    packet_size, len(data))
                continue

            values = struct.unpack(packet_format, data)
            msg = Float32MultiArray()
            msg.data = list(values)
            publisher.publish(msg)
            rospy.logdebug_throttle(
                1.0, 'leader packet pose=(%.3f, %.3f, %.3f), velocity=(%.3f, %.3f, %.3f)',
                values[0], values[1], values[2], values[3], values[4], values[5])
    finally:
        udp_socket.close()


if __name__ == '__main__':
    try:
        frame_listener()
    except rospy.ROSInterruptException:
        pass
