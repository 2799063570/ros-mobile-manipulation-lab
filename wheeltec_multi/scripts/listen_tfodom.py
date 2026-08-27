#!/usr/bin/env python
# coding=utf-8
"""接收主车 UDP 状态，发布新版 leader_state 和兼容的 multfodom。"""

import socket
import struct

import rospy
from std_msgs.msg import Float32MultiArray
from wheeltec_multi.msg import LeaderState


def frame_listener():
    rospy.init_node('listen_tfodom')
    publisher = rospy.Publisher('multfodom', Float32MultiArray, queue_size=1)
    state_publisher = rospy.Publisher('leader_state', LeaderState, queue_size=1)

    udp_port = rospy.get_param('~udp_port', 10000)
    packet_format_v2 = '<4sId6f'
    packet_size_v2 = struct.calcsize(packet_format_v2)
    packet_format_legacy = '<6f'
    packet_size_legacy = struct.calcsize(packet_format_legacy)
    accept_legacy = rospy.get_param('~accept_legacy', True)
    max_clock_skew = max(0.0, rospy.get_param('~max_clock_skew', 0.5))
    map_frame = rospy.get_param('~map_frame', 'map')
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

            received_at = rospy.Time.now()
            source_stamp_valid = False
            sequence = 0
            if len(data) == packet_size_v2:
                magic, sequence, sent_at = struct.unpack(
                    '<4sId', data[:struct.calcsize('<4sId')])
                if magic != b'WFM2':
                    rospy.logwarn_throttle(1.0, 'invalid leader packet magic')
                    continue
                values = struct.unpack('<6f', data[struct.calcsize('<4sId'):])
                source_stamp = rospy.Time.from_sec(sent_at)
                source_stamp_valid = abs((received_at - source_stamp).to_sec()) <= max_clock_skew
                if not source_stamp_valid:
                    rospy.logwarn_throttle(
                        5.0, 'leader clock differs by more than %.2f s; using receive time',
                        max_clock_skew)
            elif accept_legacy and len(data) == packet_size_legacy:
                values = struct.unpack(packet_format_legacy, data)
                source_stamp = received_at
                rospy.logwarn_throttle(5.0, 'receiving legacy leader packets without timestamps')
            else:
                rospy.logwarn_throttle(
                    1.0, 'invalid leader packet size: received %d bytes', len(data))
                continue

            msg = Float32MultiArray()
            msg.data = list(values)
            publisher.publish(msg)

            state = LeaderState()
            state.header.stamp = source_stamp
            state.header.frame_id = map_frame
            state.sequence = sequence
            state.received_at = received_at
            state.source_stamp_valid = source_stamp_valid
            state.pose.x, state.pose.y, state.pose.theta = values[:3]
            state.twist.linear.x, state.twist.linear.y = values[3:5]
            state.twist.angular.z = values[5]
            state_publisher.publish(state)
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
