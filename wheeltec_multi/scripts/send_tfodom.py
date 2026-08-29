#!/usr/bin/env python
# coding=utf-8
"""广播主车在 map 中的位姿以及 base_link 坐标系速度。"""

import socket
import struct

import rospy
import tf
from nav_msgs.msg import Odometry


leader_vx = 0.0
leader_vy = 0.0
leader_wz = 0.0
sequence = 0


def odom_callback(msg):
    global leader_vx, leader_vy, leader_wz
    leader_vx = msg.twist.twist.linear.x
    leader_vy = msg.twist.twist.linear.y
    leader_wz = msg.twist.twist.angular.z


def publish_odom():
    global sequence
    rospy.init_node('send_tfodom')
    rospy.Subscriber('odom', Odometry, odom_callback, queue_size=1)

    broadcast_rate = max(1.0, rospy.get_param('~broadcast_rate', 30.0))
    map_frame = rospy.get_param('~map_frame', 'map')
    base_frame = rospy.get_param('~base_frame', 'base_link')
    udp_port = rospy.get_param('~udp_port', 10000)
    # V2 packet: magic, monotonically increasing sequence, sender wall-clock
    # stamp, followed by pose (x, y, yaw) and body twist (vx, vy, wz).
    packet_format = '<4sId6f'

    listener = tf.TransformListener()
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    rate = rospy.Rate(broadcast_rate)

    try:
        while not rospy.is_shutdown():
            try:
                trans, rotation = listener.lookupTransform(
                    map_frame, base_frame, rospy.Time(0))
                yaw = tf.transformations.euler_from_quaternion(rotation)[2]
                sequence = (sequence + 1) & 0xffffffff
                packet = struct.pack(packet_format, b'WFM2', sequence,
                                     rospy.Time.now().to_sec(),
                                     trans[0], trans[1], yaw,
                                     leader_vx, leader_vy, leader_wz)
                udp_socket.sendto(packet, ('<broadcast>', udp_port))
                rospy.logdebug_throttle(
                    1.0, 'leader pose=(%.3f, %.3f, %.3f), velocity=(%.3f, %.3f, %.3f)',
                    trans[0], trans[1], yaw, leader_vx, leader_vy, leader_wz)
            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException) as exc:
                rospy.logwarn_throttle(1.0, 'leader TF unavailable: %s', str(exc))
            rate.sleep()
    finally:
        udp_socket.close()


if __name__ == '__main__':
    try:
        publish_odom()
    except rospy.ROSInterruptException:
        pass
