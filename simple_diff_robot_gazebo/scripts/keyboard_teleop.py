#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import select
import sys
import termios
import tty

import rospy
from geometry_msgs.msg import Twist


HELP = """
Simple differential-drive keyboard control
------------------------------------------
        w
   a    s    d

w/s : forward/backward
a/d : turn left/right
space or x : stop
q : quit

Hold a movement key to keep moving. The robot stops automatically when
keyboard input times out; press space to stop immediately.
"""


def read_key(timeout=0.1):
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if ready else ""


def make_twist(linear=0.0, angular=0.0):
    msg = Twist()
    msg.linear.x = linear
    msg.angular.z = angular
    return msg


def main():
    rospy.init_node("keyboard_teleop")
    publisher = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    linear_speed = rospy.get_param("~linear_speed", 0.30)
    angular_speed = rospy.get_param("~angular_speed", 0.80)
    command_timeout = rospy.Duration(rospy.get_param("~command_timeout", 0.50))

    if not sys.stdin.isatty():
        rospy.logfatal("Keyboard teleop requires an interactive terminal (TTY).")
        return 1

    settings = termios.tcgetattr(sys.stdin)
    print(HELP)

    try:
        tty.setraw(sys.stdin.fileno())
        last_motion_key = rospy.Time(0)
        moving = False
        while not rospy.is_shutdown():
            key = read_key()
            if key == "w":
                publisher.publish(make_twist(linear_speed, 0.0))
                last_motion_key, moving = rospy.Time.now(), True
            elif key == "s":
                publisher.publish(make_twist(-linear_speed, 0.0))
                last_motion_key, moving = rospy.Time.now(), True
            elif key == "a":
                publisher.publish(make_twist(0.0, angular_speed))
                last_motion_key, moving = rospy.Time.now(), True
            elif key == "d":
                publisher.publish(make_twist(0.0, -angular_speed))
                last_motion_key, moving = rospy.Time.now(), True
            elif key in (" ", "x"):
                publisher.publish(make_twist())
                moving = False
            elif key in ("q", "\x03"):
                break
            elif moving and rospy.Time.now() - last_motion_key > command_timeout:
                publisher.publish(make_twist())
                moving = False
    finally:
        publisher.publish(make_twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("\nRobot stopped.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
