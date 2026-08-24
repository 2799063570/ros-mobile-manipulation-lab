#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import select
import sys
import termios
import time
import tty

import rospy
from geometry_msgs.msg import Twist


HELP = """
AUBO mobile base keyboard control
---------------------------------
       w
  a    s    d

w/s       : forward/backward
a/d       : rotate left/right
q/e       : forward-left/forward-right
z/c       : backward-left/backward-right
space, x  : stop
+/-       : increase/decrease speed
h         : print this help
Ctrl-C    : quit

Hold or repeatedly press a movement key. The watchdog publishes zero velocity
when keyboard input stops.
"""


def make_twist(linear=0.0, angular=0.0):
    message = Twist()
    message.linear.x = linear
    message.angular.z = angular
    return message


def read_key(timeout):
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if ready else ""


def main():
    rospy.init_node("aubo_mobile_keyboard_teleop")

    command_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
    linear_speed = float(rospy.get_param("~linear_speed", 0.20))
    angular_speed = float(rospy.get_param("~angular_speed", 0.60))
    speed_step = float(rospy.get_param("~speed_step", 0.05))
    max_linear_speed = float(rospy.get_param("~max_linear_speed", 0.40))
    max_angular_speed = float(rospy.get_param("~max_angular_speed", 1.00))
    command_timeout = float(rospy.get_param("~command_timeout", 0.45))
    publish_rate = float(rospy.get_param("~publish_rate", 20.0))

    if not sys.stdin.isatty():
        rospy.logfatal("Keyboard teleop requires an interactive terminal (TTY).")
        return 1
    if publish_rate <= 0.0 or command_timeout <= 0.0:
        rospy.logfatal("publish_rate and command_timeout must be positive")
        return 1

    publisher = rospy.Publisher(command_topic, Twist, queue_size=1)
    terminal_settings = termios.tcgetattr(sys.stdin)
    current_command = make_twist()
    last_key_wall_time = time.time()

    movement = {
        "w": (1.0, 0.0),
        "s": (-1.0, 0.0),
        "a": (0.0, 1.0),
        "d": (0.0, -1.0),
        "q": (1.0, 1.0),
        "e": (1.0, -1.0),
        "z": (-1.0, -1.0),
        "c": (-1.0, 1.0),
    }

    print(HELP)
    try:
        tty.setraw(sys.stdin.fileno())
        while not rospy.is_shutdown():
            key = read_key(1.0 / publish_rate)
            now = time.time()

            if key in movement:
                linear_scale, angular_scale = movement[key]
                current_command = make_twist(
                    linear_scale * linear_speed, angular_scale * angular_speed
                )
                last_key_wall_time = now
            elif key in (" ", "x"):
                current_command = make_twist()
                last_key_wall_time = now
            elif key in ("+", "="):
                linear_speed = min(max_linear_speed, linear_speed + speed_step)
                angular_speed = min(max_angular_speed, angular_speed + 2.0 * speed_step)
                print("\rlinear={:.2f} angular={:.2f}      ".format(linear_speed, angular_speed))
            elif key in ("-", "_"):
                linear_speed = max(speed_step, linear_speed - speed_step)
                angular_speed = max(2.0 * speed_step, angular_speed - 2.0 * speed_step)
                print("\rlinear={:.2f} angular={:.2f}      ".format(linear_speed, angular_speed))
            elif key == "h":
                print(HELP)
            elif key in ("\x03",):
                break

            if now - last_key_wall_time > command_timeout:
                current_command = make_twist()

            publisher.publish(current_command)
    finally:
        publisher.publish(make_twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal_settings)
        print("\nRobot stopped.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
