#!/usr/bin/env python

# Author: christoph.roesmann@tu-dortmund.de
import rospy,math
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
def set_mini_4wd_velocity(data):
    pub_vel_mini_4wd_left_front_wheel = rospy.Publisher('/wheeltec/mini_4wd_left_front_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_mini_4wd_right_front_wheel = rospy.Publisher('/wheeltec/mini_4wd_right_front_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_mini_4wd_left_rear_wheel = rospy.Publisher('/wheeltec/mini_4wd_left_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_mini_4wd_right_rear_wheel = rospy.Publisher('/wheeltec/mini_4wd_right_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    a  = 0.095    #for mini_4wd
    b  = 0.086    #for mini_4wd
    Vlf = (data.linear.x-data.angular.z*(a+b))/0.031
    Vrf = (data.linear.x+data.angular.z*(a+b))/0.031
    Vrr = (data.linear.x+data.angular.z*(a+b))/0.031
    Vlr = (data.linear.x-data.angular.z*(a+b))/0.031
    pub_vel_mini_4wd_left_front_wheel.publish(Vlf)
    pub_vel_mini_4wd_right_front_wheel.publish(Vrf)
    pub_vel_mini_4wd_left_rear_wheel.publish(Vlr)
    pub_vel_mini_4wd_right_rear_wheel.publish(Vrr)

def Sub_cmd_vel_mini_4wd():

    rospy.init_node('Sub_cmd_vel_mini_4wd', anonymous=True)

    rospy.Subscriber("/cmd_vel", Twist, set_mini_4wd_velocity, queue_size=1,buff_size=52428800)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        Sub_cmd_vel_mini_4wd()
    except rospy.ROSInterruptException:
        pass
