#!/usr/bin/env python

# Author: christoph.roesmann@tu-dortmund.de
import rospy,math
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
def set_mini_mec_velocity(data):
    pub_vel_mini_mec_left_front_wheel = rospy.Publisher('/wheeltec/mini_mec_left_front_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_mini_mec_right_front_wheel = rospy.Publisher('/wheeltec/mini_mec_right_front_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_mini_mec_left_rear_wheel = rospy.Publisher('/wheeltec/mini_mec_left_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_mini_mec_right_rear_wheel = rospy.Publisher('/wheeltec/mini_mec_right_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    a  = 0.095    #for mini_mec
    b  = 0.0875    #for mini_mec
    Vlf = (data.linear.x-data.linear.y-data.angular.z*(a+b))/0.0375
    Vrf = (data.linear.x+data.linear.y+data.angular.z*(a+b))/0.0375
    Vrr = (data.linear.x-data.linear.y+data.angular.z*(a+b))/0.0375
    Vlr = (data.linear.x+data.linear.y-data.angular.z*(a+b))/0.0375
    pub_vel_mini_mec_left_front_wheel.publish(Vlf)
    pub_vel_mini_mec_right_front_wheel.publish(Vrf)
    pub_vel_mini_mec_left_rear_wheel.publish(Vlr)
    pub_vel_mini_mec_right_rear_wheel.publish(Vrr)

def Sub_cmd_vel_mini_mec():

    rospy.init_node('Sub_cmd_vel_mini_mec', anonymous=True)

    rospy.Subscriber("/cmd_vel", Twist, set_mini_mec_velocity, queue_size=1,buff_size=52428800)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        Sub_cmd_vel_mini_mec()
    except rospy.ROSInterruptException:
        pass
