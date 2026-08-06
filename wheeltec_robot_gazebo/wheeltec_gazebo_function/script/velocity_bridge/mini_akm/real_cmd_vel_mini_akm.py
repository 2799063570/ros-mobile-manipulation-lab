#!/usr/bin/env python

# Author: christoph.roesmann@tu-dortmund.de
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import LinkStates
def calculate_mini_akm_link_velocity_steering(data):
    pub_vel_mini_akm_base_footprint_velocity=rospy.Publisher('/wheeltec/mini_akm_base_footprint_velocity/link_cmd_vel', Float64,queue_size=1)
    pub_vel_mini_akm_base_footprint_steering=rospy.Publisher('/wheeltec/mini_akm_base_footprint_steering/link_cmd_vel', Float64,queue_size=1)
    v=data.twist
    if(len(v) == 18):     
        wbase=v[11].angular.z
        vbase=v[11].linear.x
        pub_vel_mini_akm_base_footprint_velocity.publish(vbase)
        pub_vel_mini_akm_base_footprint_steering.publish(wbase)

def calculate_mini_akm_joint_velocity_steering(data):
    pub_vel_mini_akm_left_rear_wheel = rospy.Publisher('/wheeltec/mini_akm_left_rear_wheel_velocity/cmd_vel', Float64, queue_size=1)
    pub_vel_mini_akm_right_rear_wheel = rospy.Publisher('/wheeltec/mini_akm_right_rear_wheel_velocity/cmd_vel', Float64, queue_size=1)
    pub_pos_mini_akm_left_steering_hinge = rospy.Publisher('/wheeltec/mini_akm_left_steering_hinge/cmd_vel', Float64, queue_size=1)
    pub_pos_mini_akm_right_steering_hinge = rospy.Publisher('/wheeltec/mini_akm_right_steering_hinge/cmd_vel', Float64, queue_size=1)
    a=data.velocity
    b = data.position

    left_rear_wheel_velocity=a[1]*0.0325
    right_rear_wheel_velocity=a[4]*0.0325
    left_steering_turn=(b[2]/3.14)*180
    right_steering_turn=(b[5]/3.14)*180

    pub_vel_mini_akm_left_rear_wheel.publish(left_rear_wheel_velocity)
    pub_vel_mini_akm_right_rear_wheel.publish(right_rear_wheel_velocity)
    pub_pos_mini_akm_left_steering_hinge.publish(left_steering_turn)
    pub_pos_mini_akm_right_steering_hinge.publish(right_steering_turn)

def Pub_mini_akm_real_cmd_vel():

    rospy.init_node('real_cmd_vel_mini_akm', anonymous=True)

    rospy.Subscriber("/wheeltec/joint_states", JointState, calculate_mini_akm_joint_velocity_steering)
    rospy.Subscriber("/gazebo/link_states", LinkStates, calculate_mini_akm_link_velocity_steering)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        Pub_mini_akm_real_cmd_vel()
    except rospy.ROSInterruptException:
        pass
