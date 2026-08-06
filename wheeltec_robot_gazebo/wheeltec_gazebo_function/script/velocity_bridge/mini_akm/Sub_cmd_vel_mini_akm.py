#!/usr/bin/env python

# Author: christoph.roesmann@tu-dortmund.de
import rospy,math
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist



def set_mini_akm_velocity_steering(data):
    pub_vel_mini_akm_left_rear_wheel = rospy.Publisher('/wheeltec/mini_akm_left_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_mini_akm_right_rear_wheel = rospy.Publisher('/wheeltec/mini_akm_right_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_pos_mini_akm_left_front_steering_hinge = rospy.Publisher('/wheeltec/mini_akm_left_steering_hinge_position_controller/command', Float64, queue_size=1)
    pub_pos_mini_akm_right_front_steering_hinge = rospy.Publisher('/wheeltec/mini_akm_right_steering_hinge_position_controller/command', Float64, queue_size=1)
    #cmd_vel linear.x
    v = data.linear.x
    #cmd_vel angular.z
    w = data.angular.z
    # akm velocity calculation
    if v > 0 :
        g=1
    elif v == 0:
        g=0
    else:
        g=-1
    if w >= 0 :
	    k = 1       #turn left
    else:
	    k = -1      #turn right
    if w == 0 :
        wl = 0
        wr = 0
        vlr = v/0.031   #w=v/r moving forward
        vrr = v/0.031
    else :
        R = v/w
        a2 =0.08
        wheelbase = 0.143
        l = 0.16
        m= R **2 -a2 **2
        ml= abs(m) **0.5 - k*(wheelbase/2)
        mr= abs(m) **0.5 + k*(wheelbase/2)
	if m < 0 or ml<0 or mr<0 :
            wl=0
            wr=0
            vlr = 0
            vrr = 0
	else :
            vlr= (g*(((v**2) -((w*a2)**2))**0.5-((g*wheelbase*w)/2)))/0.031 
            vrr= (g*(((v**2) -((w*a2)**2))**0.5+((g*wheelbase*w)/2)))/0.031 
            wl = g*k*math.atan(l/ml)
            wr = g*k*math.atan(l/mr)
    #publish velocity to controller command
    pub_pos_mini_akm_left_front_steering_hinge.publish(wl)
    pub_pos_mini_akm_right_front_steering_hinge.publish(wr)
    pub_vel_mini_akm_left_rear_wheel.publish(vlr)
    pub_vel_mini_akm_right_rear_wheel.publish(vrr)
def Sub_cmd_vel_mini_akm():

    rospy.init_node('Sub_cmd_vel_mini_akm', anonymous=True)
    #Subscriber cmd_vel
    rospy.Subscriber("/cmd_vel", Twist, set_mini_akm_velocity_steering, queue_size=1)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        Sub_cmd_vel_mini_akm()
    except rospy.ROSInterruptException:
        pass
