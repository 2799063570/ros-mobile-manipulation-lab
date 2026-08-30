#!/usr/bin/env python3
# coding=utf-8
import rospy 

import math
import PyKDL
from geometry_msgs.msg import PoseWithCovarianceStamped


class PoseSetter(rospy.SubscribeListener):
    def __init__(self, pose, stamp, publish_time):
        self.pose = pose
        self.stamp = stamp
        self.publish_time = publish_time

        self.frame_id = rospy.get_param('~frame_id', 'map')

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        p = PoseWithCovarianceStamped()
        # 这里必须是从车在地图中的真实初始位姿，不能使用期望编队偏置。
        position_x = self.pose[0]
        position_y = self.pose[1]

        p.header.frame_id = self.frame_id
        p.header.stamp = self.stamp if self.stamp != rospy.Time() else rospy.Time.now()
        p.pose.pose.position.x = position_x
        p.pose.pose.position.y = position_y
        (p.pose.pose.orientation.x,
         p.pose.pose.orientation.y,
         p.pose.pose.orientation.z,
         p.pose.pose.orientation.w) = PyKDL.Rotation.RPY(0, 0, self.pose[2]).GetQuaternion()
        p.pose.covariance[6*0+0] = 0.5 * 0.5
        p.pose.covariance[6*1+1] = 0.5 * 0.5
        p.pose.covariance[6*5+5] = math.pi/12.0 * math.pi/12.0
        # wait for the desired publish time
        while rospy.get_rostime() < self.publish_time:
            rospy.sleep(0.01)
        rospy.loginfo("Publishing initial pose in %s: [%.3f, %.3f, %.3f]",
                      self.frame_id, position_x, position_y, self.pose[2])
        peer_publish(p)


if __name__ == '__main__':
    pose = list(map(float, rospy.myargv()[1:4]))
    t_stamp = rospy.Time()
    t_publish = rospy.Time()
    if len(rospy.myargv()) > 4:
        t_stamp = rospy.Time.from_sec(float(rospy.myargv()[4]))
    if len(rospy.myargv()) > 5:
        t_publish = rospy.Time.from_sec(float(rospy.myargv()[5]))
    rospy.init_node('pose_setter', anonymous=True)
    rospy.loginfo("Going to publish pose {} with stamp {} at {}".format(pose, t_stamp.to_sec(), t_publish.to_sec()))
    pub = rospy.Publisher("initialpose", PoseWithCovarianceStamped, PoseSetter(pose, stamp=t_stamp, publish_time=t_publish), queue_size=1)
    rospy.spin()
