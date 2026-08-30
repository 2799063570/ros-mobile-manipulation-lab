# -*- coding: utf-8 -*-

import rospy
import tf
from numpy import array
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped
from numpy import floor
from numpy.linalg import norm
from numpy import inf
# ________________________________________________________________________________


class robot:
    goal = MoveBaseGoal()
    start = PoseStamped()
    end = PoseStamped()

    def __init__(self, name):
        self.assigned_point = []
        self.name = name
        self.global_frame = rospy.get_param('~global_frame', '/map')
        self.robot_frame = rospy.get_param('~robot_frame', 'base_link')
        self.plan_service = rospy.get_param('~plan_service', '/move_base_node/NavfnROS/make_plan')
        self.listener = tf.TransformListener()
        self.listener.waitForTransform(self.global_frame, self.name + '/' + self.robot_frame,
                                       rospy.Time(0), rospy.Duration(10.0))
        cond = 0
        while cond == 0:
            try:
                rospy.loginfo('Waiting for the robot transform')
                (trans, rot) = self.listener.lookupTransform(self.global_frame,
                                                             self.name + '/' + self.robot_frame,
                                                             rospy.Time(0))
                cond = 1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                cond == 0
        self.position = array([trans[0], trans[1]])
        self.assigned_point = self.position
        self.client = actionlib.SimpleActionClient(self.name + '/move_base', MoveBaseAction)
        self.client.wait_for_server()
        robot.goal.target_pose.header.frame_id = self.global_frame
        robot.goal.target_pose.header.stamp = rospy.Time.now()

        rospy.wait_for_service(self.name + self.plan_service)
        self.make_plan = rospy.ServiceProxy(self.name + self.plan_service, GetPlan)
        robot.start.header.frame_id = self.global_frame
        robot.end.header.frame_id = self.global_frame

    def getPosition(self):
        cond = 0
        while cond == 0:
            try:
                (trans, rot) = self.listener.lookupTransform(self.global_frame,
                                                             self.name + '/' + self.robot_frame,
                                                             rospy.Time(0))
                cond = 1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                cond == 0
        self.position = array([trans[0], trans[1]])
        return self.position

    def sendGoal(self, point):
        robot.goal.target_pose.pose.position.x = point[0]
        robot.goal.target_pose.pose.position.y = point[1]
        robot.goal.target_pose.pose.orientation.w = 1.0
        self.client.send_goal(robot.goal)
        self.assigned_point = array(point)

    def cancelGoal(self):
        self.client.cancel_goal()
        self.assigned_point = self.getPosition()

    def getState(self):
        return self.client.get_state()

    def makePlan(self, start, end):
        robot.start.pose.position.x = start[0]
        robot.start.pose.position.y = start[1]
        robot.end.pose.position.x = end[0]
        robot.end.pose.position.y = end[1]
        start = self.listener.transformPose(self.name + '/map', robot.start)
        end = self.listener.transformPose(self.name + '/map', robot.end)
        plan = self.make_plan(start=start, goal=end, tolerance=0.0)
        return plan.plan.poses


# ________________________________________________________________________________


def index_of_point(mapData, Xp):
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y
    width = mapData.info.width
    cell_x = int(floor((Xp[0] - Xstartx) / resolution))
    cell_y = int(floor((Xp[1] - Xstarty) / resolution))
    if cell_x < 0 or cell_x >= mapData.info.width or cell_y < 0 or cell_y >= mapData.info.height:
        return -1
    return cell_y * width + cell_x


def point_of_index(mapData, i):
    row = i // mapData.info.width
    column = i % mapData.info.width
    y = mapData.info.origin.position.y + row * mapData.info.resolution
    x = mapData.info.origin.position.x + column * mapData.info.resolution
    return array([x, y])


# ________________________________________________________________________________


def informationGain(mapData, point, r):
    infoGain = 0
    index = index_of_point(mapData, point)
    if index < 0:
        return 0.0
    r_region = int(r / mapData.info.resolution)
    init_index = index - r_region * (mapData.info.width + 1)
    for n in range(0, 2 * r_region + 1):
        start = n * mapData.info.width + init_index
        end = start + 2 * r_region
        limit = ((start // mapData.info.width) + 2) * mapData.info.width
        for i in range(start, end + 1):
            if (i >= 0 and i < limit and i < len(mapData.data)):
                if (mapData.data[i] == -1 and norm(array(point) - point_of_index(mapData, i)) <= r):
                    infoGain += 1
    return infoGain * (mapData.info.resolution**2)


def checkAround(mapData, point, r):
    valueAtound = 0
    index = index_of_point(mapData, point)
    if index < 0:
        return 10000
    r_region = int(r / mapData.info.resolution)
    init_index = index - r_region * (mapData.info.width + 1)
    for n in range(0, 2 * r_region + 1):
        start = n * mapData.info.width + init_index
        end = start + 2 * r_region
        # limit = ((start/mapData.info.width)+2)*mapData.info.width
        for i in range(start, end + 1):
            if (i >= 0 and i < len(mapData.data)):
                if (mapData.data[i] > 0):
                    valueAtound += mapData.data[i]
    return valueAtound


# ________________________________________________________________________________


def discount(mapData, assigned_pt, centroids, infoGain, r):
    index = index_of_point(mapData, assigned_pt)
    if index < 0:
        return infoGain
    r_region = int(r / mapData.info.resolution)  # 附近的栅各数目
    init_index = index - r_region * (mapData.info.width + 1)  # 第一行第一个点
    for n in range(0, 2 * r_region + 1):
        start = n * mapData.info.width + init_index     # 遍历 每行的第一个点
        end = start + 2 * r_region      # 遍历 每行的最后一个点
        limit = ((start // mapData.info.width) + 2) * mapData.info.width
        for i in range(start, end + 1):
            if (i >= 0 and i < limit and i < len(mapData.data)):
                for j in range(0, len(centroids)):
                    current_pt = centroids[j]
                    if (mapData.data[i] == -1 and norm(point_of_index(mapData, i) - current_pt) <= r
                            and norm(point_of_index(mapData, i) - assigned_pt) <= r):
                        # this should be modified, subtract the area of a cell, not 1
                        infoGain[j] -= 1
    return infoGain


# ________________________________________________________________________________


def pathCost(path):
    if (len(path) > 0):
        i = len(path) // 2
        p1 = array([path[i - 1].pose.position.x, path[i - 1].pose.position.y])
        p2 = array([path[i].pose.position.x, path[i].pose.position.y])
        return norm(p1 - p2) * (len(path) - 1)
    else:
        return inf


# ________________________________________________________________________________


def unvalid(mapData, pt):
    index = index_of_point(mapData, pt)
    if index < 0:
        return True
    r_region = 5
    init_index = index - r_region * (mapData.info.width + 1)
    for n in range(0, 2 * r_region + 1):
        start = n * mapData.info.width + init_index
        end = start + 2 * r_region
        limit = ((start // mapData.info.width) + 2) * mapData.info.width
        for i in range(start, end + 1):
            if (i >= 0 and i < limit and i < len(mapData.data)):
                if (mapData.data[i] == 1):
                    return True
    return False


# ________________________________________________________________________________


def Nearest(V, x):
    n = inf
    i = 0
    for i in range(0, V.shape[0]):
        n1 = norm(V[i, :] - x)
        if (n1 < n):
            n = n1
            result = i
    return result


# ________________________________________________________________________________


def Nearest2(V, x):
    n = inf
    result = 0
    for i in range(0, len(V)):
        n1 = norm(V[i] - x)

        if (n1 < n):
            n = n1
    return i


# ________________________________________________________________________________


def gridValue(mapData, Xp):
    Data = mapData.data
    # returns grid value at "Xp" location
    # map data:  100 occupied      -1 unknown       0 free
    index = index_of_point(mapData, Xp)
    if index < 0 or index >= len(Data):
        return 100
    return Data[index]
