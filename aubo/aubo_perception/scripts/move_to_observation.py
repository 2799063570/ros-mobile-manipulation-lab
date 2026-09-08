#!/usr/bin/env python3
"""Move the simulated wrist camera to an SRDF observation target once."""
import sys

import actionlib
import moveit_commander
import rospy
from control_msgs.msg import FollowJointTrajectoryAction
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_to_observation')
    ready = rospy.Publisher('~ready', Bool, queue_size=1, latch=True)
    ready.publish(False)
    arm = None
    try:
        timeout = float(rospy.get_param('~startup_timeout', 120.0))
        controller = actionlib.SimpleActionClient(
            '/aubo_i5/aubo_i5_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction)
        if not controller.wait_for_server(rospy.Duration(timeout)):
            raise RuntimeError('Arm trajectory controller did not become ready')
        rospy.wait_for_message('/joint_states', JointState,
                               timeout=timeout)
        arm = moveit_commander.MoveGroupCommander('aubo_i5', wait_for_servers=timeout)
        scene = moveit_commander.PlanningSceneInterface(synchronous=True)
        table = PoseStamped()
        table.header.frame_id = 'base_link'
        table.pose.orientation.w = 1.0
        table.pose.position.x = 0.70
        table.pose.position.z = float(rospy.get_param('~table_z', 0.10)) - 0.10
        scene.add_box('perception_table', table, size=(0.70, 1.00, 0.20))
        arm.set_max_velocity_scaling_factor(0.2)
        arm.set_max_acceleration_scaling_factor(0.2)
        arm.set_planning_time(10.0)
        target = rospy.get_param('~named_target', 'observe')
        arm.set_start_state_to_current_state()
        arm.set_named_target(target)
        if not arm.go(wait=True):
            raise RuntimeError('Failed to reach observation target: ' + target)
        arm.stop()
        ready.publish(True)
        rospy.loginfo('Observation target %s reached; perception is ready', target)
        rospy.spin()
    except (rospy.ROSException, RuntimeError) as exc:
        rospy.logerr('Observation move failed: %s', exc)
        return 1
    finally:
        if arm is not None:
            arm.stop()
        moveit_commander.roscpp_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
