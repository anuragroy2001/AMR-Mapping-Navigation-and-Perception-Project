#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

def create_goal(x, y, z, ox, oy, oz, ow):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = z
    goal.target_pose.pose.orientation.x = ox
    goal.target_pose.pose.orientation.y = oy
    goal.target_pose.pose.orientation.z = oz
    goal.target_pose.pose.orientation.w = ow
    return goal

if __name__ == "__main__":
    rospy.init_node("multi_goal_nav_client")
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    ac.wait_for_server()

    goal_list = [
    create_goal(22, -21, 0.0, 0.0, 0.0, -1.0, 0.1),
    create_goal(10, -19, 0.0, 0.0, 0.0, -1.0, 0.0),
    create_goal(10, -15, 0.0, 0.0, 0.0, 0.707, 0.707),
    create_goal(18, -15, 0.0, 0.0, 0.0, 0.0, 1.0),
    create_goal(18, -11, 0.0, 0.0, 0.0, 0.7, 0.7),
    create_goal(10, -11, 0.0, 0.0, 0.0, -1.0, 0.0),
    create_goal(10, -7,  0.0, 0.0, 0.0, 0.7, 0.7),
    create_goal(18, -7,  0.0, 0.0, 0.0, 0.0, 1.0),
    create_goal(18, -3,  0.0, 0.0, 0.0, 0.7, 0.7),
    create_goal(10, -3,  0.0, 0.0, 0.0, -1.0, 0.0)
]


    for idx, goal in enumerate(goal_list):
        rospy.loginfo(f"Sending goal {idx+1}/{len(goal_list)}")
        ac.send_goal(goal)
        ac.wait_for_result()

        if ac.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo(f"Goal {idx+1} reached.")
        else:
            rospy.logwarn(f"Goal {idx+1} failed. State: {ac.get_state()}")

