#!/usr/bin/env python3


import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal



if __name__ == "__main__":
    rospy.init_node("nav_client")
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)  # send request to movebase

    ac.wait_for_server() #wait for navigation server

    goal= MoveBaseGoal()  #target goal point

    goal.target_pose.header.frame_id = "map"

    goal.target_pose.pose.position.x = 7.0

    goal.target_pose.pose.position.y = -0.05

    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.0
    goal.target_pose.pose.orientation.w = 1.0

    ac.send_goal(goal)  #send msg to movebase
    rospy.loginfo("start navigating...")
    ac.wait_for_result()


    if ac.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("sucessfully navigated!")
    else:
        rospy.loginfo("fail to navigate...")







