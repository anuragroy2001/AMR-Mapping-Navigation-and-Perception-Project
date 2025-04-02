#include "CleaningPathPlanner.h"
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <actionlib_msgs/GoalID.h>

bool getRobotPose(tf2_ros::Buffer& tf_buffer, geometry_msgs::PoseStamped& pose) {
    try {
        geometry_msgs::TransformStamped transform = tf_buffer.lookupTransform("map", "base_link", ros::Time(0));
        pose.header = transform.header;
        pose.pose.position.x = transform.transform.translation.x;
        pose.pose.position.y = transform.transform.translation.y;
        pose.pose.orientation = transform.transform.rotation;
        return true;
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return false;
    }
}

bool isInRegion(double x, double y) {
    // Region defined by corners: x ∈ [10, 20], y ∈ [-19, -3]
    return (x >= 10.0 && x <= 20.0 && y >= -19.0 && y <= -3.0);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_planning_node");
    ros::NodeHandle nh;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    costmap_2d::Costmap2DROS lcr("cleaning_costmap", tf_buffer);
    ros::Duration(5).sleep();  // Let TF and map settle

    CleaningPathPlanning clr(&lcr);

    // For canceling move_base goals
    ros::Publisher cancel_pub = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);

    ros::Rate rate(1.0);
    bool triggered = false;

    while (ros::ok()) {
        geometry_msgs::PoseStamped current_pose;
        if (getRobotPose(tf_buffer, current_pose)) {
            double x = current_pose.pose.position.x;
            double y = current_pose.pose.position.y;

            if (!triggered && isInRegion(x, y)) {
                ROS_INFO("Robot entered the target region, cancelling move_base goals...");
                actionlib_msgs::GoalID cancel_msg;
                cancel_pub.publish(cancel_msg);

                ROS_INFO("Starting coverage path planning...");
                clr.GetPathInROS();
                triggered = true;
            }

            if (triggered) {
                clr.PublishCoveragePath();
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    ros::shutdown();
    return 0;
}

