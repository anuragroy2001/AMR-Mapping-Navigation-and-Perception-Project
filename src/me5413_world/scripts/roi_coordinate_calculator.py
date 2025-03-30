#!/usr/bin/env python3
import rospy
import tf
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2D
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np


class ROICoordinateCalculator:
    def __init__(self):
        rospy.init_node('roi_coordinate_calculator')

        self.listener = tf.TransformListener()

        self.camera_info_sub = rospy.Subscriber('/front/rgb/camera_info', CameraInfo, self.camera_info_callback)
        self.depth_image_sub = rospy.Subscriber('/me5413/current_depth', Image, self.depth_callback)
        self.detection_sub = rospy.Subscriber('/me5413/detected', Detection2D, self.detection_callback)

        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.goal_name_pub = rospy.Publisher('/rviz_panel/goal_name', String, queue_size=10)
        self.calculated_target_pub = rospy.Publisher('/calculated/target', PoseStamped, queue_size=10)

        self.bridge = CvBridge()
        self.camera_info = None
        self.depth_image = None

    def camera_info_callback(self, data):
        self.camera_info = data

    def depth_callback(self, data):
        # Use 32FC1 encoding to get depth in meters
        self.depth_image = data

    def detection_callback(self, detection_msg):
        if self.camera_info is None or self.depth_image is None:
            rospy.loginfo("Waiting for camera info and depth image...")
            return

        if detection_msg.bbox.size_x == 0 or detection_msg.bbox.size_y == 0:
            return

        rospy.loginfo("Target found.")
        current_time = self.depth_image.header.stamp
        cv_depth_image = self.bridge.imgmsg_to_cv2(self.depth_image, desired_encoding="32FC1")
        bbox = detection_msg.bbox
        center = bbox.center
        size_x = bbox.size_x
        size_y = bbox.size_y

        start_x = int(max(center.x - size_x / 2, 0))
        start_y = int(max(center.y - size_y / 2, 0))
        end_x = int(min(center.x + size_x / 2, cv_depth_image.shape[1] - 1))
        end_y = int(min(center.y + size_y / 2, cv_depth_image.shape[0] - 1))

        roi_depth = cv_depth_image[start_y:end_y, start_x:end_x]

        average_depth = np.nanmean(roi_depth)

        # Get the depth value at the center of the bounding box
        depth = average_depth

        if np.isnan(depth) or np.isinf(depth):
            rospy.loginfo("Invalid depth value.")
            return

        fx = self.camera_info.K[0]
        fy = self.camera_info.K[4]
        cx = self.camera_info.K[2]
        cy = self.camera_info.K[5]

        X = (center.x - cx) * depth / fx
        Y = (center.y - cy) * depth / fy
        Z = depth - 1.0  # Subtract 1.0 to avoid collision with the target
        if Z < 0:
            Z = 0.0

        point_stamped = PointStamped()
        # current_time = rospy.Time.now()
        point_stamped.header.stamp = current_time
        point_stamped.header.frame_id = "front_frame_optical"
        point_stamped.point.x = X
        point_stamped.point.y = Y
        point_stamped.point.z = Z

        # Try to transform the point to the map frame
        try:
            self.listener.waitForTransform("map", point_stamped.header.frame_id, current_time, rospy.Duration(4.0))
            map_point = self.listener.transformPoint("map", point_stamped)

            goal_pose = PoseStamped()
            goal_pose.header.stamp = current_time
            goal_pose.header.frame_id = "map"
            goal_pose.pose.position.x = map_point.point.x
            goal_pose.pose.position.y = map_point.point.y
            goal_pose.pose.position.z = map_point.point.z
            goal_pose.pose.orientation.w = 1.0

            rospy.loginfo("Target position successfully calculated: x={}, y={}, z={}".format(
                goal_pose.pose.position.x,
                goal_pose.pose.position.y,
                goal_pose.pose.position.z
            ))

            # Publish the goal pose
            self.goal_pub.publish(goal_pose)
            self.calculated_target_pub.publish(goal_pose)
            # Publish the goal name to substitute the goal name from the last goal
            goal_name_msg = String()
            goal_name_msg.data = "/done_1"
            self.goal_name_pub.publish(goal_name_msg)

            return
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("TF error when converting point: %s", e)


if __name__ == '__main__':
    node = ROICoordinateCalculator()
    rospy.spin()
