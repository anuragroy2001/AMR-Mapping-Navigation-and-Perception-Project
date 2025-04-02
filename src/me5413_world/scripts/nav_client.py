#!/usr/bin/env python3

import rospy
import actionlib
import cv2
import numpy as np
import yaml
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

# === MAP CONFIG ===
map_yaml_path = '/home/ariel/ME5413_Final_Project/src/me5413_world/maps/map.yaml'
map_pgm_path  = '/home/ariel/ME5413_Final_Project/src/me5413_world/maps/map.pgm'

# === COVERAGE AREA (world coordinates) ===
region_x_min, region_x_max = 10, 20
region_y_min, region_y_max = -19, -3

# === SPARSITY CONTROL ===
point_stride = 10  # spacing between points on a line
grid_stride = 10   # spacing between lines

def load_map(yaml_file, pgm_file):
    with open(yaml_file, 'r') as f:
        map_metadata = yaml.safe_load(f)
    resolution = map_metadata['resolution']
    origin = map_metadata['origin']
    image = cv2.imread(pgm_file, cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError("Map image not found.")
    thresh = cv2.threshold(image, 250, 1, cv2.THRESH_BINARY_INV)[1]
    return thresh, resolution, origin

def map_to_world(mx, my, resolution, origin):
    wx = mx * resolution + origin[0]
    wy = my * resolution + origin[1]
    return wx, wy

def yaw_to_quaternion(yaw):
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return qx, qy, qz, qw

def generate_sparse_path(binary_map, resolution, origin, start_pose=None):
    h, w = binary_map.shape
    path_poses = []

    for y in range(0, h, grid_stride):
        row = range(0, w, point_stride) if (y // grid_stride) % 2 == 0 else reversed(range(0, w, point_stride))
        for x in row:
            if binary_map[y, x] == 0:
                wx, wy = map_to_world(x, y, resolution, origin)
                if region_x_min <= wx <= region_x_max and region_y_min <= wy <= region_y_max:
                    path_poses.append((wx, wy))

    # Insert the start pose at the beginning if provided
    if start_pose:
        path_poses.insert(0, (start_pose[0], start_pose[1]))

    # Assign yaw between consecutive points
    full_path = []
    for i in range(len(path_poses)):
        x, y = path_poses[i]
        if i < len(path_poses) - 1:
            x2, y2 = path_poses[i + 1]
            yaw = math.atan2(y2 - y, x2 - x)
        else:
            yaw = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        full_path.append((x, y, qx, qy, qz, qw))
    return full_path

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

def main():
    rospy.init_node("sparse_coverage_path_executor")
    rospy.loginfo("Loading map...")
    binary_map, resolution, origin = load_map(map_yaml_path, map_pgm_path)

    rospy.loginfo("Generating sparse coverage path...")
    start_pose = (2.0, -19.0, 0.0)  # start point for coverage
    raw_path = generate_sparse_path(binary_map, resolution, origin, start_pose=start_pose)

    # Fixed first goal before coverage starts
    initial_goal = (22.0, -21.0, *yaw_to_quaternion(0.0))
    final_path = [initial_goal]

    # Sort the coverage goals by nearest distance (greedy)
    remaining_goals = raw_path.copy()
    current_x, current_y = initial_goal[0], initial_goal[1]

    while remaining_goals:
        min_dist = float('inf')
        min_idx = -1
        for i, (x, y, *_) in enumerate(remaining_goals):
            dist = math.hypot(x - current_x, y - current_y)
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        next_goal = remaining_goals.pop(min_idx)
        final_path.append(next_goal)
        current_x, current_y = next_goal[0], next_goal[1]

    # Send the goals to move_base
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("Waiting for move_base server...")
    ac.wait_for_server()

    for i, (x, y, qx, qy, qz, qw) in enumerate(final_path):
        rospy.loginfo(f"\nSending goal {i+1}/{len(final_path)}:")
        print(f"  x={x:.2f}, y={y:.2f}, qz={qz:.3f}, qw={qw:.3f}")
        goal = create_goal(x, y, 0.0, qx, qy, qz, qw)
        ac.send_goal(goal)
        ac.wait_for_result()

        if ac.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo(f"✔ Goal {i+1} reached.")
        else:
            rospy.logwarn(f"✘ Goal {i+1} failed. State: {ac.get_state()}")

if __name__ == '__main__':
    main()

