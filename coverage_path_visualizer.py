#!/usr/bin/env python3

import cv2
import numpy as np
import yaml
import matplotlib.pyplot as plt
import csv
import math

# === CONFIG ===
map_yaml_path = 'src/me5413_world/maps/map.yaml'
map_pgm_path = 'src/me5413_world/maps/map.pgm'
output_csv = 'coverage_path.csv'

# Region of interest (world coordinates)
region_x_min, region_x_max = 9, 19
region_y_min, region_y_max = -20, -2

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

def world_to_map(x, y, resolution, origin):
    map_x = int((x - origin[0]) / resolution)
    map_y = int((y - origin[1]) / resolution)
    return map_x, map_y

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

def generate_coverage_path(binary_map, resolution, origin, point_stride=10, grid_stride=10):
    h, w = binary_map.shape
    path_poses = []
    step_counter = 0

    for y in range(0, h, grid_stride):  # skip rows = sparse sweep lines
        row = range(0, w, point_stride) if (y // grid_stride) % 2 == 0 else reversed(range(0, w, point_stride))
        for x in row:
            if binary_map[y, x] == 0:
                wx, wy = map_to_world(x, y, resolution, origin)
                if region_x_min <= wx <= region_x_max and region_y_min <= wy <= region_y_max:
                    path_poses.append((wx, wy))

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

def save_path(path, output_file):
    with open(output_file, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['x', 'y', 'qx', 'qy', 'qz', 'qw'])
        for pose in path:
            writer.writerow(pose)

def visualize_path(binary_map, path, resolution, origin):
    map_img = 255 - (binary_map * 255).astype(np.uint8)
    map_img_color = cv2.cvtColor(map_img, cv2.COLOR_GRAY2BGR)
    for wx, wy, _, _, _, _ in path:
        mx, my = world_to_map(wx, wy, resolution, origin)
        if 0 <= my < binary_map.shape[0] and 0 <= mx < binary_map.shape[1]:
            cv2.circle(map_img_color, (mx, my), 1, (0, 0, 255), -1)
    plt.imshow(map_img_color)
    plt.title('Coverage Path Visualization')
    plt.show()

def main():
    print("Loading map...")
    binary_map, resolution, origin = load_map(map_yaml_path, map_pgm_path)
    print("Generating coverage path...")
    path = generate_coverage_path(binary_map, resolution, origin)
    print(f"Generated {len(path)} poses.")
    print("Saving to CSV...")
    save_path(path, output_csv)
    print(f"Saved to {output_csv}")
    visualize_path(binary_map, path, resolution, origin)

if __name__ == '__main__':
    main()

