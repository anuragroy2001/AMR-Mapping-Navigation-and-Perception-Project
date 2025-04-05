import numpy as np
import imageio
import yaml

# 读取 YAML 文件
with open("map.yaml", 'r') as f:
    map_metadata = yaml.safe_load(f)

# 提取元数据
resolution = map_metadata['resolution']          # 每像素代表多少米
origin = map_metadata['origin']                  # 地图原点 (x, y, yaw)
image_path = map_metadata['image']               # 图像文件路径
occupied_thresh = map_metadata['occupied_thresh']
free_thresh = map_metadata['free_thresh']
negate = map_metadata['negate']

# 读取 PGM 图像（灰度图）
map_img = imageio.imread(image_path)
map_array = np.array(map_img)

# 地图尺寸（像素）
height, width = map_array.shape

# 相对于原点的 x 坐标（单位：米）
x_positions_relative = [6, 10, 14, 18]
square_size_m = 0.8  # 正方形边长（米）

# 计算实际世界坐标（加上原点偏移）
x_positions_m = [x_rel for x_rel in x_positions_relative]
print(f"相对于原点的 x 坐标（米）：{x_positions_m}")
# 将米转换为像素，并添加占据区域
for x_m in x_positions_m:
    # 中心点坐标 (x_m, y=0) 转为像素坐标
    # center_x_px = int((x_m / resolution)- origin[0]) 
    center_x_px = int((x_m - origin[0]) / resolution)
    center_y_px = int(( origin[1]) / resolution)

    half_len_px = int(square_size_m / (2 * resolution))

    # 计算像素区域范围（防止越界）
    x_min = center_x_px - half_len_px
    x_max = center_x_px + half_len_px
    y_min = center_y_px - half_len_px
    y_max = center_y_px + half_len_px
    print(f"中心点坐标：({x_m - origin[0]}, {origin[1]})")
    print(f"正方形区域范围：({x_min}, {y_min}) 到 ({x_max}, {y_max})")
    # 设置为占据（黑色，值为0）
    map_array[x_min:x_max,y_min:y_max] = 0

# 保存修改后的地图图像
output_image_path = "fused_map_with_obstacles.pgm"
imageio.imwrite(output_image_path, map_array)

print(f"已保存带障碍物的新地图：{output_image_path}")
