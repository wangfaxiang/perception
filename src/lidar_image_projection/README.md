
<!-- 播放rosbag -->
ros2 bag play rosbag2_1/ --loop --remap /rslidar_points:=/front/rslidar_points
ros2 bag play rosbag2_2/ --loop --remap /rslidar_points:=/rear/rslidar_points

## 启动激光雷达
ros2 launch rslidar_sdk start.py

## 启动centerpoint检测
ros2 run centerpoint centerpoint

## 显示检测框
python3 src/centerpoint/scripts/boxs_to_marker.py
/usr/bin/python3 src/centerpoint/scripts/boxs_to_marker.py

## 编译

```bash
cd <workspace>

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --packages-select centerpoint
s
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --packages-select lidar_image_projection

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --packages-select pothole_detection

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --packages-select ground_segmentation

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --packages-select cluster
```

## 运行

前雷达与后雷达各启动一个节点实例，分别订阅并独立做边坡监测：

| 节点实例 | 输入话题 | 输出话题前缀 | frame_id |
|---|---|---|---|
| `lidar_image_projection_front` | `/front/rslidar_points` | `/front/lidar_*` | `front_rslidar` |
| `lidar_image_projection_rear`  | `/rear/rslidar_points`  | `/rear/lidar_*`  | `rear_rslidar` |

```bash
# 使用 launch 文件（同时启动 front + rear 两个实例）
source install/setup.bash
source /opt/ros/galactic/setup.bash
ros2 launch lidar_image_projection lidar_image_projection.launch.py

source install/setup.bash
source /opt/ros/galactic/setup.bash
ros2 launch pothole_detection pothole_detection.launch.py

# 或分别直接运行节点
ros2 run lidar_image_projection lidar_image_projection_node \
  --ros-args -r __node:=lidar_image_projection_front \
  -p lidar_name:=front -p enable_edge_detection:=true

ros2 run lidar_image_projection lidar_image_projection_node \
  --ros-args -r __node:=lidar_image_projection_rear \
  -p lidar_name:=rear -p enable_edge_detection:=true
```

## 依赖

- ROS 2 (Galactic 或更高)
- OpenCV 4.x
- PCL (Point Cloud Library)
- cv_bridge
- pcl_conversions

## 在 RViz 中查看

1. 添加 `Image` display
2. 设置话题为:
   - 前雷达边缘图: `/front/lidar_edge_image`
   - 后雷达边缘图: `/rear/lidar_edge_image`
3. 红色线条即为检测到的边坡边缘（高度突变处）

其他图像话题:
- 原始高度图: `/front/lidar_height_image`、`/rear/lidar_height_image`
- 膨胀后高度图: `/front/lidar_dilated_height_image`、`/rear/lidar_dilated_height_image`
- 边缘点云: `/front/lidar_edge_cloud`、`/rear/lidar_edge_cloud`

