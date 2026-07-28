## 启动激光雷达
ros2 launch rslidar_sdk start.py

## 启动centerpoint检测
ros2 run centerpoint centerpoint

## 显示检测框
python3 src/centerpoint/scripts/boxs_to_marker.py

## 编译

```bash
cd <workspace>

colcon build --packages-select pothole_detection

```

## 运行

```bash
# 使用 launch 文件（默认参数）
source install/setup.bash
source /opt/ros/galactic/setup.bash
ros2 launch pothole_detection pothole_detection.launch.py

# 或直接运行节点
ros2 run pothole_detection pothole_detection_node \
  --ros-args -p enable_edge_detection:=true
```

## 依赖

- ROS 2 (Galactic 或更高)
- OpenCV 4.x
- PCL (Point Cloud Library)
- cv_bridge
- pcl_conversions

## 在 RViz 中查看

1. 添加 `Image` display
2. 设置话题为 `/pothole_edge_image`
3. 红色线条即为检测到的边坡边缘（高度突变处）
