## 编译
```bash
cd <workspace>

source /opt/ros/galactic/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --packages-select lidar_leveling

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --packages-select centerpoint

export PATH=/usr/bin:$PATH
colcon build --packages-select lidar_centerpoint \
  --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DTENSORRT_ROOT=/home/promote/wfx/tools/TensorRT/TensorRT-8.6.0.12 \
  -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-11.8 \
  -DCMAKE_BUILD_TYPE=Debug

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --packages-select lidar_image_projection

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --packages-select pothole_detection

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --packages-select ground_segmentation

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --packages-select cluster

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --packages-select range_image_segmentation
```
-------------------------------------------------------------------------------------------

# 前进：v=+0.3 m/s、ω=0，10 Hz 恒发(条件（建议默认阈值 v=0.05 m/s、ω=0.10 rad/s）)
ros2 topic pub -r 10 /control/cmd_gate/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: -0.1}, angular: {z: 0.0}}'

-------------------------------------------------------------------------------------------
## 一键启动（推荐：等价于下面「步骤」的 1~7 全部命令）
```bash
cd <workspace>
source /opt/ros/galactic/setup.bash
source install/setup.bash
colcon build --packages-select perception_common     # 首次：装上 bringup launch 文件

# 默认：回放 bag（rosbag2_stone_front / rosbag2_stone_rear，--loop）
#       + 调平 + 边坡/沟壑 + 地面滤除 + 聚类 + 警告融合 + front/rear 两个 rviz
ros2 launch perception_common perception_bringup.launch.py

# 数据源二选一：用真实雷达驱动（不下发 bag）
ros2 launch perception_common perception_bringup.launch.py use_bag:=false

# 只跑链路不看图 / 换 bag / 不循环
ros2 launch perception_common perception_bringup.launch.py start_rviz:=false
ros2 launch perception_common perception_bringup.launch.py \
  front_bag:=rosbag2_obstacle_front rear_bag:=rosbag2_obstacle_rear bag_loop:=false
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `use_bag` | `true` | `true` 回放 bag；`false` 启动 `rslidar_sdk` 雷达驱动 |
| `front_bag` / `rear_bag` | `rosbag2_stone_front` / `rosbag2_stone_rear` | bag 目录 |
| `bag_loop` | `true` | bag 是否 `--loop` 循环播放 |
| `bag_delay_s` | `3.0` | 延迟多久开始回放（等下游节点先就位） |
| `workspace_dir` | 空（= 当前目录） | bag 与 `.rviz` 的相对路径基准 |
| `start_rviz` | `true` | 是否启动 `front.rviz` / `rear.rviz` 两个 rviz2 |
| `front_rviz_config` / `rear_rviz_config` | `front.rviz` / `rear.rviz` | rviz 配置文件名 |
| `rviz_delay_s` | `3.0` | 延迟多久启动 rviz2 |
| `rslidar_config_path` | 空 | 驱动配置文件；留空 = 用 `rslidar_sdk` 包内 `config/config.yaml` |

> 请在 `<workspace>` 根目录执行：`rosbag2_*` 与 `front.rviz`/`rear.rviz` 都在根目录，
> 相对路径按启动时的当前目录解析（也可用 `workspace_dir:=` 显式指定）。

-------------------------------------------------------------------------------------------
## 步骤
source /opt/ros/galactic/setup.bash
source install/setup.bash

## 启动激光雷达
ros2 launch rslidar_sdk start.py
1.发布原始点云
/front/rslidar_points
/rear/rslidar_points

## 将点云调平
ros2 launch lidar_leveling lidar_leveling.launch.py
2.调平后点云
/front/rslidar_points_leveled
/rear/rslidar_points_leveled

-------------------------------------------------------------------------------------------

## 启动边坡，沟壑检测节点
ros2 launch pothole_detection pothole_detection.launch.py
3.边界点云（点云类型）
/front/ditch_cloud
/rear/ditch_cloud
4.边界线（红加黄，MarkerArray类型）
/front/ditch_line
/rear/ditch_line

----------------------------------------------------------------------------------------------

## 启动点云地面提取
ros2 launch ground_segmentation ransac_ground_filter.launch.py
输出话题：
/front/no_ground
/rear/no_ground

## 启动点云聚类
ros2 launch cluster cluster.launch.py                          # 聚类 ×2 + 融合，一条命令
<!-- ros2 launch cluster cluster.launch.py start_warning_fusion:=false   # 融合已在别处起过时 -->
输出话题：
/front/box/cluster/euclidean/marker（MarkerArray类型）
/rear/box/cluster/euclidean/marker（MarkerArray类型）

## 启动警告融合
ros2 launch perception_common warning_fusion.launch.py

---------------------------------------------------------------------------------------------
## 聚类方法2
ros2 launch range_image_segmentation range_image_segmentation.launch.py

----------------------------------------------------------------------------------------------
<!-- 播放rosbag -->
ros2 bag play rosbag2_1/ --loop --remap /rslidar_points:=/front/rslidar_points
ros2 bag play rosbag2_2/ --loop --remap /rslidar_points:=/rear/rslidar_points


## 启动centerpoint检测
ros2 run centerpoint centerpoint

## 显示检测框
python3 src/centerpoint/scripts/boxs_to_marker.py
/usr/bin/python3 src/centerpoint/scripts/boxs_to_marker.py



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

