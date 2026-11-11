# 感知链路启动说明（perception）

前后雷达各一路，整条链路按「驱动 → 调平 → 预处理 → 边坡/障碍物检测 → 报警融合」编排。

## 环境准备

```bash
cd /home/promote/wfx/work/lin_gong/perception
source /opt/ros/galactic/setup.bash
source install/setup.bash
```

> 提示：若使用 miniconda，需先 `conda deactivate`，确保 `colcon` / `ros2` 用系统 `/usr/bin/python3`。

---

## 1. 启动镭神雷达驱动

```bash
ros2 launch lslidar_driver lslidar_ls_double_launch.py
```

- 前后两颗雷达各一个驱动实例，并附带 `lidar_status_monitor` 数据流监测。
- 输出话题：`/front/rslidar_points`、`/rear/rslidar_points`（`sensor_msgs/PointCloud2`）。

## 2. 启动调平节点

```bash
ros2 launch lidar_leveling lidar_leveling.launch.py
```

- 输入：`/front|rear/rslidar_points`
- 输出：`/front|rear/rslidar_points_leveled`

## 3. 启动预处理节点

```bash
ros2 launch pointcloud_preprocess pointcloud_preprocess.launch.py
```

- 输入：`/front|rear/rslidar_points_leveled`
- 输出：`/front|rear/leveled_filtered_smoothed`（全量扬尘过滤 + 降采样 + 高度平滑）

## 4. 启动边坡检测节点（range_image_segmentation）

```bash
ros2 launch range_image_segmentation range_image_segmentation.launch.py
```

- 输入：`/front|rear/all_filter`（或 `/front/leveled_filtered_smoothed`）
- 输出：`/front|rear/ditch_cloud`、`/front|rear/ditch_line`，以及单雷达原始告警 `~/edge_warning_raw`

## 5. 启动障碍物检测节点（elevation_obstacle_detection）

```bash
ros2 launch elevation_obstacle_detection elevation_obstacle_detection.launch.py
```

- 输入：`/front|rear/leveled_filtered_smoothed`
- 输出：`/front|rear/elevation_obstacle/{cloud, boxs, marker, ground_grid}`，以及单雷达原始告警 `~/obstacle_warning_raw`

## 6. 启动融合报警节点

```bash
ros2 launch perception_common warning_fusion.launch.py
```

- 输入：边坡 `edge_warning_raw`（range_image_segmentation）+ 障碍物 `obstacle_warning_raw`（elevation_obstacle_detection）
- 输出（契约三话题，唯一发布者）：
  - `/perception/edge_warning`
  - `/perception/obstacle_warning`
  - `/perception/stop_command`

## 7. 一键启动以上所有节点

```bash
ros2 launch perception_common perception_bringup_v2.launch.py use_bag:=false
```

等价于按顺序启动：镭神雷达驱动 → 调平 → 预处理 → 边坡检测 → 障碍物检测 → 报警融合，并拉起 `front.rviz` / `rear.rviz`。

> 旧链路（无预处理/高程障碍物检测）仍保留在 `perception_bringup.launch.py`。

常用变体：

```bash
# 回放 rosbag（离线调试，默认 --loop），不启动真实雷达
ros2 launch perception_common perception_bringup_v2.launch.py

# 切速腾 rslidar_sdk
ros2 launch perception_common perception_bringup_v2.launch.py use_bag:=false lidar_driver:=rslidar

# 只跑链路、不启动 rviz
ros2 launch perception_common perception_bringup_v2.launch.py use_bag:=false start_rviz:=false

# 换 bag / 不循环
ros2 launch perception_common perception_bringup_v2.launch.py \
  front_bag:=rosbag2_obstacle_front rear_bag:=rosbag2_obstacle_rear bag_loop:=false
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `use_bag` | `true` | `true` 回放 bag；`false` 启动真实雷达驱动 |
| `lidar_driver` | `lslidar` | `use_bag:=false` 时的驱动：`lslidar`=镭神（默认）；`rslidar`=速腾 |
| `front_bag` / `rear_bag` | `rosbag2_stone_front` / `rosbag2_stone_rear` | 回放的 bag 目录 |
| `bag_loop` | `true` | bag 是否 `--loop` 循环 |
| `bag_delay_s` | `3.0` | 延迟多久开始回放（等下游节点就位） |
| `workspace_dir` | 空（=当前目录） | bag 与 `.rviz` 的相对路径基准 |
| `start_rviz` | `true` | 是否启动 `front.rviz` / `rear.rviz` |
| `front_rviz_config` / `rear_rviz_config` | `front.rviz` / `rear.rviz` | rviz 配置文件名 |
| `rviz_delay_s` | `3.0` | 延迟多久启动 rviz2 |
| `rslidar_config_path` | 空 | 速腾驱动配置；留空用 `rslidar_sdk` 包内 `config/config.yaml` |

> 请在工作区根目录执行：`rosbag2_*` 与 `front.rviz`/`rear.rviz` 都在根目录，相对路径按当前目录解析（也可用 `workspace_dir:=` 显式指定）。
