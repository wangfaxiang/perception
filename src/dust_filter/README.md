# dust_filter —— 矿山扬尘过滤

对镭神 LS 雷达点云做**基于时间一致性的去扬尘**：灰尘/扬尘是瞬态点（每帧位置随机），
真实障碍物/地面是稳定点（连续多帧落在同一网格）。把最近 `n` 帧历史点云堆到当前帧网格，
某网格被 `min_occupied_frames` 帧以上占据则保留，否则视为扬尘剔除；
并对其周围 **3D 26 邻域**（水平角/垂直角/水平距离三轴各 ±1）网格中“当前帧有点但历史不稳定”的扬尘点一并剔除，增强对扬尘团边缘的抑制；仅当前帧为空的无点网格不参与扩散。

## 数据流

```
/ls/lslidar_point_cloud  -->  dust_filter  -->  /ls/lslidar_point_cloud_filtered
   (sensor_msgs/PointCloud2)                       (sensor_msgs/PointCloud2，保留全部字段)
```

## 构建

```bash
source /opt/ros/galactic/setup.bash
cd ~/wfx/work/lin_gong/perception
colcon build --packages-select dust_filter --symlink-install
source install/setup.bash
```

## 运行

```bash
ros2 launch dust_filter dust_filter.launch.py
```

## 参数（config/params.yaml）

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `input_topic` | `/ls/lslidar_point_cloud` | 订阅话题 |
| `output_topic` | `/ls/lslidar_point_cloud_filtered` | 发布话题 |
| `frame_id` | `""` | 输出 frame_id，空则沿用输入 |
| `horiz_angle_resolution` | `0.2` | 水平角分辨率（°） |
| `vert_angle_resolution` | `0.2` | 垂直角分辨率（°） |
| `range_resolution` | `0.1` | 近处基准距离分辨率（m） |
| `range_scale` | `20.0` | 距离每增大该值，距离分辨率翻倍（m） |
| `history_frames` | `2` | 保留的历史帧数 n |
| `min_occupied_frames` | `2` | 网格被 ≥ 该值帧历史占据才保留（1..n） |
| `use_tf` | `true` | 是否用 TF 做运动补偿 |
| `target_frame` | `odom` | 固定坐标系（`use_tf=true` 时生效） |

## 可行性说明（0.5 m/s 场景）

- 矿车 0.5 m/s、雷达约 10 Hz → **每帧位移约 5 cm**；n=2 时最老帧与当前帧相差约 2 帧的位移量。
- **不做运动补偿**会把静态障碍物（正前方/后方的墙、小石块）在网格里"滑动"而误删。
- 默认开启 `use_tf`，把历史帧与当前帧都变换到 `target_frame` 再堆叠，静态物每帧落回同一网格；
  查不到 TF 时自动退回雷达坐标系堆叠（等价于假设车辆静止）。

## 调参建议

- 若误删真实障碍（过严）：把 `min_occupied_frames` 降到 `1`，或放大分辨率（如 `range_resolution` 0.1→0.2、角分辨率 0.2°→0.5°、调小 `range_scale` 使远距离分辨率增长更快）。
- 若扬尘残留（过松）：提高 `min_occupied_frames` 或增加 `history_frames`。
- 矿车颠簸大、odom 抖动明显时，优先放大角/距离分辨率而不是调低阈值。
- 距离自适应：有效距离分辨率 = `range_resolution × (1 + 水平距离 / range_scale)`，远处网格更粗、抗抖动与测距噪声能力更强。
- 26 邻域扩散会显著增强去扬尘力度：若障碍边缘被误删过多，可降低 `min_occupied_frames` 或调小分辨率（让邻域覆盖的物理范围更小）。
- 启动后前 `history_frames` 帧因历史为空，输出为空点云，属正常现象。

## 已知局限

- 运动中的其它车辆/行人也会被部分滤除（同为"时间不一致"点），矿山场景一般可接受。
- 无 TF/odom 时只能假设车辆静止，高速行驶下效果会明显变差。
