# pothole_detection

基于 E1R 固态雷达（120°×90°）检测地面沟壑（负障碍物），用于防矿卡坠沟。

## 原理

雷达水平安装、安装高度为 `h`，俯角为 β 的线束打到水平地面的水平距离为
`x = h·cot(β)`。同一水平角方向上相邻两条线束（近→远）的理论落点间距为
`Δx_th = h·(cot β_far − cot β_near)`。

若两条线束之间横跨一条沟（沟底被近沿遮挡、雷达扫不到），远处线束直接落到沟的
远沿或沟后地面，实测间距 `Δx_meas` 明显大于理论值。据此判定：

```text
ratio = Δx_meas / Δx_th > ditch_ratio_thresh  且  Δx_meas − Δx_th > ditch_min_width
```

沟的近沿点按横向连续列数聚类后发布为 `/front/ditch_cloud`（intensity 编码间距比）。

## 定向检测与告警融合（AutoDTH 契约 §3.1 / §6.2）

检测实例订阅 gate 镜像 `/control/cmd_gate/cmd_vel` 自判运动方向，按方向分配检测重点
（四态判定实现为共用头 `perception_common/include/perception_common/motion_state.hpp`，
与 cluster 障碍物检测共用同一份）：

| 运动方向 | 判定 | 参与检测 |
| --- | --- | --- |
| 前进 | `linear.x ≥ +motion_v_thresh` | 仅前雷达 |
| 后退 | `linear.x ≤ −motion_v_thresh` | 仅后雷达 |
| 原地旋转 | `\|linear.x\| < v` 且 `\|angular.z\| ≥ motion_w_thresh` | 前后均检测 |
| 停车 | 两轴均低于阈值（含镜像断流 > `motion_timeout_s`） | 前后均检测 |

不参与检测的一侧整帧跳过点云处理（省算力），但仍按帧发布安全告警（`999.0` + `SAFE`），
使融合侧能区分「在线但未检测」与「节点掉线」；方向切换后下一帧即恢复（≪200 ms）。

话题链路（`/perception/edge_warning` / `/perception/stop_command` 是契约话题，各只能有一个发布者，
发布者为 `perception_common` 包的共用融合节点 `warning_fusion`，由它同时融合 cluster 的障碍物告警）：

```text
pothole_detection_front ─► /pothole_detection_front/edge_warning_raw ┐
pothole_detection_rear  ─► /pothole_detection_rear/edge_warning_raw  ├─► warning_fusion ┬─► /perception/edge_warning
cluster_front (障碍物)   ─► /cluster_front/obstacle_warning_raw       │                 ├─► /perception/obstacle_warning
cluster_rear  (障碍物)   ─► /cluster_rear/obstacle_warning_raw        ┘                 └─► /perception/stop_command
```

融合规则与急停联动详见 `perception_common/README.md`：边坡取等级更高的一路（同级取距离更近），
默认 10 Hz——前进时后侧原始告警恒为 SAFE，取高等级自然只体现前侧结果；停车/原地旋转时
两侧都在检测，取高等级即「综合前后雷达警告级别」；某侧断流超时按 `999.0` / `SAFE` 参与融合。

## 急停联动（契约 §5.1 / §5.2 / §7）

边坡等级达 `LEVEL_DANGER` 时，融合节点同周期发布
`/perception/stop_command`（`stop=true`、`reason=REASON_EDGE_DANGER`、
`confidence` 由距离线性换算、`description` 形如 `前方 4.0m 边坡`）；
障碍物进入 `obstacle_danger_dist` 时以 `reason=1/2/3` 参与同一路 stop 决策（优先级最高者胜出）。
消抖、回退、锁存规则与阈值说明见 `perception_common/README.md`。

## 编译

```bash
cd <workspace>
colcon build --packages-select perception_common pothole_detection
```

## 运行

```bash
source install/setup.bash
ros2 launch pothole_detection pothole_detection.launch.py
# 契约话题发布端（边坡 + 障碍物共用）：
ros2 launch perception_common warning_fusion.launch.py

# 或直接运行单个检测节点（必须带【两个】参数文件：共享阈值 + 本包参数）
ros2 run pothole_detection pothole_detection_node --ros-args \
  --params-file install/perception_common/share/perception_common/config/params.yaml \
  --params-file install/pothole_detection/share/pothole_detection/config/params.yaml \
  -p lidar_name:=front -p mount_height:=1.2
```

## 依赖

- ROS 2
- PCL (Point Cloud Library)
- pcl_conversions
- `perception_common`（共用件：运动方向门控头 + 契约话题融合/急停节点 `warning_fusion`）

## 参数

> 边坡分档阈值 `edge_danger_dist` / `edge_caution_dist` 定义在 **`perception_common/config/params.yaml`
> 顶部的 `/**` 共享段**（该文件被所有感知节点加载，`/**` 通配任意节点名）——检测实例与融合节点同源，
> 代码里没有默认值，未提供则节点启动即失败。因此运行/调试本节点必须同时加载两个参数文件
> （共享阈值 + 本包参数），见上方「运行」。
> 注意（Galactic 实测）：**参数文件里的键优先于命令行 `-p`**，要改值请改文件（或另写参数文件）。

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `lidar_name` | "" | front / rear，推导默认话题与 frame_id |
| `mount_height` | 1.2 | 雷达安装高度（米） |
| `horiz_res_deg` | 0.1 | 列分辨率，须与雷达原生水平分辨率一致 |
| `min_vert_angle_deg` | -45.0 | 参与检测的最小垂直角（最近处） |
| `max_vert_angle_deg` | -1.0 | 参与检测的最大垂直角（最远处，须 < 0） |
| `max_range` / `min_range` | 30.0 / 0.5 | 参与检测的水平距离范围（米） |
| `ditch_ratio_thresh` | 2.0 | 间距比阈值 |
| `ditch_min_width` | 0.5 | 最小沟宽（米） |
| `cluster_dist` | 1.5 | 沟沿点聚类距离（米） |
| `min_points_per_ditch` | 5 | 每段沟最少点数（横向连续列数） |
| `voxel_leaf_size` | 0.05 | 体素降采样叶子尺寸（米），<=0 关闭 |
| `cliff_enable` | true | 是否启用悬崖检测 |
| `cliff_max_range` | 7.0 | 悬崖边缘最大水平距离（米） |
| `cliff_min_points` | 3 | 列内最少点数才判悬崖 |
| `cliff_vert_margin_deg` | 1.0 | 最远点距 `max_vert_angle` 的最小角裕量（度） |
| `edge_warning_raw_topic` | `~/edge_warning_raw` | 本实例原始边坡告警话题（由 warning_fusion 消费） |
| `edge_danger_dist` / `edge_caution_dist` | **4.0 / 10.0** | 最近距离分档阈值（米）→ DANGER / CAUTION。**唯一真源在 `perception_common/config/params.yaml` 的 `/**` 共享段**（检测实例与融合节点同源），代码无默认值 |
| `edge_heartbeat_hz` / `edge_timeout_s` | 10.0 / 0.5 | 告警心跳频率（契约 ≥5Hz）/ 点云断流判定超时（秒） |
| `motion_gate_enable` | true | 是否启用运动方向定向检测门控 |
| `motion_cmd_topic` | `/control/cmd_gate/cmd_vel` | gate 镜像话题（只订阅，不发布 cmd_vel） |
| `motion_v_thresh` / `motion_w_thresh` | 0.05 / 0.05 | 四态判定阈值（m/s、rad/s） |
| `motion_timeout_s` | 0.5 | 镜像断流超时（秒）→ 按停车处理 |

融合节点（`perception_common` 包的 `warning_fusion`）参数见 `perception_common/README.md`：
契约话题名、四路原始告警来源、`heartbeat_hz`（10.0）、`source_timeout_s`（0.5）、
急停联动（`stop_command_topic` / `stop_on_danger` / `stop_debounce_frames`）与
confidence 换算用的阈值（取自共享段，代码无默认值）。

调试命令：

```bash
# 看门控效果：被停用的一侧恒为 999.0 / SAFE，启用侧为真实检测结果
ros2 topic echo /pothole_detection_front/edge_warning_raw
ros2 topic echo /pothole_detection_rear/edge_warning_raw
# 看融合输出（等级更高的一路，日志含取哪一侧）
ros2 topic echo /perception/edge_warning
# 看急停联动（DANGER 时应为 stop=true / reason=4；障碍物触发时 reason=1/2/3）
ros2 topic echo /perception/stop_command
ros2 topic hz /perception/stop_command
# 融合日志每秒一条，带当前运动方向、两侧原始告警与急停状态（便于定位“为何取前/为何急停”）
# 融合: 运动=前进 边坡[前 DANGER 4.0m 后 SAFE 999.0m → DANGER 4.0m 取前] 障碍[前 无 999.0m 后 无 999.0m → 无 999.0m 取前] 急停=ON(边坡 DANGER)
# 急停状态翻转时另有独立日志：急停触发(WARN) / 急停解除(INFO)
# 方向取值：前进 / 后退 / 原地旋转 / 停车 / 无信号（未收到或断流镜像，按停车处理）
# 手动注入运动方向（前进 / 后退 / 原地旋转 / 停车）
ros2 topic pub -r 10 /control/cmd_gate/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.3}, angular: {z: 0.0}}'
# 离线验证急停联动（无需雷达）：向两侧原始告警话题灌 DANGER 帧
ros2 topic pub -r 10 /pothole_detection_front/edge_warning_raw dth_messages/msg/EdgeWarning \
  '{warning: true, dist_to_edge_m: 4.0, level: 2}'
```


悬崖（下方无回波）检测结果发布为 `/front/cliff_cloud` / `/rear/cliff_cloud`（intensity 固定 255），
RViz 中悬崖边界为橙色折线，沟壑边界为红色折线。
体素化降采样后的点云发布到 `/front/voxel_cloud` / `/rear/voxel_cloud`（便于调试查看降采样效果）。

## 在 RViz 中查看

1. 添加 `PointCloud2` display
2. 设置话题为 `/front/ditch_cloud`
3. 颜色越亮表示间距比越大（越可能是沟）
