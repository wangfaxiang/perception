# cluster

基于 ransac 地面过滤后的非地面点云做聚类（欧氏 / 区域生长），输出障碍物包围盒，
并按 AutoDTH 契约 §3.1/§5.3 发布**单雷达原始障碍物告警**，供共用融合节点合成
`/perception/obstacle_warning` 与 `/perception/stop_command`。

对接文档：`dth_perception 感知模块 ROS 2 接口对接文档` v1.1（AutoDTH 项目，
<http://192.168.2.100:81/project_2026/o27/docs/autodht_site/dth_perception_interface/>）

## 数据流

```text
/front/no_ground ─► cluster_front ─┬─► /front/box/cluster/{euclidean,region_rowing}(+marker)   （调试用）
                                   └─► /cluster_front/obstacle_warning_raw ┐
/front/no_ground ─► cluster_rear  ─┬─► /rear/box/cluster/...               ├─► warning_fusion ─┬─► /perception/obstacle_warning
                                   └─► /cluster_rear/obstacle_warning_raw  ┘  (perception_common) └─► /perception/stop_command
```

`warning_fusion` 与边坡检测（`pothole_detection`）共用，见 `perception_common/README.md`：
障碍物两路按**类别优先（人员 > 车辆 > 其他）、同级取最近**取一路发布；进入
`obstacle_danger_dist` 时以 `reason=1/2/3` 参与急停决策（与边坡 DANGER 合并，取优先级最高者）。

## 定向检测（契约 §3.1 / §6.2）

节点订阅 gate 镜像 `/control/cmd_gate/cmd_vel` 自判运动方向并门控——实现为共用头
`perception_common/include/perception_common/motion_state.hpp`（与边坡检测同一份）：

| 运动方向 | 判定 | 参与检测 |
| --- | --- | --- |
| 前进 | `linear.x ≥ +motion_v_thresh` | 仅前雷达 |
| 后退 | `linear.x ≤ −motion_v_thresh` | 仅后雷达 |
| 原地旋转 | `\|linear.x\| < v` 且 `\|angular.z\| ≥ motion_w_thresh` | 前后均检测 |
| 停车 | 两轴均低于阈值（含镜像断流 > `motion_timeout_s`） | 前后均检测 |

被门控的一侧整帧跳过聚类（省算力），但仍按帧发布安全默认告警（`detected=false` / `type=0` /
`dist_m=999.0`），使融合侧能区分「在线但未检测」与「节点掉线」；方向切换后下一帧即恢复。

## 障碍物告警字段（契约 §5.3）

| 字段 | 本包取值 |
| --- | --- |
| `detected` | 最近威胁物水平距离 ≤ `obstacle_warn_dist` → true，否则 false |
| `obstacle_type` | 由 `obstacle_type` 参数给出：**cluster 只做几何聚类、无类别识别能力，默认 3=其他**；联调或后续接入分类器时可强制 1=人员 / 2=车辆 |
| `obstacle_x` / `obstacle_y` | **胜出簇内距雷达最近的点**（对大体积/贴墙目标更保守），坐标系 = `header.frame_id`（该雷达帧，如 `front_rslidar`） |
| `dist_m` | `√(x²+y²)`，与坐标严格自洽（契约要求） |
| `description` | 中文简述，如 `前雷达 左前 3.2m 障碍物(0.8×0.6m)`；方位按**本雷达坐标系**给出并带雷达前缀，避免把后雷达的「前方」误读为车体前方 |

- 未检出时：`detected=false`、`obstacle_type=0`、坐标 `0.0`、`dist_m=999.0`、`description` 空。
- 位置**未做车体系变换**（系统侧按 `frame_id` 自行变换；如需统一到 `base_link`，应在检测实例
  加安装外参后输出）。
- 心跳：默认 10 Hz（契约 §4.3 必须 ≥5 Hz）；点云断流超 `obstacle_timeout_s` → 安全默认值。

## 编译 / 运行

```bash
colcon build --packages-select perception_common cluster
source install/setup.bash

# 前后雷达聚类实例 + 共用报警融合节点（契约三话题发布端），本条命令默认一并拉起
ros2 launch cluster cluster.launch.py

# 融合节点已在别处启动（单独 launch / 调试实例）时用开关关闭，避免重复发布契约话题
ros2 launch cluster cluster.launch.py start_warning_fusion:=false

# 也可以只单独启动融合节点（与边坡检测共用同一个融合节点）
ros2 launch perception_common warning_fusion.launch.py
```

> `warning_fusion` 订阅 `cluster_front/rear` 与 `pothole_detection_front/rear` 四路原始告警；
> 只跑 cluster 时边坡两路按断流（`source_timeout_s`）计入安全默认值，属预期行为。

## 参数（`config/params.yaml`，按节点名分段）

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `lidar_name` | "" | front / rear，决定门控归属与默认话题 |
| `input_topic` | `/<lidar>/no_ground` | ransac 地面过滤后的非地面点云 |
| `cluster_tolerance` / `min_cluster_size` / `max_cluster_size` | 0.2 / 15 / 100000 | 欧氏聚类参数 |
| `region_rowing` | false | 是否同时跑区域生长聚类（调试） |
| `obstacle_warning_raw_topic` | `~/obstacle_warning_raw` | 本实例原始告警话题（由 warning_fusion 消费） |
| `obstacle_source` | `euclidean` | 哪一路聚类结果作为告警来源：`euclidean` / `region_rowing` |
| `obstacle_type` | 3 | 强制类别（1 人员 / 2 车辆 / 3 其他）；0 为契约保留值，禁止配置 |
| `obstacle_warn_dist` | **15.0** | 威胁物上报距离（米）。**唯一真源在 `perception_common/config/params.yaml` 的 `/**` 共享段**，代码无默认值，未提供则启动即失败 |
| `frame_id` | `<lidar>_rslidar` | 无点云时的占位帧；有点云时用输入点云的 frame |
| `obstacle_heartbeat_hz` / `obstacle_timeout_s` | 10.0 / 0.5 | 告警心跳频率（契约 ≥5Hz）/ 点云断流判定超时（秒） |
| `motion_gate_enable` | true | 是否启用运动方向定向检测门控 |
| `motion_cmd_topic` | `/control/cmd_gate/cmd_vel` | gate 镜像话题（只订阅，严禁发布 cmd_vel） |
| `motion_v_thresh` / `motion_w_thresh` | 0.05 / 0.05 | 四态判定阈值（m/s、rad/s） |
| `motion_timeout_s` | 0.5 | 镜像断流超时（秒）→ 按停车处理 |

> ⚠️ 本节点必须加载**两个**参数文件（共享阈值 + 本包参数），见 launch。
> 单独 `ros2 run` 时要显式给 `--params-file`（`obstacle_warn_dist` 无代码默认值）。

## 调试

```bash
# 原始告警（本实例输出的最近威胁物；被门控的一侧恒为未检出/999.0）
ros2 topic echo /cluster_front/obstacle_warning_raw
# 契约话题（融合后）
ros2 topic echo /perception/obstacle_warning
# 手动注入运动方向（门控逐帧生效）
ros2 topic pub -r 10 /control/cmd_gate/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.3}, angular: {z: 0.0}}'
```
