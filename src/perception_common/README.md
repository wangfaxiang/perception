# perception_common

感知侧**共用件**：把「只能有一份」的东西集中到这里，供边坡检测（`pothole_detection`）与
障碍物检测（`cluster`）两个包共用。

对接文档：`dth_perception 感知模块 ROS 2 接口对接文档` v1.1（AutoDTH 项目，
<http://192.168.2.100:81/project_2026/o27/docs/autodht_site/dth_perception_interface/>）

| 共用件 | 位置 | 谁用 |
| --- | --- | --- |
| 运动方向判定 / 定向检测门控（契约 §3.1 / §6.2） | `include/perception_common/motion_state.hpp` | 两个检测包的每个雷达实例 |
| 契约告警融合 + 急停决策节点（契约 §4 / §5 / §7） | `src/warning_fusion_node.cpp` | 全系统唯一的 `/perception/*` 发布者 |
| 共享阈值参数（唯一真源） | `config/params.yaml` 顶部 `/**` 段 | 所有感知节点（launch 里一并加载） |

## 1. 为什么共用

- **运动方向**：契约 §3.1 的定向策略（前进只测前、后退只测后、停车/原地旋转双向均测）对
  边坡与障碍物完全一致，两份实现必然跑偏 → 头文件一份，两个检测包各自按 `lidar_name`
  建一个实例。
- **融合报警**：契约 §4.1 的三路话题是冻结契约，系统侧 4 个消费方直接依赖话题名，
  **每路只能有一个发布者**；且「边坡 DANGER」与「障碍物 PERSON/VEHICLE/OTHER」必须在同一个
  周期内合成一条不矛盾的 `stop_command` → 两类检测的原始告警都汇到本节点，
  由它统一发布三路话题。

## 2. 数据流

```text
                        （两类检测实例各自订阅 gate 镜像自判方向并门控）
pothole_detection_front ─► /pothole_detection_front/edge_warning_raw ┐
pothole_detection_rear  ─► /pothole_detection_rear/edge_warning_raw  │
                                                                     ├─► warning_fusion ─┬─► /perception/edge_warning
cluster_front (障碍物)   ─► /cluster_front/obstacle_warning_raw       │                   ├─► /perception/obstacle_warning
cluster_rear  (障碍物)   ─► /cluster_rear/obstacle_warning_raw        ┘                   └─► /perception/stop_command
```

被门控跳过检测的一侧，其原始告警恒为**安全默认值**（`999.0` / `SAFE` / `detected=false`），
所以融合节点只做「取哪一路」，不需要再做方向判断——**运动方向在本节点仅用于日志标注**。

## 3. 融合规则

| 输出 | 规则 | 依据 |
| --- | --- | --- |
| `edge_warning` | 取等级更高的一路，同级取 `dist` 更近者；`warning = (level ≠ LEVEL_SAFE)` | §5.2 |
| `obstacle_warning` | 类别优先（人员 > 车辆 > 其他），同级取 `dist_m` 最近者；无威胁 → `detected=false`/`type=0`/`dist=999.0` | §3.1 / §5.3 |
| `stop_command` | 见下 | §3.3 / §5.1 / §7 |

急停决策（每周期）：

1. **候选 1 边坡**：融合等级 == `LEVEL_DANGER`；`reason=REASON_EDGE_DANGER(4)`，
   `confidence = clamp(1 − d/edge_danger_dist, 0.5, 1)`，`description = "前方/后方 X.Xm 边坡"`。
2. **候选 2 障碍物**：`detected=true` 且 `dist_m ≤ obstacle_danger_dist`；
   `reason` = `obstacle_type`（1 人员 / 2 车辆 / 3 其他），confidence 同式换算，
   `description` 直接沿用检测实例给出的「方位 + 距离 + 类别」。
3. 两个候选**各自独立消抖**（连续 `stop_debounce_frames` 帧**原始告警**成立；按来源帧序号计数，
   不按采样周期——本节点会复用最近一帧最长 `source_timeout_s`，按周期计数会让单帧毛刺蒙混过关）。
4. 合并：`reason` 取数值最小者（= 优先级最高：**人员 1 > 车辆 2 > 其他 3 > 边坡 4**），
   `confidence` / `description` 与胜出候选同源。
   契约未规定两类同时成立时如何取舍，此处按 §5.1 枚举数值顺序（与 §3.1 的类别优先一致）取最严者，
   属实现选择。
5. **回退不消抖**：风险消失后下一周期立即发 `stop=false`（`reason=0`、`confidence=0.0`、
   `description` 空）——这是系统侧操作员复位（`/mission/reset`）的前置条件（§7.3）。
6. **锁存不在感知侧**：`autodth_cmd_gate` 锁存 `stop` 电平；本节点只负责风险持续期间持续发布
   `stop=true`（§4.3 心跳义务）。`stop_on_danger: false` 可整体关闭联动（只发 `stop=false` 心跳）。

心跳与安全默认值（§4.3）：启动即进入安全态并周期发布（默认 10 Hz）；某路原始告警断流超时
按安全默认值参与融合，全部断流即输出安全默认值；`header.stamp` 每帧填节点时钟 `now()`
（自动尊重 `use_sim_time`）。

## 4. 参数

`config/params.yaml` 顶部 `/**` 段（**唯一真源**，被所有感知节点加载；代码无默认值，
未提供则节点启动即失败）：

| 参数 | 值 | 用途 |
| --- | --- | --- |
| `edge_danger_dist` | 4.0 | 边坡 DANGER 阈值（米）——检测实例分档 + 融合侧 confidence 换算 |
| `edge_caution_dist` | 10.0 | 边坡 CAUTION 阈值（米）——仅检测实例分档用 |
| `obstacle_warn_dist` | 15.0 | 障碍物「威胁物」上报距离（米）——仅 cluster 检测实例用 |
| `obstacle_danger_dist` | 5.0 | 障碍物急停阈值（米）——仅融合节点用 |

`warning_fusion` 段：话题（`edge_warning_topic` / `obstacle_warning_topic` / `stop_command_topic`）、
来源（`front/rear_edge_topic`、`front/rear_obstacle_topic`）、`frame_id`（`base_link`）、
`heartbeat_hz`（10.0）、`source_timeout_s`（0.5）、`stop_on_danger`（true）、
`stop_debounce_frames`（2）、运动方向镜像四项（仅日志标注）。

> ⚠️ **Galactic 实测坑**：参数文件里的键优先于命令行 `-p`。想临时改值必须另写参数文件
> （`.vscode/launch.json` 的调试配置因此把节点 `__node:=` 改成调试名，让文件里的节点专属段不生效）。

## 5. 编译与运行

```bash
colcon build --packages-select perception_common
source install/setup.bash
ros2 launch perception_common warning_fusion.launch.py
```

## 6. 离线验证（无需雷达）

```bash
# 安全心跳：三路话题均应 ≥5Hz，且初值为安全默认值
ros2 topic hz /perception/stop_command
ros2 topic echo /perception/stop_command
ros2 topic echo /perception/obstacle_warning

# 注入障碍物威胁（人员 3m）→ obstacle_warning.detected=true、type=1；≤5m → stop=true reason=1
ros2 topic pub -r 10 /cluster_front/obstacle_warning_raw dth_messages/msg/ObstacleWarning \
  '{detected: true, obstacle_type: 1, obstacle_x: 3.0, obstacle_y: 0.0, dist_m: 3.0, description: "前雷达 前方 3.0m 人员"}'

# 注入边坡 DANGER → stop=true reason=4
ros2 topic pub -r 10 /pothole_detection_front/edge_warning_raw dth_messages/msg/EdgeWarning \
  '{warning: true, dist_to_edge_m: 4.0, level: 2}'

# 手动注入运动方向（仅影响日志标注；门控在两个检测实例内）
ros2 topic pub -r 10 /control/cmd_gate/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.3}, angular: {z: 0.0}}'
```

融合日志每秒一条，含两侧原始告警、取哪一侧、当前运动方向与急停状态：

```text
融合: 运动=前进 边坡[前 DANGER 4.0m 后 SAFE 999.0m → DANGER 4.0m 取前] 障碍[前 人员 3.0m 后 无 999.0m → 人员 3.0m 取前] 急停=ON(人员)
```

急停状态翻转时另有独立日志（触发 WARN / 解除 INFO）。
