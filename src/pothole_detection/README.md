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

## 编译

```bash
cd <workspace>
colcon build --packages-select pothole_detection
```

## 运行

```bash
source install/setup.bash
ros2 launch pothole_detection pothole_detection.launch.py

# 或直接运行单个节点
ros2 run pothole_detection pothole_detection_node \
  --ros-args -p lidar_name:=front -p mount_height:=1.2
```

## 依赖

- ROS 2
- PCL (Point Cloud Library)
- pcl_conversions

## 参数

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

悬崖（下方无回波）检测结果发布为 `/front/cliff_cloud` / `/rear/cliff_cloud`（intensity 固定 255），
RViz 中悬崖边界为橙色折线，沟壑边界为红色折线。
体素化降采样后的点云发布到 `/front/voxel_cloud` / `/rear/voxel_cloud`（便于调试查看降采样效果）。

## 在 RViz 中查看

1. 添加 `PointCloud2` display
2. 设置话题为 `/front/ditch_cloud`
3. 颜色越亮表示间距比越大（越可能是沟）
