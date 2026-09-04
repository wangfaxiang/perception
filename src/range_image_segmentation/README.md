# range_image_segmentation

基于**距离图像（range image）**的地面分割 + 非地面障碍物聚类，方法参考
LeGO-LOAM 的 `imageProjection`（`projectPointCloud` / `groundRemoval` / `cloudSegmentation`），
并针对**速腾聚创 E1R 全固态雷达**适配。

## 为什么不用现有 ransac + 聚类

| 维度 | 本包（range image） | 现有 ransac + PCL 聚类 |
|---|---|---|
| 地面分割 | 逐列逐相邻线局部判断，可适应起伏/多段地面 | 全局单一平面，坡 >10° 直接放弃 |
| 效率 | 投影 + 矩阵遍历，近线性，无迭代/KDTree | RANSAC 迭代 + KDTree 聚类，慢 |
| 聚类抗粘连 | 角度阈值天然抗遮挡粘连 | 欧式聚类易把相邻物连成一簇 |

E1R 输出 144 线规则结构点云，正好满足 range image 的前提，因此本方案效率更高、
对矿区/工地起伏地面更鲁棒。

## 数据流

```
/<lidar_name>/rslidar_points_leveled   (调平点云, PointXYZI)
        │
        ▼
 range_image_segmentation
        ├── /<lidar_name>/ground_cloud                地面点云
        ├── /<lidar_name>/segmented_cloud_pure        非地面聚类点云 (intensity=簇号)
        ├── /<lidar_name>/box/cluster/range_image     聚类包围盒 (box_msg/Boxs)
        └── /<lidar_name>/box/cluster/range_image/marker
```

## 算法说明

1. **投影** `projectPointCloud`：对每个点计算距离 r，用 `asin(z/r)` 反算俯仰角得到行号、
   `atan2(y,x)` 反算方位角得到列号，落到 `144 × 180` 的 range image（E1R：120°×90°、144 线）。
2. **地面分割** `groundRemoval`：逐列、在 `0 ~ ground_scan_index` 行内取上下相邻两线的点，
   若其连线与水平的垂直夹角与 `sensor_mount_angle` 之差 ≤ `ground_angle_threshold`，标记为地面。
3. **分割** `cloudSegmentation`：在 range image 上做 4 邻域 BFS 连通域，相邻点用角度阈值
   `tan(segment_theta)` 判定是否属于同一物体；点数过少的簇标记为离群点丢弃。
4. 有效簇按点云算 AABB 包围盒，输出 `box_msg/Boxs` 与 `MarkerArray`。

## 关键适配（相对 LeGO-LOAM）

- **前视 120°、非 360° 环绕**：列映射改为前视方位角区间，BFS 水平边界不做 wrap-around。
- **调平点云无 ring**：行号由俯仰角反算，而非直接读 ring。
- **144 线**：线间角差仅 ~0.625°，`ground_angle_threshold` 需比 VLP-16 时更谨慎标定。

## 参数标定建议

- `vertical_angle_bottom/top`：覆盖调平后点云的俯仰分布。雷达向下俯视安装
  `pitch=+30°`、垂直 FOV 90° 时约 `[-75°, +15°]`（默认值）。
- `num_horizontal_scans`：120° 内每线点数 ≈ 260000 / 帧率 / 144（10Hz 时约 180）。
- `ground_scan_index`：地面在 range image 下方（低行号）。过大易把低矮障碍误判为地面，
  过小会漏地面，需按实际安装高度标定。
- `segment_theta`：越小分割越细，越大越粗（抗欠分割/过分割的取舍）。

## 构建与运行

```bash
colcon build --packages-select range_image_segmentation
source install/setup.bash
ros2 launch range_image_segmentation range_image_segmentation.launch.py
```

单节点调试：

```bash
ros2 run range_image_segmentation range_image_segmentation_node \
  --ros-args -p lidar_name:=front -p input_topic:=/front/rslidar_points_leveled
```
