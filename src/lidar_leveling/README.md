# lidar_leveling

根据前/后雷达安装欧拉角，将点云旋转回水平，输出调平后的点云话题，供其他模块使用。

## 功能

- 订阅原始点云话题（如 `/front/rslidar_points`、`/rear/rslidar_points`）
- 按安装欧拉角 `roll/pitch/yaw` 构造旋转矩阵，将每个点变换到水平坐标系
- 发布调平后的点云话题（如 `/front/rslidar_points_leveled`、`/rear/rslidar_points_leveled`）

## 旋转约定

```
p_level = Rz(yaw) * Ry(pitch) * Rx(roll) * p_lidar
```

- `roll` 绕 X 轴（前）
- `pitch` 绕 Y 轴（左）
- `yaw` 绕 Z 轴（上）
- 均遵循右手定则，参数单位为度

调平示例：

- 雷达向下俯仰安装 θ：`pitch_deg = +θ`
- 雷达向左横滚安装 θ：`roll_deg = -θ`

## 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `lidar_name` | `""` | 雷达标识（`front` / `rear`），用于推导默认话题名 |
| `input_topic` | `/<name>/rslidar_points` | 输入点云话题 |
| `output_topic` | `/<name>/rslidar_points_leveled` | 输出点云话题 |
| `frame_id` | `""` | 输出 frame_id，为空沿用输入 frame_id |
| `roll_deg` | `0.0` | 横滚角（度） |
| `pitch_deg` | `0.0` | 俯仰角（度） |
| `yaw_deg` | `0.0` | 偏航角（度） |

## 构建与运行

```bash
colcon build --packages-select lidar_leveling
source install/setup.bash
ros2 launch lidar_leveling lidar_leveling.launch.py
```

单实例运行示例：

```bash
ros2 run lidar_leveling lidar_leveling_node --ros-args \
  -p input_topic:=/front/rslidar_points \
  -p output_topic:=/front/rslidar_points_leveled \
  -p pitch_deg:=5.0
```
