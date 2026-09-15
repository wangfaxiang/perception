# dth_messages（镜像副本）

AutoDTH 感知接口契约消息。**本包不是真源**，仅为「本项目（感知侧）与系统侧独立开发」提供编译依赖。

- 真源：AutoDTH 系统侧仓库 `src/messages/dth_messages/`
- 上游文档：`dth_perception 感知模块 ROS 2 接口对接文档` v1.1 (2026-09-13)
- 同步版本：v1.1（含 `ObstacleWarning` / `EdgeWarning` / `StopCommand` 三型）

## 铁律

1. `.msg` 只保留字段与常量行，无注释、无排版改动 —— 与文档附录 A 逐字一致。
2. **改包名 / 改消息名 / 增删字段或常量 = 断开联调**。ROS 2 的接口类型标识是
   `dth_messages/msg/<Name>`，任何一侧改名或改定义，DDS 层即对不上。
3. 本项目自有的内部消息类型（调试、可视化、中间结果）**不要**加进本包，
   另建 `perception_msgs`。契约包与自有包必须分离。

## 相关约定（文档 §9 冻结项）

- 三话题名与 QoS：`/perception/stop_command`、`/perception/edge_warning`、`/perception/obstacle_warning`
  —— 默认 QoS（RELIABLE + VOLATILE + KEEP_LAST(10)），≥5 Hz（建议 10）。
- `999.0` = 无数据哨兵值；`obstacle_type=0` = 无（枚举未定义 0 常量，属既定约定，勿补常量）。
- 调试类话题必须挂节点私有名空间 `~/`（如 `~/debug/point_cloud`），不得占用 `/perception/*`。
