/**
 * @file range_image_segmentation.cpp
 * @brief 统一沟壑(负障碍) + 悬崖边缘检测
 *
 * 将调平点云按 (水平角, 俯仰角) 投影成一张 1° 网格图像(默认 120x45)，
 * 在"距离连续"这一维上找断点：
 *   Rule 1: 单个格子内，多点按距离排序，相邻两点距离差 > gap_thresh 的近侧点 -> 沟壑边缘
 *   Rule 2: 水平相邻两格子，合并排序后找距离断点 -> 沟壑边缘(侧壁/横向边界)
 *   Rule 3: 垂直相邻两格子，合并排序后找距离断点 -> 沟壑边缘(近沿)
 *   Rule 4: 某列无沟壑时，取该列最上方有回波格子里最近的点 -> 悬崖边缘
 *
 * 输入 : 调平点云 PointXYZI (lidar_leveling 输出)
 * 输出 : /<lidar_name>/ditch_cloud (PointCloud2, intensity: 255=沟壑, 60=悬崖)
 *        /<lidar_name>/ditch_line  (MarkerArray, LINE_STRIP)
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <dth_messages/msg/edge_warning.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <perception_common/motion_state.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace
{
constexpr double kPi = 3.14159265358979323846;
}  // namespace

class RangeImageSegmentation : public rclcpp::Node
{
public:
  RangeImageSegmentation()
  : Node("range_image_segmentation")
  {
    lidar_name_ = declare_parameter<std::string>("lidar_name", "");
    const std::string prefix = lidar_name_.empty() ? "" : ("/" + lidar_name_);

    input_topic_ =
      declare_parameter<std::string>("input_topic", prefix + "/rslidar_points_leveled");
    const std::string edge_cloud_topic =
      declare_parameter<std::string>("edge_cloud_topic", prefix + "/ditch_cloud");
    const std::string edge_line_topic =
      declare_parameter<std::string>("edge_line_topic", prefix + "/ditch_line");
    frame_id_ = declare_parameter<std::string>(
      "frame_id", lidar_name_.empty() ? "rslidar" : lidar_name_ + "_rslidar");

    max_range_ = declare_parameter<double>("max_range", 12.0);
    min_range_ = declare_parameter<double>("min_range", 0.2);
    gap_thresh_ = declare_parameter<double>("gap_thresh", 1.0);
    edge_max_range_ = declare_parameter<double>("edge_max_range", 10.0);
    outlier_mean_k_ = declare_parameter<int>("outlier_mean_k", 6);
    outlier_std_mul_ = declare_parameter<double>("outlier_std_mul", 1.0);

    // 边坡告警: 本实例只发布「单雷达原始告警」到节点私有话题，由 perception_common 包的
    // warning_fusion 合并后发布契约话题 /perception/edge_warning 与 /perception/stop_command。
    edge_topic_ =
      declare_parameter<std::string>("edge_warning_raw_topic", "~/edge_warning_raw");
    // 边坡分档阈值：唯一真源 = perception_common/config/params.yaml 的 /** 共享段
    // （与 pothole_detection 一致），声明为「无默认值」——未提供则启动即失败。
    this->declare_parameter("edge_danger_dist", rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("edge_caution_dist", rclcpp::ParameterType::PARAMETER_DOUBLE);
    edge_heartbeat_hz_ = declare_parameter<double>("edge_heartbeat_hz", 10.0);
    edge_timeout_s_ = declare_parameter<double>("edge_timeout_s", 0.5);
    this->get_parameter("edge_danger_dist", edge_danger_dist_);
    this->get_parameter("edge_caution_dist", edge_caution_dist_);
    if (!(edge_danger_dist_ > 0.0 && edge_caution_dist_ > edge_danger_dist_)) {
      RCLCPP_FATAL(get_logger(),
                   "边坡分档阈值非法（edge_danger_dist=%.1f, edge_caution_dist=%.1f）："
                   "须 0 < danger < caution，见 config/params.yaml 的 /** 共享段",
                   edge_danger_dist_, edge_caution_dist_);
      throw std::runtime_error("invalid edge thresholds");
    }
    if (!(edge_heartbeat_hz_ > 0.0)) edge_heartbeat_hz_ = 10.0;
    edge_heartbeat_period_s_ = 1.0 / edge_heartbeat_hz_;

    // 定向检测门控（契约 §3.1/§6.2）：订阅 gate 镜像自判运动方向，决定本雷达是否参与检测
    motion_gate_enable_ = declare_parameter<bool>("motion_gate_enable", true);
    gate_ = std::make_unique<perception_common::MotionGate>(*this, lidar_name_);

    const double horizontal_angle_min =
      declare_parameter<double>("horizontal_angle_min", -60.0);
    const double horizontal_angle_max =
      declare_parameter<double>("horizontal_angle_max", 60.0);
    const double horizontal_res = declare_parameter<double>("horizontal_res", 1.0);
    const double vertical_angle_min =
      declare_parameter<double>("vertical_angle_min", -45.0);
    const double vertical_angle_max =
      declare_parameter<double>("vertical_angle_max", 0.0);
    const double vertical_res = declare_parameter<double>("vertical_res", 1.0);

    cols_ = std::max(1, static_cast<int>(
      std::lround((horizontal_angle_max - horizontal_angle_min) / horizontal_res)));
    rows_ = std::max(1, static_cast<int>(
      std::lround((vertical_angle_max - vertical_angle_min) / vertical_res)));
    horiz_min_ = horizontal_angle_min;
    horiz_max_ = horizontal_angle_max;
    vert_min_ = vertical_angle_min;
    vert_max_ = vertical_angle_max;

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&RangeImageSegmentation::onCloud, this, std::placeholders::_1));
    pub_edge_cloud_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(edge_cloud_topic, rclcpp::QoS(10));
    pub_edge_line_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(edge_line_topic, rclcpp::QoS(10));
    pub_edge_ =
      create_publisher<dth_messages::msg::EdgeWarning>(edge_topic_, rclcpp::QoS(10));

    // 心跳：与点云解耦周期发布（契约 §4.3 必须 ≥5Hz）；点云断流时输出安全默认值（§3.3）。
    edge_heartbeat_timer_ = create_wall_timer(
      std::chrono::duration<double>(edge_heartbeat_period_s_),
      std::bind(&RangeImageSegmentation::onEdgeHeartbeat, this));

    RCLCPP_INFO(
      get_logger(),
      "range_image_segmentation started (lidar_name=%s): input=%s, image=%dx%d, "
      "horizontal=[%.1f,%.1f]deg, vertical=[%.1f,%.1f]deg, "
      "max_range=%.1fm, gap_thresh=%.2fm, edge_warning=%s [danger<%.1fm, caution<%.1fm, "
      "heartbeat=%.1fHz, timeout=%.2fs]",
      lidar_name_.c_str(), input_topic_.c_str(), cols_, rows_, horiz_min_, horiz_max_,
      vert_min_, vert_max_, max_range_, gap_thresh_, edge_topic_.c_str(),
      edge_danger_dist_, edge_caution_dist_, edge_heartbeat_hz_, edge_timeout_s_);
    RCLCPP_INFO(get_logger(), "定向检测门控: %s（%s）",
                motion_gate_enable_ ? "on" : "off", gate_->describe().c_str());
  }

private:
  // 每个格子存 (距离, 全局点 id)
  using Cell = std::vector<std::pair<float, int>>;

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // ---- 定向检测门控（契约 §3.1/§6.2）----
    // 前进只测前方、后退只测后方；不参与检测的一侧整帧跳过点云处理（省算力），
    // 但仍按帧发布安全告警，保证原始告警话题不断流、融合侧能判断本实例在线。
    const auto motion = gate_->state();
    const bool detect = !motion_gate_enable_ || gate_->active();

    if (motion != last_motion_) {
      RCLCPP_INFO(get_logger(), "%s 运动方向 %s → %s（%s）",
                  perception_common::toString(gate_->side()),
                  perception_common::toString(last_motion_),
                  perception_common::toString(motion),
                  detect ? "参与检测" : "门控跳过");
      last_motion_ = motion;
    }

    if (!detect) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                           "%s 门控跳过检测（运动方向 %s），发布安全默认值",
                           perception_common::toString(gate_->side()),
                           perception_common::toString(motion));
      publishSafeEdge();
      return;
    }

    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*msg, cloud);

    const int cell_count = cols_ * rows_;
    std::vector<Cell> cells(cell_count);
    std::vector<pcl::PointXYZI> points;   // 全局点表, 下标即 id
    std::vector<int> point_cell;          // 每个点所属格子 index
    points.reserve(cloud.size());
    point_cell.reserve(cloud.size());

    // 投影: (方位角, 俯仰角) -> (col, row)
    for (const auto & p : cloud.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }
      const float range = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
      if (range < min_range_ || range > max_range_) {
        continue;
      }

      const double azimuth = std::atan2(p.y, p.x) * 180.0 / kPi;
      const double elevation = std::asin(p.z / range) * 180.0 / kPi;
      if (azimuth < horiz_min_ || azimuth > horiz_max_) {
        continue;
      }
      if (elevation < vert_min_ || elevation > vert_max_) {
        continue;
      }

      // 推进梁过滤：雷达前方近处、接近水平、且高于雷达的结构件
      // if (elevation > -10.0 && range < 3.0 && p.z > -0.5f) {
      //   continue;
      // }
      if (range < 3.0 && p.z > -1.0f) {
        continue;
      }      

      // row: 上=vert_max(0°), 下=vert_min(-45°)
      int col = static_cast<int>((azimuth - horiz_min_) / (horiz_max_ - horiz_min_) * cols_);
      int row = static_cast<int>((vert_max_ - elevation) / (vert_max_ - vert_min_) * rows_);
      col = std::clamp(col, 0, cols_ - 1);
      row = std::clamp(row, 0, rows_ - 1);

      const int id = static_cast<int>(points.size());
      points.push_back(p);
      point_cell.push_back(row * cols_ + col);
      cells[row * cols_ + col].emplace_back(range, id);
    }

    std::vector<uint8_t> flagged(points.size(), 0);
    std::vector<uint8_t> is_cliff(points.size(), 0);
    std::vector<uint8_t> col_has_ditch(cols_, 0);

    // 在已按距离升序的序列里找相邻断点, 近侧点标记为沟壑边缘
    auto detectGaps = [&](const Cell & sorted) {
      for (size_t i = 0; i + 1 < sorted.size(); ++i) {
        if (sorted[i + 1].first - sorted[i].first > gap_thresh_) {
          const int id = sorted[i].second;
          flagged[id] = 1;
          col_has_ditch[point_cell[id] % cols_] = 1;
        }
      }
    };

    // Rule 1: 单格内
    for (auto & cell : cells) {
      if (cell.size() < 2) {
        continue;
      }
      std::sort(cell.begin(), cell.end());
      detectGaps(cell);
    }

    // 两格合并排序找断点
    auto checkPair = [&](const Cell & a, const Cell & b) {
      if (a.size() + b.size() < 2) {
        return;
      }
      Cell merged;
      merged.reserve(a.size() + b.size());
      merged.insert(merged.end(), a.begin(), a.end());
      merged.insert(merged.end(), b.begin(), b.end());
      std::sort(merged.begin(), merged.end());
      detectGaps(merged);
    };

    // Rule 2: 水平相邻
    for (int r = 0; r < rows_; ++r) {
      for (int c = 0; c + 1 < cols_; ++c) {
        checkPair(cells[r * cols_ + c], cells[r * cols_ + c + 1]);
      }
    }

    // Rule 3: 垂直相邻
    for (int c = 0; c < cols_; ++c) {
      for (int r = 0; r + 1 < rows_; ++r) {
        checkPair(cells[r * cols_ + c], cells[(r + 1) * cols_ + c]);
      }
    }

    // Rule 4: 无沟壑的列, 取最上方有回波格子里最近的点作为悬崖
    for (int c = 0; c < cols_; ++c) {
      if (col_has_ditch[c]) {
        continue;
      }
      int topmost = -1;
      for (int r = 0; r < rows_; ++r) {
        if (!cells[r * cols_ + c].empty()) {
          topmost = r;
          break;
        }
      }
      if (topmost < 0) {
        continue;
      }
      const Cell & cell = cells[topmost * cols_ + c];  // Rule 1 后已按距离升序
      const int id = cell.front().second;
      flagged[id] = 1;
      is_cliff[id] = 1;
    }

    publish(msg->header, points, flagged, is_cliff);
  }

  void publish(
    const std_msgs::msg::Header & header,
    const std::vector<pcl::PointXYZI> & points,
    const std::vector<uint8_t> & flagged,
    const std::vector<uint8_t> & is_cliff)
  {
    pcl::PointCloud<pcl::PointXYZI> edge;
    for (size_t id = 0; id < points.size(); ++id) {
      if (!flagged[id]) {
        continue;
      }
      pcl::PointXYZI p = points[id];
      const float range = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
      if (range > edge_max_range_) {
        continue;
      }
      p.intensity = is_cliff[id] ? 60.0f : 255.0f;  // 悬崖暗, 沟壑亮(仅调试用)
      edge.push_back(p);
    }

    // 统计离群点移除：剔除 k 近邻平均距离明显偏大的孤立点
    pcl::PointCloud<pcl::PointXYZI> edge_filtered;
    if (edge.size() > static_cast<size_t>(outlier_mean_k_)) {
      pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
      sor.setInputCloud(edge.makeShared());
      sor.setMeanK(outlier_mean_k_);
      sor.setStddevMulThresh(outlier_std_mul_);
      sor.filter(edge_filtered);
    } else {
      edge_filtered = edge;
    }

    // 边缘点云
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(edge_filtered, cloud_msg);
    cloud_msg.header = header;
    pub_edge_cloud_->publish(cloud_msg);

    // 按方位角排序（用于线段）
    std::vector<std::pair<double, int>> order;  // (方位角, 在 edge_filtered 中的下标)
    order.reserve(edge_filtered.size());
    for (int i = 0; i < static_cast<int>(edge_filtered.size()); ++i) {
      const auto & p = edge_filtered[i];
      order.emplace_back(std::atan2(p.y, p.x), i);
    }

    // 边缘线段
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear;
    clear.header = header;
    clear.ns = "ditch";
    clear.id = 0;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);

    if (!edge_filtered.empty()) {
      std::sort(order.begin(), order.end());

      // 调试：每 10 个点打印一次 z / 垂直角 / range
      // RCLCPP_INFO(get_logger(), "----------------------------------------");
      // for (size_t i = 0; i < order.size(); i += 3) {
      //   const auto & p = edge_filtered[order[i].second];
      //   const float range = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
      //   const double elevation = std::asin(p.z / range) * 180.0 / kPi;
      //   RCLCPP_INFO(get_logger(), "[edge] z=%.3f elev=%.2fdeg range=%.3fm",
      //               p.z, elevation, range);
      // }

      // 每 3 个点取平均坐标作为代表，平滑
      std::vector<geometry_msgs::msg::Point> sampled;
      sampled.reserve(order.size() / 3 + 1);
      for (size_t i = 0; i < order.size(); i += 3) {
        const size_t end = std::min(order.size(), i + 3);
        double sx = 0.0, sy = 0.0, sz = 0.0;
        for (size_t j = i; j < end; ++j) {
          const auto & q = edge_filtered[order[j].second];
          sx += q.x;
          sy += q.y;
          sz += q.z;
        }
        const double n = static_cast<double>(end - i);
        geometry_msgs::msg::Point pt;
        pt.x = static_cast<float>(sx / n);
        pt.y = static_cast<float>(sy / n);
        pt.z = static_cast<float>(sz / n);
        sampled.push_back(pt);
      }

      visualization_msgs::msg::Marker line;
      line.header = header;
      line.ns = "ditch";
      line.id = 0;
      line.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line.action = visualization_msgs::msg::Marker::ADD;
      line.scale.x = 0.1;
      line.color.r = 1.0f;
      line.color.g = 0.0f;
      line.color.b = 0.0f;
      line.color.a = 1.0f;
      line.pose.orientation.w = 1.0;
      line.points.reserve(sampled.size());
      for (const auto & pt : sampled) {
        line.points.push_back(pt);
      }
      array.markers.push_back(line);
    }

    pub_edge_line_->publish(array);

    // ---- 边坡告警（原始告警，由 warning_fusion 合并发布契约话题）----
    publishEdgeWarning(edge_filtered);
  }

  // 根据本帧边缘点云计算并发布原始边坡告警（由 warning_fusion 合并为契约话题）
  void publishEdgeWarning(const pcl::PointCloud<pcl::PointXYZI> & edge)
  {
    dth_messages::msg::EdgeWarning msg;
    if (!edge.empty()) {
      double min_dist = std::numeric_limits<double>::max();
      for (const auto & p : edge.points) {
        min_dist = std::min(min_dist, static_cast<double>(std::hypot(p.x, p.y)));
      }
      msg.dist_to_edge_m = static_cast<float>(min_dist);
      if (min_dist < edge_danger_dist_) {
        msg.warning = true;
        msg.level = dth_messages::msg::EdgeWarning::LEVEL_DANGER;
      } else if (min_dist <= edge_caution_dist_) {
        msg.warning = true;
        msg.level = dth_messages::msg::EdgeWarning::LEVEL_CAUTION;
      } else {
        msg.warning = false;
        msg.level = dth_messages::msg::EdgeWarning::LEVEL_SAFE;
      }
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                           "%s 检测到边坡: 最近距离=%.2f m（边缘 %zu pts）→ %s "
                           "（报警阈值 %.1f m / 停车阈值 %.1f m）",
                           perception_common::toString(gate_->side()),
                           min_dist, edge.size(), levelName(msg.level),
                           edge_caution_dist_, edge_danger_dist_);
    } else {
      msg.dist_to_edge_m = 999.0f;
      msg.warning = false;
      msg.level = dth_messages::msg::EdgeWarning::LEVEL_SAFE;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                           "%s 没有检测到边坡 → SAFE（报警阈值 %.1f m / 停车阈值 %.1f m）",
                           perception_common::toString(gate_->side()),
                           edge_caution_dist_, edge_danger_dist_);
    }

    std::lock_guard<std::mutex> lock(edge_mutex_);
    cloud_seen_ = true;
    last_cloud_time_ = this->now();
    publishEdgeLocked(msg);
    last_edge_ = msg;
  }

  // 发布安全告警（999.0 / SAFE）并刷新心跳时间戳：
  // 门控跳过检测的一侧即由此保持「在线 + 安全」语义（契约 §4.3）
  void publishSafeEdge()
  {
    dth_messages::msg::EdgeWarning msg;
    msg.warning = false;
    msg.dist_to_edge_m = 999.0f;
    msg.level = dth_messages::msg::EdgeWarning::LEVEL_SAFE;

    std::lock_guard<std::mutex> lock(edge_mutex_);
    cloud_seen_ = true;
    last_cloud_time_ = this->now();
    publishEdgeLocked(msg);
    last_edge_ = msg;
  }

  // 填充 header 并发布（调用方需持有 edge_mutex_）
  void publishEdgeLocked(dth_messages::msg::EdgeWarning & msg)
  {
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;
    pub_edge_->publish(msg);
    last_edge_publish_time_ = msg.header.stamp;
  }

  // 心跳定时器：与点云处理解耦，保证原始告警周期发布（契约 §4.3 ≥5Hz）；
  // 点云断流超过 edge_timeout_s 时切换为安全默认值（契约 §3.3）。
  void onEdgeHeartbeat()
  {
    std::lock_guard<std::mutex> lock(edge_mutex_);
    const rclcpp::Time now = this->now();
    const bool sensor_ok =
        cloud_seen_ && (now - last_cloud_time_).seconds() <= edge_timeout_s_;

    if (!sensor_ok) {
      if (cloud_seen_ && !stale_warned_) {
        RCLCPP_WARN(get_logger(),
                    "%s 点云 %s 断流超过 %.2f s，原始边坡告警转安全默认值（999.0/SAFE）",
                    perception_common::toString(gate_->side()),
                    input_topic_.c_str(), edge_timeout_s_);
        stale_warned_ = true;
      }
      dth_messages::msg::EdgeWarning safe;
      safe.warning = false;
      safe.dist_to_edge_m = 999.0f;
      safe.level = dth_messages::msg::EdgeWarning::LEVEL_SAFE;
      publishEdgeLocked(safe);
      return;
    }

    if (stale_warned_) {
      RCLCPP_INFO(get_logger(), "%s 点云 %s 恢复，原始边坡告警恢复正常检测输出",
                  perception_common::toString(gate_->side()), input_topic_.c_str());
      stale_warned_ = false;
    }

    if ((now - last_edge_publish_time_).seconds() >= edge_heartbeat_period_s_ * 1.5) {
      dth_messages::msg::EdgeWarning msg = last_edge_;
      publishEdgeLocked(msg);
    }
  }

  static const char * levelName(uint8_t level)
  {
    switch (level) {
      case dth_messages::msg::EdgeWarning::LEVEL_DANGER:  return "DANGER";
      case dth_messages::msg::EdgeWarning::LEVEL_CAUTION: return "CAUTION";
      default: return "SAFE";
    }
  }

  // ---- 参数 ---- //
  double max_range_ = 12.0;
  double min_range_ = 0.2;
  double gap_thresh_ = 1.0;
  double edge_max_range_ = 10.0;
  int outlier_mean_k_ = 6;
  double outlier_std_mul_ = 1.0;
  double horiz_min_ = -60.0;
  double horiz_max_ = 60.0;
  double vert_min_ = -45.0;
  double vert_max_ = 0.0;
  int cols_ = 120;
  int rows_ = 45;

  std::string lidar_name_;
  std::string input_topic_;
  std::string frame_id_;
  std::string edge_topic_;
  double edge_danger_dist_ = 5.0;
  double edge_caution_dist_ = 10.0;
  double edge_heartbeat_hz_ = 10.0;
  double edge_heartbeat_period_s_ = 0.1;
  double edge_timeout_s_ = 0.5;

  // 定向检测门控（契约 §3.1/§6.2）：共用 perception_common/motion_state.hpp
  std::unique_ptr<perception_common::MotionGate> gate_;
  bool motion_gate_enable_ = true;
  perception_common::MotionState last_motion_{perception_common::MotionState::kStopped};

  // ---- 订阅 / 发布 ---- //
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_edge_cloud_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_edge_line_;
  rclcpp::Publisher<dth_messages::msg::EdgeWarning>::SharedPtr pub_edge_;
  rclcpp::TimerBase::SharedPtr edge_heartbeat_timer_;

  // ---- 边坡告警心跳状态（onCloud 与心跳定时器共享，加锁保护）---- //
  std::mutex edge_mutex_;
  dth_messages::msg::EdgeWarning last_edge_;
  rclcpp::Time last_cloud_time_;
  rclcpp::Time last_edge_publish_time_;
  bool cloud_seen_ = false;
  bool stale_warned_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RangeImageSegmentation>());
  rclcpp::shutdown();
  return 0;
}
