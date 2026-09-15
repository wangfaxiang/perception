/**
 * @file pothole_detection_node.cpp
 * @brief 基于相邻线束地面落点水平间距比检测沟壑（负障碍物）与悬崖
 *
 * 适用: 速腾聚创 E1R 固态雷达（120°×90°，1200×144），水平安装在车体上。
 *
 * 沟壑原理:
 *   雷达安装高度为 h、光轴水平时，俯角为 β 的线束打到水平地面的水平距离为
 *       x(β) = h · cot(β)
 *   同一水平角（同一列）方向上相邻两条线束（近→远，俯角 β_i > β_{i+1}）的理论落点间距为
 *       Δx_th = h · (cot β_{i+1} − cot β_i)
 *
 *   若两条线束之间横跨一条沟（沟底被近沿遮挡、雷达扫不到），远处线束将直接落到沟的
 *   远沿或沟后地面，实测间距 Δx_meas 明显大于理论值，据此判定为沟:
 *       ratio = Δx_meas / Δx_th > 阈值
 *
 *   沟的近沿点即为危险边界，按横向连续列数聚类后发布为 PointCloud2，
 *   intensity 编码归一化的间距比。
 *
 * 悬崖原理:
 *   悬崖下方无回波，某一列在近距离处就停止出现回波（最远点明显近于探测范围）。
 *   若该列没有检测到沟壑边缘，则把该列最远点作为悬崖边缘候选。
 *
 * 支持多雷达: 通过 lidar_name 参数（front/rear）区分前/后雷达，同一节点可启动多个实例。
 *
 * 定向检测（契约 §3.1/§6.2）: 按车辆运动方向门控——前进只测前方、后退只测后方、
 * 停车/原地旋转前后均检测（四态判定见 motion_state.hpp）。本实例只发布「单雷达原始告警」
 * 到节点私有话题（~/edge_warning_raw），由 edge_warning_fusion 取高等级合并为契约话题
 * /perception/edge_warning。
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <dth_messages/msg/edge_warning.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

#include "motion_state.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace
{
constexpr double kDeg2Rad = M_PI / 180.0;

double horizontalDistance(const pcl::PointXYZI &p) { return std::hypot(p.x, p.y); }

double verticalAngle(const pcl::PointXYZI &p)
{
  const double r = std::hypot(p.x, p.y, p.z);
  return std::asin(std::clamp(p.z / r, -1.0, 1.0));
}
}  // namespace

class PotholeDetection : public rclcpp::Node
{
public:
  PotholeDetection() : Node("pothole_detection")
  {
    // 雷达标识（front / rear），用于推导默认话题名、frame_id 与定向检测归属
    std::string lidar_name;
    this->declare_parameter<std::string>("lidar_name", "");
    this->get_parameter("lidar_name", lidar_name);
    lidar_name_ = lidar_name;

    const std::string prefix = lidar_name.empty() ? "" : ("/" + lidar_name);
    this->declare_parameter<std::string>("input_topic", prefix + "/rslidar_points_leveled");
    this->declare_parameter<std::string>("ditch_cloud_topic", prefix + "/ditch_cloud");
    this->declare_parameter<std::string>("ditch_line_topic", prefix + "/ditch_line");
    this->declare_parameter<std::string>("voxel_cloud_topic", prefix + "/voxel_cloud");
    this->declare_parameter<std::string>("frame_id",
                                         lidar_name.empty() ? "rslidar" : (lidar_name + "_rslidar"));

    this->declare_parameter("mount_height", 1.2);         // 雷达安装高度（米）
    this->declare_parameter("horiz_fov_deg", 120.0);      // 水平视场角（度）
    this->declare_parameter("horiz_res_deg", 0.1);        // 列分辨率（须与雷达原生水平分辨率一致）
    this->declare_parameter("min_vert_angle_deg", -45.0); // 参与检测的最小垂直角（最近处）
    this->declare_parameter("max_vert_angle_deg", -1.0);  // 参与检测的最大垂直角（最远处，须 < 0）
    this->declare_parameter("max_range", 30.0);           // 参与检测的最大水平距离（米）
    this->declare_parameter("min_range", 0.5);            // 参与检测的最小水平距离（米）
    this->declare_parameter("ditch_ratio_thresh", 7.0);   // 间距比阈值
    this->declare_parameter("ditch_min_width", 0.5);      // 最小沟宽（米）
    this->declare_parameter("min_gap_m", 0.02);           // 最小理论线束间距（米）
    this->declare_parameter("cluster_dist", 1.5);         // 沟沿点聚类距离（米）
    this->declare_parameter("min_points_per_ditch", 5);   // 每段沟最少点数（≈横向连续列数）

    this->declare_parameter("voxel_leaf_size", 0.05);       // 体素降采样叶子尺寸（米），<=0 关闭
    this->declare_parameter("cliff_enable", false);          // 是否启用悬崖检测
    this->declare_parameter("cliff_max_range", 7.0);        // 悬崖边缘最大水平距离（米）
    this->declare_parameter("cliff_min_points", 3);         // 列内最少点数才判悬崖
    this->declare_parameter("cliff_vert_margin_deg", 1.0);  // 最远点距 max_vert_angle 的最小角裕量（度）
    this->declare_parameter<std::string>("cliff_cloud_topic", prefix + "/cliff_cloud");

    // 边坡告警: 本实例只发布「单雷达原始告警」到节点私有话题（契约 §9 命名要求），
    // 由 edge_warning_fusion 合并后发布契约话题 /perception/edge_warning
    this->declare_parameter<std::string>("edge_warning_raw_topic", "~/edge_warning_raw");
    this->declare_parameter("edge_danger_dist", 6.0);   // 最近距离 < 该值 → DANGER
    this->declare_parameter("edge_caution_dist", 10.0); // 最近距离 < 该值 → CAUTION，否则 SAFE
    this->declare_parameter("edge_heartbeat_hz", 10.0); // 心跳频率（契约 §4.3 必须 ≥5Hz，建议 10Hz）
    this->declare_parameter("edge_timeout_s", 0.5);     // 点云断流判定超时（秒）→ 转安全默认值（§3.3）

    this->get_parameter("input_topic", input_topic_);
    this->get_parameter("ditch_cloud_topic", ditch_topic_);
    this->get_parameter("ditch_line_topic", line_topic_);
    this->get_parameter("voxel_cloud_topic", voxel_topic_);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("mount_height", mount_height_);
    this->get_parameter("horiz_fov_deg", horiz_fov_deg_);
    this->get_parameter("horiz_res_deg", horiz_res_deg_);
    this->get_parameter("min_vert_angle_deg", min_vert_deg_);
    this->get_parameter("max_vert_angle_deg", max_vert_deg_);
    this->get_parameter("max_range", max_range_);
    this->get_parameter("min_range", min_range_);
    this->get_parameter("ditch_ratio_thresh", ratio_thresh_);
    this->get_parameter("ditch_min_width", min_width_);
    this->get_parameter("min_gap_m", min_gap_);
    this->get_parameter("cluster_dist", cluster_dist_);
    this->get_parameter("min_points_per_ditch", min_pts_);
    this->get_parameter("voxel_leaf_size", voxel_leaf_size_);
    this->get_parameter("cliff_enable", cliff_enable_);
    this->get_parameter("cliff_max_range", cliff_max_range_);
    this->get_parameter("cliff_min_points", cliff_min_pts_);
    this->get_parameter("cliff_vert_margin_deg", cliff_vert_margin_deg_);
    this->get_parameter("cliff_cloud_topic", cliff_topic_);
    this->get_parameter("edge_warning_raw_topic", edge_topic_);
    this->get_parameter("edge_danger_dist", edge_danger_dist_);
    this->get_parameter("edge_caution_dist", edge_caution_dist_);
    this->get_parameter("edge_heartbeat_hz", edge_heartbeat_hz_);
    this->get_parameter("edge_timeout_s", edge_timeout_s_);

    // 心跳周期（频率非法时回退 10 Hz）
    if (!(edge_heartbeat_hz_ > 0.0)) edge_heartbeat_hz_ = 10.0;
    edge_heartbeat_period_s_ = 1.0 / edge_heartbeat_hz_;

    // 定向检测门控（契约 §3.1/§6.2）：订阅 gate 镜像自判运动方向，决定本雷达是否参与检测
    this->declare_parameter("motion_gate_enable", true);
    this->get_parameter("motion_gate_enable", motion_gate_enable_);
    gate_ = std::make_unique<pothole_detection::MotionGate>(*this, lidar_name_);

    cols_ = static_cast<int>(horiz_fov_deg_ / horiz_res_deg_);
    horiz_min_rad_ = -horiz_fov_deg_ / 2.0 * kDeg2Rad;
    horiz_res_rad_ = horiz_res_deg_ * kDeg2Rad;
    min_vert_rad_ = min_vert_deg_ * kDeg2Rad;
    max_vert_rad_ = max_vert_deg_ * kDeg2Rad;
    cliff_vert_margin_rad_ = cliff_vert_margin_deg_ * kDeg2Rad;

    sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&PotholeDetection::onCloud, this, std::placeholders::_1));
    pub_ditch_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(ditch_topic_, 10);
    pub_cliff_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cliff_topic_, 10);
    pub_voxel_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(voxel_topic_, 10);
    pub_line_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(line_topic_, 10);
    pub_edge_ = this->create_publisher<dth_messages::msg::EdgeWarning>(edge_topic_, 10);

    // 心跳：与点云解耦周期发布（契约 §4.3 必须 ≥5Hz）。点云断流时输出安全默认值（§3.3），
    // 保证感知静默死亡/雷达掉线时融合侧仍能通过原始告警话题判断本实例在线。
    edge_heartbeat_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(edge_heartbeat_period_s_),
        std::bind(&PotholeDetection::onEdgeHeartbeat, this));

    RCLCPP_INFO(this->get_logger(),
                "Ditch/cliff detection started (lidar=%s, height=%.2fm, ratio>%.1f, "
                "width>%.2fm, cliff=%s, cliff_range<%.1fm, voxel=%.2fm, "
                "edge_warning=%s [danger<%.1fm, caution<%.1fm, heartbeat=%.1fHz, timeout=%.2fs])",
                lidar_name.empty() ? "single" : lidar_name.c_str(),
                mount_height_, ratio_thresh_, min_width_,
                cliff_enable_ ? "on" : "off", cliff_max_range_, voxel_leaf_size_,
                edge_topic_.c_str(), edge_danger_dist_, edge_caution_dist_,
                edge_heartbeat_hz_, edge_timeout_s_);

    RCLCPP_INFO(this->get_logger(), "定向检测门控: %s（%s）",
                motion_gate_enable_ ? "on" : "off", gate_->describe().c_str());
  }

private:
  struct Candidate
  {
    pcl::PointXYZI point;
    float ratio;    // 沟壑: 间距比; 悬崖: <0
    bool is_cliff;  // true: 悬崖候选（下方无回波）
  };

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // ---- 定向检测门控（契约 §3.1/§6.2）----
    // 前进只测前方、后退只测后方；不参与检测的一侧整帧跳过点云处理（省算力），
    // 但仍按帧发布安全告警，保证原始告警话题不断流、融合侧能判断本实例在线。
    const auto motion = gate_->state();
    const bool detect = !motion_gate_enable_ || gate_->active();

    if (motion != last_motion_)
    {
      RCLCPP_INFO(this->get_logger(), "%s 运动方向 %s → %s（%s）",
                  pothole_detection::toString(gate_->side()),
                  pothole_detection::toString(last_motion_),
                  pothole_detection::toString(motion),
                  detect ? "参与检测" : "门控跳过");
      last_motion_ = motion;
    }

    if (!detect)
    {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "%s 门控跳过检测（运动方向 %s），发布安全默认值",
                           pothole_detection::toString(gate_->side()),
                           pothole_detection::toString(motion));
      publishSafeEdge();
      return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *cloud);

    std::vector<int> nan_indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, nan_indices);

    // 体素化降采样: 降低点云密度、抑制噪声（叶子尺寸 <=0 则跳过）
    if (voxel_leaf_size_ > 0.0)
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr voxel(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::VoxelGrid<pcl::PointXYZI> vg;
      vg.setInputCloud(cloud);
      vg.setLeafSize(static_cast<float>(voxel_leaf_size_),
                     static_cast<float>(voxel_leaf_size_),
                     static_cast<float>(voxel_leaf_size_));
      vg.filter(*voxel);
      cloud = voxel;

      // 发布体素化降采样后的点云
      sensor_msgs::msg::PointCloud2 out;
      pcl::toROSMsg(*cloud, out);
      out.header.stamp = msg->header.stamp;
      out.header.frame_id = frame_id_;
      pub_voxel_->publish(out);
    }

    // 按列（水平角）分桶，每列保存该方向上的地面点
    std::vector<std::vector<pcl::PointXYZI>> cols(cols_);
    for (const auto &pt : cloud->points)
    {
      const double d = horizontalDistance(pt);
      if (d < min_range_ || d > max_range_) continue;

      const double va = verticalAngle(pt);
      if (va < min_vert_rad_ || va > max_vert_rad_) continue;

      const double ha = std::atan2(pt.y, pt.x);
      if (ha < horiz_min_rad_ || ha > -horiz_min_rad_) continue;

      const int col = static_cast<int>((ha - horiz_min_rad_) / horiz_res_rad_);
      if (col < 0 || col >= cols_) continue;
      cols[col].push_back(pt);
    }

    // 逐列检测: 相邻线束落点间距比异常 → 沟候选
    std::vector<Candidate> candidates;
    std::vector<bool> col_has_ditch(cols_, false);
    for (int col = 0; col < cols_; ++col)
    {
      auto &col_pts = cols[col];
      if (col_pts.size() < 2) continue;

      // 近→远: 垂直角升序（俯角从大到小）
      std::sort(col_pts.begin(), col_pts.end(),
                [](const pcl::PointXYZI &a, const pcl::PointXYZI &b) {
                  return verticalAngle(a) < verticalAngle(b);
                });

      for (size_t i = 0; i + 1 < col_pts.size(); ++i)
      {
        const pcl::PointXYZI &near_pt = col_pts[i];
        const pcl::PointXYZI &far_pt = col_pts[i + 1];

        const double beta_near = -verticalAngle(near_pt);  // 近点俯角（较大）
        const double beta_far = -verticalAngle(far_pt);    // 远点俯角（较小）
        if (beta_far <= 0.0 || beta_far >= beta_near) continue;

        const double gap_th = mount_height_ *
                              (1.0 / std::tan(beta_far) - 1.0 / std::tan(beta_near));
        if (gap_th < min_gap_) continue;

        const double gap_meas = horizontalDistance(far_pt) - horizontalDistance(near_pt);
        if (gap_meas <= 0.0) continue;

        const double ratio = gap_meas / gap_th;
        if (ratio > ratio_thresh_ && (gap_meas - gap_th) > min_width_)
        {
          col_has_ditch[col] = true;
          candidates.push_back({near_pt, static_cast<float>(ratio), false});
        }
      }
    }

    // 悬崖检测: 该列没有沟壑边缘时，若最远回波明显近于探测范围（下方无回波），
    // 则最远点即为悬崖边缘。
    if (cliff_enable_)
    {
      for (int col = 0; col < cols_; ++col)
      {
        if (col_has_ditch[col]) continue;

        const auto &col_pts = cols[col];
        if (static_cast<int>(col_pts.size()) < cliff_min_pts_) continue;

        const pcl::PointXYZI &far_pt = col_pts.back();
        const double d_far = horizontalDistance(far_pt);
        if (d_far >= cliff_max_range_) continue;  // 已到探测边界，非悬崖

        // 最远点若接近最浅束（max_vert），通常是墙面/陡上坡回波，排除
        if (verticalAngle(far_pt) > max_vert_rad_ - cliff_vert_margin_rad_) continue;

        // 最远点与次远点之间应存在正常线束间隔（排除墙面等同一距离点簇）
        const double d_prev = horizontalDistance(col_pts[col_pts.size() - 2]);
        if (d_far - d_prev < min_gap_) continue;

        // 列内水平距离应大致随角度单调递增（地面特征）
        bool monotonic = true;
        for (size_t i = 1; i < col_pts.size(); ++i)
        {
          if (horizontalDistance(col_pts[i]) <
              horizontalDistance(col_pts[i - 1]) - min_gap_)
          {
            monotonic = false;
            break;
          }
        }
        if (!monotonic) continue;

        candidates.push_back({far_pt, -1.0f, true});
      }
    }

    // 贪心聚类: 沟沿横向连续的点归为同一段沟/悬崖边界
    std::vector<std::vector<Candidate>> clusters;
    for (const auto &cand : candidates)
    {
      int best = -1;
      double best_dist = cluster_dist_;
      for (size_t k = 0; k < clusters.size(); ++k)
      {
        for (const auto &member : clusters[k])
        {
          const double d = std::hypot(member.point.x - cand.point.x,
                                      member.point.y - cand.point.y);
          if (d < best_dist)
          {
            best_dist = d;
            best = static_cast<int>(k);
          }
        }
      }
      if (best >= 0) clusters[best].push_back(cand);
      else clusters.push_back({cand});
    }

    // 过滤点数不足的小段，并按水平角升序（-60°→+60°）排列
    std::vector<Candidate> line_points;
    for (const auto &cluster : clusters)
    {
      if (static_cast<int>(cluster.size()) < min_pts_) continue;
      line_points.insert(line_points.end(), cluster.begin(), cluster.end());
    }
    std::sort(line_points.begin(), line_points.end(),
              [](const Candidate &a, const Candidate &b) {
                return std::atan2(a.point.y, a.point.x) < std::atan2(b.point.y, b.point.x);
              });

    std::vector<Candidate> ditch_points, cliff_points;
    for (const auto &cand : line_points)
    {
      (cand.is_cliff ? cliff_points : ditch_points).push_back(cand);
    }

    // 边坡告警分档: 最近距离 < edge_danger_dist → DANGER;
    //              ≤ edge_caution_dist → CAUTION; 其余（含无边坡）→ SAFE
    // 契约 §5.2: warning = (level ≠ LEVEL_SAFE)；header 在 publishEdgeLocked() 中统一填充。
    dth_messages::msg::EdgeWarning edge_msg;

    if (!ditch_points.empty() || !cliff_points.empty())
    {
      double sum_dist = 0.0;
      double min_dist = std::numeric_limits<double>::max();
      for (const auto &cand : ditch_points)
      {
        const double d = horizontalDistance(cand.point);
        sum_dist += d;
        min_dist = std::min(min_dist, d);
      }
      for (const auto &cand : cliff_points)
      {
        const double d = horizontalDistance(cand.point);
        sum_dist += d;
        min_dist = std::min(min_dist, d);
      }
      const double mean_dist =
          sum_dist / static_cast<double>(ditch_points.size() + cliff_points.size());

      edge_msg.dist_to_edge_m = static_cast<float>(min_dist);
      if (min_dist < edge_danger_dist_)
      {
        edge_msg.warning = true;
        edge_msg.level = dth_messages::msg::EdgeWarning::LEVEL_DANGER;
      }
      else if (min_dist <= edge_caution_dist_)
      {
        edge_msg.warning = true;
        edge_msg.level = dth_messages::msg::EdgeWarning::LEVEL_CAUTION;
      }
      else
      {
        edge_msg.warning = false;
        edge_msg.level = dth_messages::msg::EdgeWarning::LEVEL_SAFE;
      }

      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "%s 检测到边坡: 最近距离=%.2f m, 平均距离=%.2f m (沟壑 %zu pts, 悬崖 %zu pts) → %s",
                           pothole_detection::toString(gate_->side()),
                           min_dist, mean_dist, ditch_points.size(), cliff_points.size(),
                           levelName(edge_msg.level));
    }
    else
    {
      edge_msg.dist_to_edge_m = 999.0f;  // 无数据哨兵值（契约 §9）
      edge_msg.warning = false;
      edge_msg.level = dth_messages::msg::EdgeWarning::LEVEL_SAFE;

      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "%s 没有检测到边坡 → SAFE",
                           pothole_detection::toString(gate_->side()));
    }

    // 立即发布本帧结果，并记录状态供心跳定时器补发（契约 §4.3 header.stamp = 节点时钟 now()）
    {
      std::lock_guard<std::mutex> lock(edge_mutex_);
      cloud_seen_ = true;
      last_cloud_time_ = this->now();
      publishEdgeLocked(edge_msg);
      last_edge_ = edge_msg;
    }

    // 发布点云: 沟壑 intensity 编码间距比，悬崖固定 255
    pcl::PointCloud<pcl::PointXYZI> ditch_cloud;
    for (const auto &cand : ditch_points)
    {
      pcl::PointXYZI pt = cand.point;
      const float normalized = std::clamp((cand.ratio - 1.0f) / 4.0f, 0.0f, 1.0f);
      pt.intensity = normalized * 255.0f;
      ditch_cloud.push_back(pt);
    }
    pcl::PointCloud<pcl::PointXYZI> cliff_cloud;
    for (const auto &cand : cliff_points)
    {
      pcl::PointXYZI pt = cand.point;
      pt.intensity = 255.0f;
      cliff_cloud.push_back(pt);
    }

    // Marker: 沟壑红色折线 + 黄色主方向轴; 悬崖橙色折线 + 紫色主方向轴
    const rclcpp::Time stamp(msg->header.stamp);
    visualization_msgs::msg::MarkerArray lines;
    if (ditch_points.empty() && cliff_points.empty())
    {
      // 无沟/悬崖时清除所有旧线，避免刷新残留
      visualization_msgs::msg::Marker clear;
      clear.action = visualization_msgs::msg::Marker::DELETEALL;
      lines.markers.push_back(clear);
    }
    else
    {
      if (!ditch_points.empty())
      {
        lines.markers.push_back(lineMarker(ditch_points, "ditch", 0, 1.0f, 0.0f, 0.0f, 0.1f, stamp));
        lines.markers.push_back(axisMarker(ditch_points, "ditch_axis", 1, 1.0f, 1.0f, 0.0f, stamp));
      }
      else
      {
        lines.markers.push_back(deleteMarker("ditch", 0));
        lines.markers.push_back(deleteMarker("ditch_axis", 1));
      }

      if (!cliff_points.empty())
      {
        lines.markers.push_back(lineMarker(cliff_points, "cliff", 0, 1.0f, 0.5f, 0.0f, 0.12f, stamp));
        lines.markers.push_back(axisMarker(cliff_points, "cliff_axis", 1, 1.0f, 0.0f, 1.0f, stamp));
      }
      else
      {
        lines.markers.push_back(deleteMarker("cliff", 0));
        lines.markers.push_back(deleteMarker("cliff_axis", 1));
      }
    }

    if (!ditch_cloud.empty())
    {
      sensor_msgs::msg::PointCloud2 out;
      pcl::toROSMsg(ditch_cloud, out);
      out.header.stamp = msg->header.stamp;
      out.header.frame_id = frame_id_;
      pub_ditch_->publish(out);
    }

    if (!cliff_cloud.empty())
    {
      sensor_msgs::msg::PointCloud2 out;
      pcl::toROSMsg(cliff_cloud, out);
      out.header.stamp = msg->header.stamp;
      out.header.frame_id = frame_id_;
      pub_cliff_->publish(out);
    }

    pub_line_->publish(lines);
  }

  // 发布安全告警（999.0 / SAFE）并刷新心跳时间戳：
  // 门控跳过检测的一侧即由此保持「在线 + 安全」语义（契约 §4.3）
  void publishSafeEdge()
  {
    dth_messages::msg::EdgeWarning msg;
    msg.warning = false;
    msg.dist_to_edge_m = 999.0f;  // 无数据哨兵值（契约 §9）
    msg.level = dth_messages::msg::EdgeWarning::LEVEL_SAFE;

    std::lock_guard<std::mutex> lock(edge_mutex_);
    cloud_seen_ = true;
    last_cloud_time_ = this->now();
    publishEdgeLocked(msg);
    last_edge_ = msg;
  }

  // 填充 header 并发布（调用方需持有 edge_mutex_）
  // 契约 §4.3: header.stamp 必须每帧填节点时钟 now()（尊重 use_sim_time）
  void publishEdgeLocked(dth_messages::msg::EdgeWarning &msg)
  {
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;
    pub_edge_->publish(msg);
    last_edge_publish_time_ = msg.header.stamp;
  }

  // 心跳定时器: 与点云处理解耦，保证 /perception/edge_warning 周期发布（契约 §4.3 ≥5Hz）。
  // 点云正常时由 onCloud 按帧发布，此处仅在超过 1.5 个心跳周期未发布时补发最新状态；
  // 点云断流超过 edge_timeout_s 时切换为安全默认值（契约 §3.3）。
  void onEdgeHeartbeat()
  {
    std::lock_guard<std::mutex> lock(edge_mutex_);
    const rclcpp::Time now = this->now();

    const bool sensor_ok =
        cloud_seen_ && (now - last_cloud_time_).seconds() <= edge_timeout_s_;

    if (!sensor_ok)
    {
      // 启动后首帧点云到来前静默发安全态；已收到过点云才提示断流
      if (cloud_seen_ && !stale_warned_)
      {
        RCLCPP_WARN(this->get_logger(),
                    "%s 点云 %s 断流超过 %.2f s，原始边坡告警转安全默认值（999.0/SAFE）",
                    pothole_detection::toString(gate_->side()),
                    input_topic_.c_str(), edge_timeout_s_);
        stale_warned_ = true;
      }

      dth_messages::msg::EdgeWarning safe;
      safe.warning = false;
      safe.dist_to_edge_m = 999.0f;  // 无数据哨兵值（契约 §9）
      safe.level = dth_messages::msg::EdgeWarning::LEVEL_SAFE;
      publishEdgeLocked(safe);
      return;
    }

    if (stale_warned_)
    {
      RCLCPP_INFO(this->get_logger(),
                  "%s 点云 %s 恢复，原始边坡告警恢复正常检测输出",
                  pothole_detection::toString(gate_->side()), input_topic_.c_str());
      stale_warned_ = false;
    }

    if ((now - last_edge_publish_time_).seconds() >= edge_heartbeat_period_s_ * 1.5)
    {
      dth_messages::msg::EdgeWarning msg = last_edge_;
      publishEdgeLocked(msg);
    }
  }

  visualization_msgs::msg::Marker lineMarker(const std::vector<Candidate> &pts,
                                             const std::string &ns, int id,
                                             float r, float g, float b, float width,
                                             const rclcpp::Time &stamp) const
  {
    visualization_msgs::msg::Marker m;
    m.header.stamp = stamp;
    m.header.frame_id = frame_id_;
    m.ns = ns;
    m.id = id;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = width;
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 1.0f;
    m.lifetime = rclcpp::Duration::from_seconds(0.5);  // 自动过期，避免残留
    for (const auto &cand : pts)
    {
      geometry_msgs::msg::Point p;
      p.x = cand.point.x;
      p.y = cand.point.y;
      p.z = cand.point.z;
      m.points.push_back(p);
    }
    return m;
  }

  visualization_msgs::msg::Marker axisMarker(const std::vector<Candidate> &pts,
                                             const std::string &ns, int id,
                                             float r, float g, float b,
                                             const rclcpp::Time &stamp)
  {
    // PCA 估计边界主方向（2D: x-y 平面）
    const size_t n = pts.size();
    double mx = 0.0, my = 0.0, mz = 0.0;
    for (const auto &cand : pts)
    {
      mx += cand.point.x;
      my += cand.point.y;
      mz += cand.point.z;
    }
    mx /= n; my /= n; mz /= n;

    double cxx = 0.0, cyy = 0.0, cxy = 0.0;
    for (const auto &cand : pts)
    {
      const double dx = cand.point.x - mx;
      const double dy = cand.point.y - my;
      cxx += dx * dx;
      cyy += dy * dy;
      cxy += dx * dy;
    }

    // 2x2 协方差矩阵 [[cxx, cxy],[cxy, cyy]] 的最大特征值与对应特征向量（主方向）
    const double tr = cxx + cyy;
    const double det = cxx * cyy - cxy * cxy;
    const double lambda_max = 0.5 * (tr + std::sqrt(std::max(0.0, tr * tr - 4.0 * det)));

    double ux = cxy;
    double uy = lambda_max - cxx;
    const double alt_norm = (lambda_max - cyy) * (lambda_max - cyy) + cxy * cxy;
    if (ux * ux + uy * uy < alt_norm)
    {
      ux = lambda_max - cyy;
      uy = cxy;
    }
    const double norm = std::hypot(ux, uy);
    if (norm > 1e-9) { ux /= norm; uy /= norm; }
    else { ux = 1.0; uy = 0.0; }

    // 沿主方向的均方差作为线段半长
    double var = 0.0;
    for (const auto &cand : pts)
    {
      const double proj = (cand.point.x - mx) * ux + (cand.point.y - my) * uy;
      var += proj * proj;
    }
    const double half_len = 1.5 * std::sqrt(var / n);  // 主方向线适当加长

    visualization_msgs::msg::Marker m;
    m.header.stamp = stamp;
    m.header.frame_id = frame_id_;
    m.ns = ns;
    m.id = id;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.08;  // 线宽（米）
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 1.0f;
    m.lifetime = rclcpp::Duration::from_seconds(0.5);

    geometry_msgs::msg::Point pa, pb;
    pa.x = mx - half_len * ux;
    pa.y = my - half_len * uy;
    pa.z = mz;
    pb.x = mx + half_len * ux;
    pb.y = my + half_len * uy;
    pb.z = mz;
    m.points.push_back(pa);
    m.points.push_back(pb);

    return m;
  }

  visualization_msgs::msg::Marker deleteMarker(const std::string &ns, int id) const
  {
    visualization_msgs::msg::Marker m;
    m.action = visualization_msgs::msg::Marker::DELETE;
    m.ns = ns;
    m.id = id;
    return m;
  }

  static const char *levelName(uint8_t level)
  {
    switch (level)
    {
      case dth_messages::msg::EdgeWarning::LEVEL_DANGER:  return "DANGER";
      case dth_messages::msg::EdgeWarning::LEVEL_CAUTION: return "CAUTION";
      default: return "SAFE";
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ditch_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cliff_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_voxel_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_line_;
  rclcpp::Publisher<dth_messages::msg::EdgeWarning>::SharedPtr pub_edge_;
  rclcpp::TimerBase::SharedPtr edge_heartbeat_timer_;

  std::string lidar_name_, input_topic_, ditch_topic_, line_topic_, frame_id_;
  std::string cliff_topic_, voxel_topic_, edge_topic_;

  // 定向检测门控（契约 §3.1/§6.2）
  std::unique_ptr<pothole_detection::MotionGate> gate_;
  bool motion_gate_enable_{true};
  pothole_detection::MotionState last_motion_{pothole_detection::MotionState::kStopped};

  double mount_height_;
  double horiz_fov_deg_, horiz_res_deg_;
  double min_vert_deg_, max_vert_deg_;
  double max_range_, min_range_;
  double ratio_thresh_, min_width_, min_gap_, cluster_dist_;
  double voxel_leaf_size_;
  double cliff_max_range_, cliff_vert_margin_deg_, cliff_vert_margin_rad_;
  double edge_danger_dist_, edge_caution_dist_;
  double edge_heartbeat_hz_, edge_heartbeat_period_s_, edge_timeout_s_;
  int min_pts_;
  int cliff_min_pts_;
  bool cliff_enable_;
  int cols_;
  double horiz_min_rad_, horiz_res_rad_;
  double min_vert_rad_, max_vert_rad_;

  // 边坡告警心跳状态（onCloud 与心跳定时器共享，加锁保护）
  std::mutex edge_mutex_;
  dth_messages::msg::EdgeWarning last_edge_;  // 最近一次检测结果
  rclcpp::Time last_cloud_time_;              // 最近一帧点云到达时刻（节点时钟）
  rclcpp::Time last_edge_publish_time_;       // 最近一次 edge_warning 发布时刻
  bool cloud_seen_{false};                    // 是否已收到过点云
  bool stale_warned_{false};                  // 断流告警只打一次
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PotholeDetection>());
  rclcpp::shutdown();
  return 0;
}
