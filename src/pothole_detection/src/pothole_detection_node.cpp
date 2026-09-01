/**
 * @file pothole_detection_node.cpp
 * @brief 基于相邻线束地面落点水平间距比检测沟壑（负障碍物）
 *
 * 适用: 速腾聚创 E1R 固态雷达（120°×90°，1200×144），水平安装在车体上。
 *
 * 原理:
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
 * 支持多雷达: 通过 lidar_name 参数（front/rear）区分前/后雷达，同一节点可启动多个实例。
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

#include <algorithm>
#include <cmath>
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
    // 雷达标识（front / rear），用于推导默认话题名与 frame_id
    std::string lidar_name;
    this->declare_parameter<std::string>("lidar_name", "");
    this->get_parameter("lidar_name", lidar_name);

    const std::string prefix = lidar_name.empty() ? "" : ("/" + lidar_name);
    this->declare_parameter<std::string>("input_topic", prefix + "/rslidar_points");
    this->declare_parameter<std::string>("ditch_cloud_topic", prefix + "/ditch_cloud");
    this->declare_parameter<std::string>("ditch_line_topic", prefix + "/ditch_line");
    this->declare_parameter<std::string>("frame_id",
                                         lidar_name.empty() ? "rslidar" : (lidar_name + "_rslidar"));

    this->declare_parameter("mount_height", 1.2);         // 雷达安装高度（米）
    this->declare_parameter("horiz_fov_deg", 120.0);      // 水平视场角（度）
    this->declare_parameter("horiz_res_deg", 0.1);        // 列分辨率（须与雷达原生水平分辨率一致）
    this->declare_parameter("min_vert_angle_deg", -45.0); // 参与检测的最小垂直角（最近处）
    this->declare_parameter("max_vert_angle_deg", -1.0);  // 参与检测的最大垂直角（最远处，须 < 0）
    this->declare_parameter("max_range", 30.0);           // 参与检测的最大水平距离（米）
    this->declare_parameter("min_range", 0.5);            // 参与检测的最小水平距离（米）
    this->declare_parameter("ditch_ratio_thresh", 10.0);   // 间距比阈值
    this->declare_parameter("ditch_min_width", 0.5);      // 最小沟宽（米）
    this->declare_parameter("min_gap_m", 0.02);           // 最小理论线束间距（米）
    this->declare_parameter("cluster_dist", 1.5);         // 沟沿点聚类距离（米）
    this->declare_parameter("min_points_per_ditch", 5);   // 每段沟最少点数（≈横向连续列数）

    this->get_parameter("input_topic", input_topic_);
    this->get_parameter("ditch_cloud_topic", ditch_topic_);
    this->get_parameter("ditch_line_topic", line_topic_);
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

    cols_ = static_cast<int>(horiz_fov_deg_ / horiz_res_deg_);
    horiz_min_rad_ = -horiz_fov_deg_ / 2.0 * kDeg2Rad;
    horiz_res_rad_ = horiz_res_deg_ * kDeg2Rad;
    min_vert_rad_ = min_vert_deg_ * kDeg2Rad;
    max_vert_rad_ = max_vert_deg_ * kDeg2Rad;

    sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&PotholeDetection::onCloud, this, std::placeholders::_1));
    pub_ditch_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(ditch_topic_, 10);
    pub_line_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(line_topic_, 10);

    RCLCPP_INFO(this->get_logger(),
                "Ditch detection started (lidar=%s, height=%.2fm, ratio>%.1f, width>%.2fm)",
                lidar_name.empty() ? "single" : lidar_name.c_str(),
                mount_height_, ratio_thresh_, min_width_);
  }

private:
  struct Candidate
  {
    pcl::PointXYZI point;
    float ratio;
  };

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *cloud);

    std::vector<int> nan_indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, nan_indices);

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
    for (auto &col_pts : cols)
    {
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
          candidates.push_back({near_pt, static_cast<float>(ratio)});
        }
      }
    }

    // 贪心聚类: 沟沿横向连续的点归为同一段沟
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

    // 发布沟点云 + 一条按水平角连线的 Marker 线（所有沟点合成一条折线）
    pcl::PointCloud<pcl::PointXYZI> ditch_cloud;
    std::vector<Candidate> line_points;
    for (const auto &cluster : clusters)
    {
      if (static_cast<int>(cluster.size()) < min_pts_) continue;
      line_points.insert(line_points.end(), cluster.begin(), cluster.end());
    }

    // 所有沟点按水平角升序（-60°→+60°）排列，连成一条折线
    std::sort(line_points.begin(), line_points.end(),
              [](const Candidate &a, const Candidate &b) {
                return std::atan2(a.point.y, a.point.x) < std::atan2(b.point.y, b.point.x);
              });

    visualization_msgs::msg::MarkerArray lines;
    if (line_points.empty())
    {
      // 无沟时清除所有旧线，避免刷新残留
      visualization_msgs::msg::Marker clear;
      clear.action = visualization_msgs::msg::Marker::DELETEALL;
      lines.markers.push_back(clear);
    }
    else
    {
      visualization_msgs::msg::Marker line;
      line.header.stamp = msg->header.stamp;
      line.header.frame_id = frame_id_;
      line.ns = "ditch";
      line.id = 0;
      line.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line.action = visualization_msgs::msg::Marker::ADD;
      line.scale.x = 0.1;   // 线宽（米）
      line.color.r = 1.0f;  // 红色
      line.color.g = 0.0f;
      line.color.b = 0.0f;
      line.color.a = 1.0f;
      line.lifetime = rclcpp::Duration::from_seconds(0.5);  // 自动过期，避免残留

      for (const auto &cand : line_points)
      {
        pcl::PointXYZI pt = cand.point;
        // intensity 编码归一化间距比: ratio ∈ [1,5] → [0,255]
        const float normalized = std::clamp((cand.ratio - 1.0f) / 4.0f, 0.0f, 1.0f);
        pt.intensity = normalized * 255.0f;
        ditch_cloud.push_back(pt);

        geometry_msgs::msg::Point p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = pt.z;
        line.points.push_back(p);
      }
      lines.markers.push_back(line);
    }

    if (!ditch_cloud.empty())
    {
      sensor_msgs::msg::PointCloud2 out;
      pcl::toROSMsg(ditch_cloud, out);
      out.header.stamp = msg->header.stamp;
      out.header.frame_id = frame_id_;
      pub_ditch_->publish(out);
    }

    pub_line_->publish(lines);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ditch_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_line_;

  std::string input_topic_, ditch_topic_, line_topic_, frame_id_;
  double mount_height_;
  double horiz_fov_deg_, horiz_res_deg_;
  double min_vert_deg_, max_vert_deg_;
  double max_range_, min_range_;
  double ratio_thresh_, min_width_, min_gap_, cluster_dist_;
  int min_pts_;
  int cols_;
  double horiz_min_rad_, horiz_res_rad_;
  double min_vert_rad_, max_vert_rad_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PotholeDetection>());
  rclcpp::shutdown();
  return 0;
}
