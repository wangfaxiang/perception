/**
 * @file elevation_obstacle_detection_node.cpp
 * @brief 局部栅格高程差法地面障碍物检测（矿山 / 起伏地面适配）
 *
 * 订阅 pointcloud_preprocess 输出的 leveled_filtered_smoothed 点云（全量扬尘过滤 + 降采样 + 高度平滑），
 * 在 xy 平面建二维栅格：
 *   1) 每格取最低点高度(min z) 建 DEM
 *   2) DEM 做邻域最小值低通 → 局部参考地面（跟随地面起伏，不把坡面/车辙当障碍物）
 *   3) 点相对其局部参考地面的高度差 > height_threshold → 障碍物点
 *   4) 障碍物点欧氏聚类 → 水平 PCA 包围盒
 *
 * 输出（均按 lidar_name 区分前后雷达）：
 *   /<lidar>/elevation_obstacle/cloud       PointCloud2   障碍物点云（rviz 点显示）
 *   /<lidar>/elevation_obstacle/boxs        Boxs          障碍物包围盒
 *   /<lidar>/elevation_obstacle/marker      MarkerArray   障碍物包围盒可视化（面+线框+角点+文字）
 *   /<lidar>/elevation_obstacle/ground_grid PointCloud2   局部参考地面栅格点（调试用，可选）
 *
 * 与 cluster 的差异：cluster 依赖上游绝对高度分割（矿山起伏地面会误分割），
 * 本节点用「局部参考地面 + 相对高度差」重新判定障碍物，对起伏地形更鲁棒。
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/header.hpp>
#include <box_msg/msg/boxs.hpp>
#include <box_msg/msg/box.hpp>
#include <dth_messages/msg/obstacle_warning.hpp>
#include <perception_common/motion_state.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{
// 二维栅格索引编码：gx 高 32 位、gy 低 32 位（支持负坐标）
inline int64_t cellKey(int gx, int gy)
{
  return (static_cast<int64_t>(gx) << 32) | static_cast<int64_t>(static_cast<uint32_t>(gy));
}

inline int keyToGx(int64_t key)
{
  return static_cast<int>(key >> 32);
}

inline int keyToGy(int64_t key)
{
  return static_cast<int>(static_cast<uint32_t>(key & 0xFFFFFFFFLL));
}

// 障碍物类别名（类型由参数指定：本节点只做几何检测，无类别识别能力）
const char *obstacleLabel(int type)
{
  switch (type) {
    case 1: return "人员";    // dth_messages/ObstacleWarning::TYPE_PERSON
    case 2: return "车辆";    // TYPE_VEHICLE
    default: return "障碍物";  // TYPE_OTHER
  }
}

// 方位名（按本雷达坐标系：+x 前、+y 左，与消息 frame_id 声明的坐标系一致）
const char *directionName(float x, float y)
{
  const double ang = std::atan2(static_cast<double>(y), static_cast<double>(x)) * 180.0 / M_PI;
  if (ang >= -22.5 && ang < 22.5) return "前方";
  if (ang >= 22.5 && ang < 67.5) return "左前";
  if (ang >= 67.5 && ang < 112.5) return "左方";
  if (ang >= 112.5 && ang < 157.5) return "左后";
  if (ang >= 157.5 || ang < -157.5) return "后方";
  if (ang >= -157.5 && ang < -112.5) return "右后";
  if (ang >= -112.5 && ang < -67.5) return "右方";
  return "右前";
}
}  // namespace

class ElevationObstacleDetector : public rclcpp::Node
{
public:
  ElevationObstacleDetector()
  : Node("elevation_obstacle_detection")
  {
    lidar_name_ = declare_parameter<std::string>("lidar_name", "");
    const std::string prefix = lidar_name_.empty() ? "" : ("/" + lidar_name_);

    input_topic_ = declare_parameter<std::string>(
      "input_topic", prefix + "/leveled_filtered_smoothed");
    frame_id_ = declare_parameter<std::string>(
      "frame_id", lidar_name_.empty() ? "rslidar" : lidar_name_ + "_rslidar");

    const std::string cloud_topic = declare_parameter<std::string>(
      "obstacle_cloud_topic", prefix + "/elevation_obstacle/cloud");
    const std::string boxs_topic = declare_parameter<std::string>(
      "boxs_topic", prefix + "/elevation_obstacle/boxs");
    const std::string marker_topic = declare_parameter<std::string>(
      "marker_topic", prefix + "/elevation_obstacle/marker");
    const std::string ground_grid_topic = declare_parameter<std::string>(
      "ground_grid_topic", prefix + "/elevation_obstacle/ground_grid");

    max_range_ = declare_parameter<double>("max_range", 20.0);
    min_range_ = declare_parameter<double>("min_range", 0.3);
    grid_size_m_ = declare_parameter<double>("grid_size_m", 0.3);
    ground_window_ = declare_parameter<int>("ground_window", 1);
    height_threshold_ = declare_parameter<double>("height_threshold", 0.25);
    cluster_tolerance_ = declare_parameter<double>("cluster_tolerance", 0.5);
    min_cluster_size_ = declare_parameter<int>("min_cluster_size", 10);
    max_cluster_size_ = declare_parameter<int>("max_cluster_size", 100000);
    publish_ground_grid_ = declare_parameter<bool>("publish_ground_grid", true);

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ElevationObstacleDetector::onCloud, this, std::placeholders::_1));

    pub_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic, rclcpp::QoS(10));
    pub_boxs_ = create_publisher<box_msg::msg::Boxs>(boxs_topic, rclcpp::QoS(10));
    pub_marker_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic, rclcpp::QoS(10));
    pub_ground_grid_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(ground_grid_topic, rclcpp::QoS(10));

    // ---- 障碍物告警（与 cluster 一致：本实例只发「单雷达原始告警」到节点私有话题，
    // 契约话题 /perception/obstacle_warning 与 /perception/stop_command 由 perception_common 的
    // warning_fusion 取类别优先者合并后发布）----
    this->declare_parameter<std::string>("obstacle_warning_raw_topic", "~/obstacle_warning_raw");
    this->declare_parameter<double>("obstacle_heartbeat_hz", 10.0);
    this->declare_parameter<double>("obstacle_timeout_s", 0.5);
    this->declare_parameter("obstacle_warn_dist", rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("obstacle_danger_dist", rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("obstacle_type", 3);
    this->declare_parameter("motion_gate_enable", true);

    this->get_parameter("obstacle_warning_raw_topic", obstacle_topic_);
    this->get_parameter("obstacle_heartbeat_hz", obstacle_heartbeat_hz_);
    this->get_parameter("obstacle_timeout_s", obstacle_timeout_s_);
    this->get_parameter("obstacle_warn_dist", obstacle_warn_dist_);
    this->get_parameter("obstacle_danger_dist", obstacle_danger_dist_);
    this->get_parameter("obstacle_type", obstacle_type_);
    this->get_parameter("motion_gate_enable", motion_gate_enable_);

    if (!(obstacle_heartbeat_hz_ > 0.0)) obstacle_heartbeat_hz_ = 10.0;
    obstacle_heartbeat_period_s_ = 1.0 / obstacle_heartbeat_hz_;
    if (!(obstacle_warn_dist_ > 0.0))
    {
      RCLCPP_FATAL(get_logger(),
                   "obstacle_warn_dist=%.1f 非法：须由 perception_common/config/params.yaml "
                   "的 /** 共享段提供且 > 0", obstacle_warn_dist_);
      throw std::runtime_error("invalid obstacle_warn_dist");
    }
    if (obstacle_type_ < 1 || obstacle_type_ > 3)
    {
      RCLCPP_FATAL(get_logger(),
                   "obstacle_type=%d 非法：1=人员 / 2=车辆 / 3=其他（0 为契约保留值，禁止配置）",
                   obstacle_type_);
      throw std::runtime_error("invalid obstacle_type");
    }

    // 定向检测门控（与 cluster / 边坡检测共用同一份实现）
    gate_ = std::make_unique<perception_common::MotionGate>(*this, lidar_name_);

    publisher_obstacle_warning_ = create_publisher<dth_messages::msg::ObstacleWarning>(
      obstacle_topic_, rclcpp::QoS(10));

    obstacle_heartbeat_timer_ = create_wall_timer(
      std::chrono::duration<double>(obstacle_heartbeat_period_s_),
      std::bind(&ElevationObstacleDetector::onObstacleHeartbeat, this));

    RCLCPP_INFO(
      get_logger(),
      "elevation_obstacle_detection started (lidar_name=%s): input=%s, frame=%s, "
      "range=[%.1f,%.1f]m, grid=%.2fm, window=%d, height_thresh=%.2fm, "
      "cluster_tolerance=%.2fm, min_points=%d",
      lidar_name_.c_str(), input_topic_.c_str(), frame_id_.c_str(),
      min_range_, max_range_, grid_size_m_, ground_window_, height_threshold_,
      cluster_tolerance_, min_cluster_size_);
    RCLCPP_INFO(
      get_logger(),
      "障碍物告警: %s (强制类别=%s, warn_dist=%.1fm, heartbeat=%.1fHz, timeout=%.2fs)",
      obstacle_topic_.c_str(), obstacleLabel(obstacle_type_),
      obstacle_warn_dist_, obstacle_heartbeat_hz_, obstacle_timeout_s_);
    RCLCPP_INFO(get_logger(), "定向检测门控: %s（%s）",
      motion_gate_enable_ ? "on" : "off", gate_->describe().c_str());
  }

private:
  struct CellInfo
  {
    float min_z = std::numeric_limits<float>::max();
    int count = 0;
  };

  // 每个保留簇的几何摘要（与 Boxs.box 一一对应、同序），用于挑最近威胁物
  struct BoxGeom
  {
    float near_x = 0.0f;      // 距雷达最近的点（水平距离最小）
    float near_y = 0.0f;
    float near_z = 0.0f;
    float near_dist = 0.0f;   // hypot(near_x, near_y)，与契约 dist_m 自洽
    size_t points = 0;        // 簇内点数
  };

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // ---- 定向检测门控（与 cluster 一致）：前进只测前、后退只测后、停车/原地旋转双向均测 ----
    const auto motion = gate_->state();
    const bool detect = !motion_gate_enable_ || gate_->active();
    if (motion != last_motion_)
    {
      RCLCPP_INFO(get_logger(), "%s 运动方向 %s → %s（%s）",
                  perception_common::toString(gate_->side()),
                  perception_common::toString(last_motion_),
                  perception_common::toString(motion),
                  detect ? "参与检测" : "门控跳过");
      last_motion_ = motion;
    }
    if (!detect)
    {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                           "%s 门控跳过障碍物检测（运动方向 %s），发布安全默认值",
                           perception_common::toString(gate_->side()),
                           perception_common::toString(motion));
      publishSafeObstacle();
      return;
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    // ---- 1) 栅格化：每格最低点高度建 DEM ----
    std::unordered_map<int64_t, CellInfo> cells;
    cells.reserve(cloud.size());
    const float inv_grid = static_cast<float>(1.0 / grid_size_m_);

    for (const auto & p : cloud.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }
      const float range = std::hypot(p.x, p.y);
      if (range < min_range_ || range > max_range_) {
        continue;
      }
      const int gx = static_cast<int>(std::floor(p.x * inv_grid));
      const int gy = static_cast<int>(std::floor(p.y * inv_grid));
      CellInfo & cell = cells[cellKey(gx, gy)];
      cell.min_z = std::min(cell.min_z, p.z);
      ++cell.count;
    }

    // ---- 2) 邻域最小值低通 + 相对高度差判定障碍物点 ----
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle(new pcl::PointCloud<pcl::PointXYZ>());
    obstacle->header = cloud.header;
    const int w = std::max(0, ground_window_);

    for (const auto & p : cloud.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }
      const float range = std::hypot(p.x, p.y);
      if (range < min_range_ || range > max_range_) {
        continue;
      }
      const int gx = static_cast<int>(std::floor(p.x * inv_grid));
      const int gy = static_cast<int>(std::floor(p.y * inv_grid));

      // 局部参考地面 = 邻域窗口内各格最低高度的最小值（跟随起伏）
      float local_ground = std::numeric_limits<float>::max();
      bool found = false;
      for (int dx = -w; dx <= w; ++dx) {
        for (int dy = -w; dy <= w; ++dy) {
          const auto it = cells.find(cellKey(gx + dx, gy + dy));
          if (it != cells.end()) {
            local_ground = std::min(local_ground, it->second.min_z);
            found = true;
          }
        }
      }
      if (!found) {
        continue;  // 邻域内没有可靠地面参考，跳过
      }
      if (p.z - local_ground > height_threshold_) {
        obstacle->push_back(p);
      }
    }

    // ---- 3) 欧氏聚类 ----
    std::vector<pcl::PointIndices> cluster_indices;
    if (!obstacle->empty()) {
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
      tree->setInputCloud(obstacle);
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
      ec.setClusterTolerance(cluster_tolerance_);
      ec.setMinClusterSize(min_cluster_size_);
      ec.setMaxClusterSize(max_cluster_size_);
      ec.setSearchMethod(tree);
      ec.setInputCloud(obstacle);
      ec.extract(cluster_indices);
    }

    // ---- 4) 每簇生成水平 PCA 包围盒（并记录最近点摘要供告警用）----
    box_msg::msg::Boxs boxs;
    boxs.header = msg->header;
    std::vector<BoxGeom> geoms;
    geoms.reserve(cluster_indices.size());
    for (const auto & indices : cluster_indices) {
      box_msg::msg::Box box;
      BoxGeom geom;
      computeClusterBox(indices, obstacle, box, geom);
      boxs.box.push_back(box);
      geoms.push_back(geom);
    }

    // ---- 发布 ----
    publishCloud(msg->header, obstacle);
    publishGroundGrid(msg->header, cells);
    pub_boxs_->publish(boxs);
    pub_marker_->publish(boxsToMarkerArray(boxs, frame_id_));
    publishObstacleWarning(boxs, geoms, msg->header);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "%s 检测到 %zu 个地面障碍物（障碍物点 %zu 个，参考地面栅格 %zu 个）",
      lidar_name_.empty() ? "lidar" : lidar_name_.c_str(),
      boxs.box.size(), obstacle->size(), cells.size());
  }

  // 单簇水平 PCA 包围盒：取水平面最大方差方向作为框朝向，尺寸为沿主轴/次轴/z 的投影跨度。
  // Box 语义与 cluster 一致：w=主轴跨度、l=次轴跨度、h=z 跨度、rt=绕 z 轴 yaw。
  void computeClusterBox(
    const pcl::PointIndices & indices,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    box_msg::msg::Box & box,
    BoxGeom & geom)
  {
    const size_t n = indices.indices.size();

    float near_x = 0.0f, near_y = 0.0f, near_z = 0.0f;
    float near_dist = std::numeric_limits<float>::max();

    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for (const int index : indices.indices) {
      const auto & p = (*cloud)[index];
      centroid += Eigen::Vector3f(p.x, p.y, p.z);
      const float d = std::hypot(p.x, p.y);
      if (d < near_dist) {
        near_dist = d;
        near_x = p.x;
        near_y = p.y;
        near_z = p.z;
      }
    }
    centroid /= static_cast<float>(n);

    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for (const int index : indices.indices) {
      const auto & p = (*cloud)[index];
      const Eigen::Vector3f d(p.x - centroid.x(), p.y - centroid.y(), p.z - centroid.z());
      cov += d * d.transpose();
    }
    cov /= static_cast<float>(n);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
    const Eigen::Matrix3f evecs = solver.eigenvectors();  // 列按特征值升序，第 3 列=最大方差方向

    const Eigen::Vector3f e1 = evecs.col(2);
    Eigen::Vector3f u(e1.x(), e1.y(), 0.0f);
    float u_norm = u.norm();
    if (u_norm < 1e-6f) {
      u = Eigen::Vector3f(1.0f, 0.0f, 0.0f);  // 主方向接近竖直时退回雷达 x 轴
      u_norm = 1.0f;
    }
    u /= u_norm;
    const Eigen::Vector3f v(-u.y(), u.x(), 0.0f);

    float min_u = std::numeric_limits<float>::max(), max_u = -std::numeric_limits<float>::max();
    float min_v = std::numeric_limits<float>::max(), max_v = -std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max(), max_z = -std::numeric_limits<float>::max();
    for (const int index : indices.indices) {
      const auto & p = (*cloud)[index];
      const float pu = u.x() * p.x + u.y() * p.y;
      const float pv = v.x() * p.x + v.y() * p.y;
      min_u = std::min(min_u, pu); max_u = std::max(max_u, pu);
      min_v = std::min(min_v, pv); max_v = std::max(max_v, pv);
      min_z = std::min(min_z, p.z); max_z = std::max(max_z, p.z);
    }

    const float cu = 0.5f * (min_u + max_u);
    const float cv = 0.5f * (min_v + max_v);
    box.x = cu * u.x() + cv * v.x();
    box.y = cu * u.y() + cv * v.y();
    box.z = 0.5f * (min_z + max_z);
    box.w = max_u - min_u;
    box.l = max_v - min_v;
    box.h = max_z - min_z;
    box.rt = std::atan2(u.y(), u.x());

    geom.near_x = near_x;
    geom.near_y = near_y;
    geom.near_z = near_z;
    geom.near_dist = near_dist;
    geom.points = n;
  }

  void publishCloud(
    const std_msgs::msg::Header & header,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & obstacle)
  {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*obstacle, cloud_msg);
    cloud_msg.header = header;
    pub_cloud_->publish(cloud_msg);
  }

  // 发布局部参考地面栅格点（每格一个点，位于格中心，z=该格最低高度），用于 rviz 验证 DEM
  void publishGroundGrid(
    const std_msgs::msg::Header & header,
    const std::unordered_map<int64_t, CellInfo> & cells)
  {
    if (!publish_ground_grid_) {
      return;
    }
    pcl::PointCloud<pcl::PointXYZ> grid;
    grid.reserve(cells.size());
    const float half = static_cast<float>(0.5 * grid_size_m_);
    for (const auto & kv : cells) {
      const int gx = keyToGx(kv.first);
      const int gy = keyToGy(kv.first);
      pcl::PointXYZ p;
      p.x = gx * grid_size_m_ + half;
      p.y = gy * grid_size_m_ + half;
      p.z = kv.second.min_z;
      grid.push_back(p);
    }
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(grid, cloud_msg);
    cloud_msg.header = header;
    pub_ground_grid_->publish(cloud_msg);
  }

  // 将 Boxs 转成 rviz MarkerArray：半透明面 + 白色线框 + 顶点小球 + 距离文字
  visualization_msgs::msg::MarkerArray boxsToMarkerArray(
    const box_msg::msg::Boxs & boxs, const std::string & frame_id)
  {
    visualization_msgs::msg::MarkerArray marker_array;

    for (const char * ns : {"elevation_box_face", "elevation_box_wire",
                            "elevation_box_corner", "elevation_box_text"}) {
      visualization_msgs::msg::Marker clear;
      clear.header.frame_id = frame_id;
      clear.header.stamp = boxs.header.stamp;
      clear.ns = ns;
      clear.id = 0;
      clear.action = visualization_msgs::msg::Marker::DELETEALL;
      marker_array.markers.push_back(clear);
    }

    auto make_marker = [&](const std::string & ns, int type, int mid) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = frame_id;
      m.header.stamp = boxs.header.stamp;
      m.ns = ns;
      m.id = mid;
      m.type = type;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.x = 0.0;
      m.pose.orientation.y = 0.0;
      m.pose.orientation.z = 0.0;
      m.pose.orientation.w = 1.0;
      return m;
    };

    int id = 1;
    for (const auto & b : boxs.box) {
      const float hx = b.w * 0.5f;
      const float hy = b.l * 0.5f;
      const float hz = b.h * 0.5f;
      const float cosr = std::cos(b.rt);
      const float sinr = std::sin(b.rt);
      const float lx[8] = {-hx, hx, hx, -hx, -hx, hx, hx, -hx};
      const float ly[8] = {-hy, -hy, hy, hy, -hy, -hy, hy, hy};
      const float lz[8] = {-hz, -hz, -hz, -hz, hz, hz, hz, hz};
      float px[8], py[8], pz[8];
      for (int k = 0; k < 8; ++k) {
        px[k] = b.x + lx[k] * cosr - ly[k] * sinr;
        py[k] = b.y + lx[k] * sinr + ly[k] * cosr;
        pz[k] = b.z + lz[k];
      }

      // 1) 半透明彩色面
      visualization_msgs::msg::Marker face =
        make_marker("elevation_box_face", visualization_msgs::msg::Marker::CUBE, id);
      face.pose.position.x = b.x;
      face.pose.position.y = b.y;
      face.pose.position.z = b.z;
      face.pose.orientation.z = std::sin(b.rt * 0.5f);
      face.pose.orientation.w = std::cos(b.rt * 0.5f);
      face.scale.x = b.w;
      face.scale.y = b.l;
      face.scale.z = b.h;
      face.color.a = 0.35f;
      face.color.r = 1.0f;
      face.color.g = 0.4f;
      face.color.b = 0.1f;
      marker_array.markers.push_back(face);

      // 2) 白色线框
      static const int edges[12][2] = {
        {0, 1}, {1, 2}, {2, 3}, {3, 0},
        {4, 5}, {5, 6}, {6, 7}, {7, 4},
        {0, 4}, {1, 5}, {2, 6}, {3, 7}};
      visualization_msgs::msg::Marker wire =
        make_marker("elevation_box_wire", visualization_msgs::msg::Marker::LINE_LIST, id);
      wire.scale.x = 0.05;
      wire.color.a = 1.0f;
      wire.color.r = 1.0f;
      wire.color.g = 1.0f;
      wire.color.b = 1.0f;
      wire.points.reserve(24);
      for (const auto & e : edges) {
        geometry_msgs::msg::Point p1, p2;
        p1.x = px[e[0]]; p1.y = py[e[0]]; p1.z = pz[e[0]];
        p2.x = px[e[1]]; p2.y = py[e[1]]; p2.z = pz[e[1]];
        wire.points.push_back(p1);
        wire.points.push_back(p2);
      }
      marker_array.markers.push_back(wire);

      // 3) 顶点小球
      visualization_msgs::msg::Marker corner =
        make_marker("elevation_box_corner", visualization_msgs::msg::Marker::SPHERE_LIST, id);
      corner.scale.x = corner.scale.y = corner.scale.z = 0.12;
      corner.color.a = 1.0f;
      corner.color.r = 1.0f;
      corner.color.g = 0.4f;
      corner.color.b = 0.1f;
      corner.points.reserve(8);
      for (int k = 0; k < 8; ++k) {
        geometry_msgs::msg::Point pt;
        pt.x = px[k]; pt.y = py[k]; pt.z = pz[k];
        corner.points.push_back(pt);
      }
      marker_array.markers.push_back(corner);

      // 4) 文字：距雷达水平最近距离
      visualization_msgs::msg::Marker text =
        make_marker("elevation_box_text", visualization_msgs::msg::Marker::TEXT_VIEW_FACING, id);
      text.pose.position.x = b.x;
      text.pose.position.y = b.y;
      text.pose.position.z = b.z + hz + 0.3f;
      text.scale.z = 0.3;
      const float du = -(b.x * cosr + b.y * sinr);
      const float dv = b.x * sinr - b.y * cosr;
      const float dx = std::max(std::fabs(du) - hx, 0.0f);
      const float dy = std::max(std::fabs(dv) - hy, 0.0f);
      const float nearest = std::hypot(dx, dy);
      char label[64];
      std::snprintf(label, sizeof(label), "%.1fm", nearest);
      text.text = label;
      text.color.a = 1.0f;
      text.color.r = 1.0f;
      text.color.g = 1.0f;
      text.color.b = 1.0f;
      marker_array.markers.push_back(text);

      ++id;
    }
    return marker_array;
  }

  // ---- 障碍物告警（与 cluster 一致）：从已保留的包围盒中挑最近目标，发布单雷达原始告警 ----
  //   检测到目标（best >= 0）即 detected=true + 合法类别 + 自洽 dist_m = √(x²+y²)；
  //   「是否报警（obstacle_warn_dist）/ 是否报停（obstacle_danger_dist）」由 warning_fusion 判定，
  //   本实例只上报实际检测结果；无目标才发安全默认值（type=0、坐标 0.0、dist=999.0）。
  void publishObstacleWarning(
    const box_msg::msg::Boxs & boxs,
    const std::vector<BoxGeom> & geoms,
    const std_msgs::msg::Header & header)
  {
    dth_messages::msg::ObstacleWarning msg;
    int best = -1;
    float best_dist = std::numeric_limits<float>::max();
    for (size_t i = 0; i < geoms.size() && i < boxs.box.size(); ++i) {
      if (geoms[i].near_dist < best_dist) {
        best_dist = geoms[i].near_dist;
        best = static_cast<int>(i);
      }
    }

    if (best >= 0) {
      const BoxGeom & geom = geoms[best];
      msg.detected = true;
      msg.obstacle_type = static_cast<uint8_t>(obstacle_type_);
      msg.obstacle_x = geom.near_x;
      msg.obstacle_y = geom.near_y;
      msg.dist_m = std::hypot(msg.obstacle_x, msg.obstacle_y);  // 与坐标自洽
      msg.description = describeThreat(geom, boxs.box[best]);

      const char * level = "SAFE";
      if (best_dist <= static_cast<float>(obstacle_danger_dist_)) {
        level = "DANGER";
      } else if (best_dist <= static_cast<float>(obstacle_warn_dist_)) {
        level = "CAUTION";
      }
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "%s检测到障碍物:最近距离=%.2f m（%zu个点）→ %s（报警阈值 %.1f m / 停车阈值 %.1f m）",
        perception_common::toString(gate_->side()),
        static_cast<double>(geom.near_dist), geom.points, level,
        obstacle_warn_dist_, obstacle_danger_dist_);
    } else {
      msg.detected = false;
      msg.obstacle_type = 0;  // detected=false 时必须为 0
      msg.obstacle_x = 0.0f;
      msg.obstacle_y = 0.0f;
      msg.dist_m = 999.0f;    // 无数据哨兵值

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "%s 没有检测到障碍物 → SAFE（报警阈值 %.1f m / 停车阈值 %.1f m）",
        perception_common::toString(gate_->side()),
        obstacle_warn_dist_, obstacle_danger_dist_);
    }

    // 透传源点云 header：frame_id = 雷达系坐标，stamp = 源点云时间戳
    msg.header = header;

    std::lock_guard<std::mutex> lock(obstacle_mutex_);
    cloud_seen_ = true;
    last_cloud_time_ = now();
    publishObstacleLocked(msg, header.stamp);
    last_obstacle_ = msg;
  }

  // 门控跳过检测时发布安全默认值并刷新心跳时间戳（保持「在线 + 无目标」语义）
  void publishSafeObstacle()
  {
    dth_messages::msg::ObstacleWarning msg;
    msg.detected = false;
    msg.obstacle_type = 0;
    msg.obstacle_x = 0.0f;
    msg.obstacle_y = 0.0f;
    msg.dist_m = 999.0f;
    msg.header.frame_id = frame_id_;

    std::lock_guard<std::mutex> lock(obstacle_mutex_);
    cloud_seen_ = true;
    last_cloud_time_ = now();
    publishObstacleLocked(msg, now());
    last_obstacle_ = msg;
  }

  // 填充 header.stamp 并发布（调用方需持有 obstacle_mutex_）
  void publishObstacleLocked(dth_messages::msg::ObstacleWarning & msg, const rclcpp::Time & stamp)
  {
    msg.header.stamp = stamp;
    publisher_obstacle_warning_->publish(msg);
    last_obstacle_publish_time_ = now();
  }

  // 心跳定时器：与点云处理解耦，保证原始告警话题周期发布；点云断流超时切安全默认值
  void onObstacleHeartbeat()
  {
    std::lock_guard<std::mutex> lock(obstacle_mutex_);
    const rclcpp::Time now_t = now();
    const bool sensor_ok = cloud_seen_ && (now_t - last_cloud_time_).seconds() <= obstacle_timeout_s_;

    if (!sensor_ok) {
      if (cloud_seen_ && !obstacle_stale_warned_) {
        RCLCPP_WARN(get_logger(),
                    "%s 点云断流超过 %.2f s，原始障碍物告警转安全默认值（未检出/999.0）",
                    perception_common::toString(gate_->side()), obstacle_timeout_s_);
        obstacle_stale_warned_ = true;
      }

      dth_messages::msg::ObstacleWarning safe;
      safe.detected = false;
      safe.obstacle_type = 0;
      safe.obstacle_x = 0.0f;
      safe.obstacle_y = 0.0f;
      safe.dist_m = 999.0f;
      safe.header.frame_id = frame_id_;
      publishObstacleLocked(safe, now_t);
      return;
    }

    if (obstacle_stale_warned_) {
      RCLCPP_INFO(get_logger(), "%s 点云恢复，原始障碍物告警恢复正常输出",
                  perception_common::toString(gate_->side()));
      obstacle_stale_warned_ = false;
    }

    if ((now_t - last_obstacle_publish_time_).seconds() >= obstacle_heartbeat_period_s_ * 1.5) {
      dth_messages::msg::ObstacleWarning msg = last_obstacle_;  // 保留 header（frame_id + 源点云时间戳）
      publishObstacleLocked(msg, msg.header.stamp);
    }
  }

  // 中文简述：方位按本雷达坐标系给出，前缀标明是哪台雷达
  std::string describeThreat(const BoxGeom & geom, const box_msg::msg::Box & box) const
  {
    const char * prefix = lidar_name_ == "front" ? "前雷达 " : (lidar_name_ == "rear" ? "后雷达 " : "");
    char buf[192];
    std::snprintf(buf, sizeof(buf), "%s%s %.1fm %s(%.1f×%.1fm)", prefix,
                  directionName(geom.near_x, geom.near_y), static_cast<double>(geom.near_dist),
                  obstacleLabel(obstacle_type_), static_cast<double>(box.w),
                  static_cast<double>(box.l));
    return std::string(buf);
  }

  // ---- 参数 ---- //
  std::string lidar_name_;
  std::string input_topic_;
  std::string frame_id_;
  double max_range_ = 20.0;
  double min_range_ = 0.3;
  double grid_size_m_ = 0.3;
  int ground_window_ = 1;
  double height_threshold_ = 0.25;
  double cluster_tolerance_ = 0.5;
  int min_cluster_size_ = 10;
  int max_cluster_size_ = 100000;
  bool publish_ground_grid_ = true;

  // ---- 障碍物告警参数 ---- //
  std::string obstacle_topic_;
  double obstacle_heartbeat_hz_ = 10.0;
  double obstacle_heartbeat_period_s_ = 0.1;
  double obstacle_timeout_s_ = 0.5;
  double obstacle_warn_dist_ = 0.0;    // 威胁物上报距离（米），来源: 共享参数文件（无代码默认值）
  double obstacle_danger_dist_ = 0.0;  // 障碍物急停距离（米），仅日志展示；实际急停由 warning_fusion 判定
  int obstacle_type_ = 3;              // 强制指定的类别（1 人员 / 2 车辆 / 3 其他）
  bool motion_gate_enable_ = true;

  // ---- 订阅 / 发布 ---- //
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
  rclcpp::Publisher<box_msg::msg::Boxs>::SharedPtr pub_boxs_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_grid_;
  rclcpp::Publisher<dth_messages::msg::ObstacleWarning>::SharedPtr publisher_obstacle_warning_;

  // ---- 运动方向定向检测门控 + 告警心跳状态 ---- //
  std::unique_ptr<perception_common::MotionGate> gate_;
  perception_common::MotionState last_motion_{perception_common::MotionState::kStopped};
  rclcpp::TimerBase::SharedPtr obstacle_heartbeat_timer_;
  std::mutex obstacle_mutex_;
  dth_messages::msg::ObstacleWarning last_obstacle_;
  rclcpp::Time last_obstacle_publish_time_;
  rclcpp::Time last_cloud_time_;
  bool cloud_seen_ = false;
  bool obstacle_stale_warned_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ElevationObstacleDetector>());
  rclcpp::shutdown();
  return 0;
}
