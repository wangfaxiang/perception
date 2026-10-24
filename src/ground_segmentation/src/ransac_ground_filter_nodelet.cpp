// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ground_segmentation/ransac_ground_filter_nodelet.hpp"

#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>

#include <Eigen/Geometry>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{
// 距离自适应体素降采样使用的体素索引与哈希
struct VoxelKey
{
  std::int64_t ix;
  std::int64_t iy;
  std::int64_t iz;

  bool operator==(const VoxelKey & other) const
  {
    return ix == other.ix && iy == other.iy && iz == other.iz;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const
  {
    std::size_t seed = std::hash<std::int64_t>{}(key.ix);
    seed ^= std::hash<std::int64_t>{}(key.iy) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<std::int64_t>{}(key.iz) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};

Eigen::Vector3d getArbitraryOrthogonalVector(const Eigen::Vector3d & input)
{
  const double x = input.x();
  const double y = input.y();
  const double z = input.z();
  const double x2 = std::pow(x, 2);
  const double y2 = std::pow(y, 2);
  const double z2 = std::pow(z, 2);

  Eigen::Vector3d unit_vec{0, 0, 0};
  if (x2 <= y2 && x2 <= z2) {
    unit_vec.x() = 0;
    unit_vec.y() = z;
    unit_vec.z() = -y;
    unit_vec = unit_vec / std::sqrt(y2 + z2);
  } else if (y2 <= x2 && y2 <= z2) {
    unit_vec.x() = -z;
    unit_vec.y() = 0;
    unit_vec.z() = x;
    unit_vec = unit_vec / std::sqrt(z2 + x2);
  } else if (z2 <= x2 && z2 <= y2) {
    unit_vec.x() = y;
    unit_vec.y() = -x;
    unit_vec.z() = 0;
    unit_vec = unit_vec / std::sqrt(x2 + y2);
  }
  return unit_vec;
}

ground_segmentation::PlaneBasis getPlaneBasis(const Eigen::Vector3d & plane_normal)
{
  ground_segmentation::PlaneBasis basis;
  basis.e_z = plane_normal;
  basis.e_x = getArbitraryOrthogonalVector(plane_normal);
  basis.e_y = basis.e_x.cross(basis.e_z);
  return basis;
}

geometry_msgs::msg::Pose getDebugPose(const Eigen::Affine3d & plane_affine)
{
  geometry_msgs::msg::Pose debug_pose;
  const Eigen::Quaterniond quat(plane_affine.rotation());
  debug_pose.position.x = plane_affine.translation().x();
  debug_pose.position.y = plane_affine.translation().y();
  debug_pose.position.z = plane_affine.translation().z();
  debug_pose.orientation.x = quat.x();
  debug_pose.orientation.y = quat.y();
  debug_pose.orientation.z = quat.z();
  debug_pose.orientation.w = quat.w();
  return debug_pose;
}

// ---- 扬尘过滤（单帧网格密度法）----
namespace dust
{
constexpr std::int64_t kAxisMask = 0x1FFFFF;
constexpr int kAxisBits = 21;
constexpr std::int64_t kInvalidKey = -1;

inline std::int64_t encodeBins(std::int64_t ia, std::int64_t ie, std::int64_t ir)
{
  return ((ia & kAxisMask) << (2 * kAxisBits)) |
         ((ie & kAxisMask) << kAxisBits) |
         (ir & kAxisMask);
}

}  // namespace dust

sensor_msgs::msg::PointCloud2 buildCloudFromIndices(
  const sensor_msgs::msg::PointCloud2 & src, const std::vector<std::size_t> & indices)
{
  sensor_msgs::msg::PointCloud2 out;
  out.header = src.header;
  out.height = 1;
  out.width = static_cast<std::uint32_t>(indices.size());
  out.fields = src.fields;
  out.is_bigendian = src.is_bigendian;
  out.point_step = src.point_step;
  out.row_step = out.point_step * indices.size();
  out.is_dense = src.is_dense;
  out.data.resize(out.row_step);
  if (!indices.empty()) {
    const std::uint8_t * src_data = src.data.data();
    std::uint8_t * dst_data = out.data.data();
    for (std::size_t k = 0; k < indices.size(); ++k) {
      std::memcpy(dst_data + k * out.point_step,
                  src_data + indices[k] * src.point_step,
                  src.point_step);
    }
  }
  return out;
}

sensor_msgs::msg::PointCloud2 concatenateClouds(
  const sensor_msgs::msg::PointCloud2 & a, const sensor_msgs::msg::PointCloud2 & b)
{
  sensor_msgs::msg::PointCloud2 out;
  out.header = a.header;
  out.height = 1;
  out.width = a.width + b.width;
  out.fields = a.fields;
  out.is_bigendian = a.is_bigendian;
  out.point_step = a.point_step;
  out.row_step = out.point_step * out.width;
  out.is_dense = a.is_dense && b.is_dense;
  out.data.resize(out.row_step);
  if (!a.data.empty()) {
    std::memcpy(out.data.data(), a.data.data(), a.data.size());
  }
  if (!b.data.empty()) {
    std::memcpy(out.data.data() + a.data.size(), b.data.data(), b.data.size());
  }
  return out;
}
}  // namespace

namespace ground_segmentation
{
namespace
{
template <typename T>
bool get_param(const std::vector<rclcpp::Parameter> & p, const std::string & name, T & value)
{
  auto it = std::find_if(p.cbegin(), p.cend(), [&name](const rclcpp::Parameter & param) {
    return param.get_name() == name;
  });
  if (it == p.cend()) {
    return false;
  }
  value = it->template get_value<T>();
  return true;
}
}  // namespace

RANSACGroundFilterComponent::RANSACGroundFilterComponent(const rclcpp::NodeOptions & options)
: Node("ransac_ground_filter", options)
{
  // 雷达标识（front / rear），用于推导默认话题名与 frame_id
  const std::string lidar_name = declare_parameter<std::string>("lidar_name", "");
  const std::string prefix = lidar_name.empty() ? "" : ("/" + lidar_name);

  // 输入/输出话题：默认处理调平后的点云；lidar_name 为空时保持向后兼容
  const std::string default_input_topic =
    lidar_name.empty() ? "/front/rslidar_points" : (prefix + "/rslidar_points_leveled");
  const std::string input_topic = declare_parameter<std::string>("input_topic", default_input_topic);
  const std::string default_no_ground_filter_topic =
    lidar_name.empty() ? "~/no_ground_filter" : (prefix + "/no_ground_filter");
  const std::string no_ground_filter_topic =
    declare_parameter<std::string>("no_ground_filter_topic", default_no_ground_filter_topic);
  const std::string default_all_filter_topic =
    lidar_name.empty() ? "~/all_filter" : (prefix + "/all_filter");
  const std::string all_filter_topic =
    declare_parameter<std::string>("all_filter_topic", default_all_filter_topic);
  // 扬尘过滤前的非地面点云（调试观察用，不参与下游计算）
  const std::string default_no_ground_raw_topic =
    lidar_name.empty() ? "~/no_ground_raw" : (prefix + "/no_ground_raw");
  const std::string no_ground_raw_topic =
    declare_parameter<std::string>("no_ground_raw_topic", default_no_ground_raw_topic);

  // 默认 base_frame 与雷达坐标系一致（调平后点云仍位于雷达坐标系）
  const std::string default_base_frame =
    lidar_name.empty() ? "base_link" : (lidar_name + "_rslidar");
  base_frame_ = declare_parameter<std::string>("base_frame", default_base_frame);

  unit_axis_ = declare_parameter("unit_axis", "z");
  max_iterations_ = declare_parameter("max_iterations", 1000);
  min_inliers_ = declare_parameter("min_trial", 5000);
  min_points_ = declare_parameter("min_points", 1000);
  outlier_threshold_ = declare_parameter("outlier_threshold", 0.01);
  plane_slope_threshold_ = declare_parameter("plane_slope_threshold", 10.0);
  voxel_size_x_ = declare_parameter("voxel_size_x", 0.04);
  voxel_size_y_ = declare_parameter("voxel_size_y", 0.04);
  voxel_size_z_ = declare_parameter("voxel_size_z", 0.04);
  use_downsample_ = declare_parameter("use_downsample", true);
  use_adaptive_voxel_ = declare_parameter("use_adaptive_voxel", true);
  adaptive_voxel_ratio_ = declare_parameter("adaptive_voxel_ratio", 0.01);
  adaptive_voxel_max_size_ = declare_parameter("adaptive_voxel_max_size", 0.25);
  height_threshold_ = declare_parameter("height_threshold", 0.01);
  max_range_ = declare_parameter("max_range", 6.0);
  debug_ = declare_parameter("debug", false);

  // 扬尘过滤参数（单帧网格密度法：网格内点数 < dust_min_density_ 判为扬尘剔除）
  enable_dust_filter_ = declare_parameter("enable_dust_filter", true);
  dust_horiz_angle_res_deg_ = declare_parameter("dust_filter_horiz_angle_resolution", 0.2);
  dust_vert_angle_res_deg_ = declare_parameter("dust_filter_vert_angle_resolution", 0.2);
  dust_range_res_ = declare_parameter("dust_filter_range_resolution", 0.4);
  dust_range_growth_ = declare_parameter("dust_filter_range_growth", 0.02);
  dust_min_density_ = declare_parameter("dust_filter_min_density", 2);
  if (dust_min_density_ < 1) dust_min_density_ = 1;
  if (dust_range_res_ <= 0.0) dust_range_res_ = 0.1;
  if (dust_range_growth_ < 0.0) dust_range_growth_ = 0.0;

  // 扬尘网格可视化（调试）：在 RViz 里画稀疏网格线，每 stride 条画一条
  debug_dust_grid_ = declare_parameter("debug_dust_grid", false);
  debug_dust_grid_topic_ = declare_parameter("debug_dust_grid_topic", "~/debug/dust_grid");
  debug_dust_grid_stride_ = declare_parameter("debug_dust_grid_stride", 20);
  debug_dust_grid_range_min_ = declare_parameter("debug_dust_grid_range_min", 0.5);
  debug_dust_grid_range_max_ = declare_parameter("debug_dust_grid_range_max", 10.0);
  debug_dust_grid_azimuth_min_ = declare_parameter("debug_dust_grid_azimuth_min", -60.0);
  debug_dust_grid_azimuth_max_ = declare_parameter("debug_dust_grid_azimuth_max", 60.0);
  debug_dust_grid_elevation_min_ = declare_parameter("debug_dust_grid_elevation_min", -45.0);
  debug_dust_grid_elevation_max_ = declare_parameter("debug_dust_grid_elevation_max", 0.0);
  if (debug_dust_grid_stride_ < 1) debug_dust_grid_stride_ = 1;

  if (unit_axis_ == "x") {
    unit_vec_ = Eigen::Vector3d::UnitX();
  } else if (unit_axis_ == "y") {
    unit_vec_ = Eigen::Vector3d::UnitY();
  } else if (unit_axis_ == "z") {
    unit_vec_ = Eigen::Vector3d::UnitZ();
  } else {
    unit_vec_ = Eigen::Vector3d::UnitZ();
  }

  if (debug_) {
    setDebugPublisher();
  }

  // Setup TF
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  // Setup publishers and subscriber (subscription kept alive by node)
  pub_no_ground_filter_ =
    this->create_publisher<PointCloud2>(no_ground_filter_topic, rclcpp::QoS(10));
  pub_no_ground_raw_ =
    this->create_publisher<PointCloud2>(no_ground_raw_topic, rclcpp::QoS(10));
  pub_all_filter_ = this->create_publisher<PointCloud2>(all_filter_topic, rclcpp::QoS(10));

  if (debug_dust_grid_) {
    debug_dust_grid_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      debug_dust_grid_topic_, rclcpp::QoS(10));
    dust_grid_timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&RANSACGroundFilterComponent::publishDustGrid, this));
  }

  using std::placeholders::_1;
  sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic, rclcpp::SensorDataQoS(),
    std::bind(&RANSACGroundFilterComponent::onPointCloud, this, _1));

  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&RANSACGroundFilterComponent::paramCallback, this, _1));

  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
}

void RANSACGroundFilterComponent::setDebugPublisher()
{
  if (is_initialized_debug_message_) {
    return;
  }
  debug_pose_array_pub_ =
    create_publisher<geometry_msgs::msg::PoseArray>("~/debug/plane_pose_array", 10);
  debug_ground_cloud_pub_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("~/debug/ground/pointcloud", 10);
  is_initialized_debug_message_ = true;
}

void RANSACGroundFilterComponent::publishDebugMessage(
  const geometry_msgs::msg::PoseArray & debug_pose_array,
  const pcl::PointCloud<PointType> & ground_cloud, const std_msgs::msg::Header & header)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_ground_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  for (const auto & ground_point : ground_cloud.points) {
    pcl::PointXYZRGB p;
    p.x = ground_point.x;
    p.y = ground_point.y;
    p.z = ground_point.z;
    p.r = 255.0;
    p.g = 0.0;
    p.b = 0.0;
    colored_ground_ptr->points.push_back(p);
  }
  sensor_msgs::msg::PointCloud2::SharedPtr ground_cloud_msg_ptr(new sensor_msgs::msg::PointCloud2);
  ground_cloud_msg_ptr->header = header;
  pcl::toROSMsg(*colored_ground_ptr, *ground_cloud_msg_ptr);
  ground_cloud_msg_ptr->header.frame_id = base_frame_;
  debug_pose_array_pub_->publish(debug_pose_array);
  debug_ground_cloud_pub_->publish(*ground_cloud_msg_ptr);
}

void RANSACGroundFilterComponent::publishDustGrid()
{
  if (!debug_dust_grid_pub_) {
    return;
  }

  visualization_msgs::msg::MarkerArray array;

  visualization_msgs::msg::Marker clear;
  clear.header.stamp = this->now();
  clear.header.frame_id = base_frame_;
  clear.ns = "dust_grid";
  clear.id = 0;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  array.markers.push_back(clear);

  constexpr double kDeg2Rad = M_PI / 180.0;
  const double az_min = debug_dust_grid_azimuth_min_ * kDeg2Rad;
  const double az_max = debug_dust_grid_azimuth_max_ * kDeg2Rad;
  const double el_min = debug_dust_grid_elevation_min_ * kDeg2Rad;
  const double el_max = debug_dust_grid_elevation_max_ * kDeg2Rad;
  const double horiz_res = dust_horiz_angle_res_deg_ * kDeg2Rad;
  const double vert_res = dust_vert_angle_res_deg_ * kDeg2Rad;
  const double r_min = debug_dust_grid_range_min_;
  const double r_max = debug_dust_grid_range_max_;

  int id = 1;

  // 1) 水平角方向：每隔 stride 个水平角 bin 画一条径向线（水平面 z=0）
  {
    const int n_az = std::max(0, static_cast<int>(std::floor((az_max - az_min) / horiz_res)));
    for (int i = 0; i <= n_az; i += debug_dust_grid_stride_) {
      const double az = az_min + i * horiz_res;
      visualization_msgs::msg::Marker m;
      m.header = clear.header;
      m.ns = "dust_grid";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.scale.x = 0.03;
      m.color.r = 1.0f; m.color.g = 0.0f; m.color.b = 0.0f; m.color.a = 1.0f;
      geometry_msgs::msg::Point p0, p1;
      p0.x = r_min * std::cos(az); p0.y = r_min * std::sin(az); p0.z = 0.0;
      p1.x = r_max * std::cos(az); p1.y = r_max * std::sin(az); p1.z = 0.0;
      m.points.push_back(p0);
      m.points.push_back(p1);
      array.markers.push_back(m);
    }
  }

  // 2) 垂直角方向：每隔 stride 个垂直角 bin 画一条径向线（方位角=0 的竖直面）
  {
    const int n_el = std::max(0, static_cast<int>(std::floor((el_max - el_min) / vert_res)));
    for (int j = 0; j <= n_el; j += debug_dust_grid_stride_) {
      const double el = el_min + j * vert_res;
      visualization_msgs::msg::Marker m;
      m.header = clear.header;
      m.ns = "dust_grid";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.scale.x = 0.03;
      m.color.r = 0.0f; m.color.g = 1.0f; m.color.b = 0.0f; m.color.a = 1.0f;
      geometry_msgs::msg::Point p0, p1;
      p0.x = r_min * std::cos(el); p0.y = 0.0; p0.z = r_min * std::sin(el);
      p1.x = r_max * std::cos(el); p1.y = 0.0; p1.z = r_max * std::sin(el);
      m.points.push_back(p0);
      m.points.push_back(p1);
      array.markers.push_back(m);
    }
  }

  // 3) 水平距离方向：按真实（线性渐变）分箱边界画完整闭合圆环（z=0，360°）
  {
    constexpr int kArcPoints = 128;
    int k = 1;
    while (true) {
      const double denom = 1.0 - k * dust_range_growth_;
      if (denom <= 1e-6) {
        break;
      }
      const double r = k * dust_range_res_ / denom;
      if (r > r_max + 1e-6) {
        break;
      }
      if (r < r_min - 1e-6) {
        ++k;
        continue;
      }
      visualization_msgs::msg::Marker m;
      m.header = clear.header;
      m.ns = "dust_grid";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.scale.x = 0.03;
      m.color.r = 0.0f; m.color.g = 0.0f; m.color.b = 1.0f; m.color.a = 1.0f;
      for (int a = 0; a <= kArcPoints; ++a) {
        const double az = 2.0 * M_PI * a / kArcPoints;
        geometry_msgs::msg::Point p;
        p.x = r * std::cos(az);
        p.y = r * std::sin(az);
        p.z = 0.0;
        m.points.push_back(p);
      }
      array.markers.push_back(m);
      ++k;
    }
  }

  debug_dust_grid_pub_->publish(array);
}

bool RANSACGroundFilterComponent::transformPointCloud(
  const std::string & in_target_frame, const PointCloud2ConstPtr & in_cloud_ptr,
  const PointCloud2::SharedPtr & out_cloud_ptr)
{
  if (in_target_frame == in_cloud_ptr->header.frame_id) {
    *out_cloud_ptr = *in_cloud_ptr;
    return true;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_->lookupTransform(
      in_target_frame, in_cloud_ptr->header.frame_id, in_cloud_ptr->header.stamp,
      rclcpp::Duration::from_seconds(1.0));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    return false;
  }
  // tf2::doTransform(*in_cloud_ptr, *out_cloud_ptr, transform_stamped);
  Eigen::Matrix4f mat = tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
  pcl::PointCloud<PointType> pcl_cloud;
  pcl::fromROSMsg(*in_cloud_ptr, pcl_cloud);
  pcl::PointCloud<PointType> pcl_transformed;
  pcl::transformPointCloud(pcl_cloud, pcl_transformed, mat);
  pcl::toROSMsg(pcl_transformed, *out_cloud_ptr);
  out_cloud_ptr->header.frame_id = in_target_frame;
  return true;
}

void RANSACGroundFilterComponent::extractPointsIndices(
  const pcl::PointCloud<PointType>::Ptr in_cloud_ptr, const pcl::PointIndices & in_indices,
  pcl::PointCloud<PointType>::Ptr out_only_indices_cloud_ptr,
  pcl::PointCloud<PointType>::Ptr out_removed_indices_cloud_ptr)
{
  pcl::ExtractIndices<PointType> extract_ground;
  extract_ground.setInputCloud(in_cloud_ptr);
  extract_ground.setIndices(pcl::make_shared<pcl::PointIndices>(in_indices));

  extract_ground.setNegative(false);  // true removes the indices, false leaves only the indices
  extract_ground.filter(*out_only_indices_cloud_ptr);

  extract_ground.setNegative(true);  // true removes the indices, false leaves only the indices
  extract_ground.filter(*out_removed_indices_cloud_ptr);
}

Eigen::Affine3d RANSACGroundFilterComponent::getPlaneAffine(
  const pcl::PointCloud<PointType> segment_ground_cloud, const Eigen::Vector3d & plane_normal)
{
  pcl::CentroidPoint<pcl::PointXYZ> centroid;
  for (const auto p : segment_ground_cloud.points) {
    centroid.add(p);
  }
  pcl::PointXYZ centroid_point;
  centroid.get(centroid_point);
  Eigen::Translation<double, 3> trans(centroid_point.x, centroid_point.y, centroid_point.z);
  const ground_segmentation::PlaneBasis basis = getPlaneBasis(plane_normal);
  Eigen::Matrix3d rot;
  rot << basis.e_x.x(), basis.e_y.x(), basis.e_z.x(), basis.e_x.y(), basis.e_y.y(), basis.e_z.y(),
    basis.e_x.z(), basis.e_y.z(), basis.e_z.z();
  return trans * rot;
}

void RANSACGroundFilterComponent::applyRANSAC(
  const pcl::PointCloud<PointType>::Ptr & input, pcl::PointIndices::Ptr & output_inliers,
  pcl::ModelCoefficients::Ptr & output_coefficients)
{
  pcl::SACSegmentation<PointType> seg;
  seg.setOptimizeCoefficients(true);
  seg.setRadiusLimits(0.3, std::numeric_limits<double>::max());
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(outlier_threshold_);
  seg.setInputCloud(input);
  seg.setMaxIterations(max_iterations_);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.segment(*output_inliers, *output_coefficients);
}

void RANSACGroundFilterComponent::onPointCloud(const sensor_msgs::msg::PointCloud2 & input)
{
  std::scoped_lock lock(mutex_);
  PointCloud2::SharedPtr no_ground_filtered(new PointCloud2);
  PointCloud2::SharedPtr no_ground_raw(new PointCloud2);
  PointCloud2::SharedPtr all_filtered(new PointCloud2);
  auto input_ptr = std::make_shared<const sensor_msgs::msg::PointCloud2>(input);
  if (!filter(input_ptr, *no_ground_raw, *no_ground_filtered, *all_filtered)) {
    return;
  }
  pub_no_ground_raw_->publish(*no_ground_raw);
  pub_no_ground_filter_->publish(*no_ground_filtered);
  pub_all_filter_->publish(*all_filtered);
}

bool RANSACGroundFilterComponent::filter(
  const PointCloud2::ConstSharedPtr & input,
  PointCloud2 & no_ground_raw_output,
  PointCloud2 & no_ground_filter_output,
  PointCloud2 & all_filter_output)
{
  sensor_msgs::msg::PointCloud2::SharedPtr input_transformed_ptr(new sensor_msgs::msg::PointCloud2);
  if (!transformPointCloud(base_frame_, input, input_transformed_ptr)) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "Failed transform from " << base_frame_ << " to " << input->header.frame_id);
    return false;
  }

  // RANSAC 平面拟合使用“水平距离 ≤ max_range_ 且(可选)降采样”的点，以降低计算量；
  // 但最终的地面/非地面划分覆盖全部点（不删除任何点）。
  pcl::PointCloud<PointType>::Ptr current_sensor_cloud_ptr(new pcl::PointCloud<PointType>);
  pcl::fromROSMsg(*input_transformed_ptr, *current_sensor_cloud_ptr);

  pcl::PointCloud<PointType>::Ptr ranged_cloud(new pcl::PointCloud<PointType>);
  ranged_cloud->points.reserve(current_sensor_cloud_ptr->points.size());
  const double max_range_squared = max_range_ * max_range_;
  for (const auto & p : current_sensor_cloud_ptr->points) {
    if (p.x * p.x + p.y * p.y <= max_range_squared) {
      ranged_cloud->points.push_back(p);
    }
  }
  ranged_cloud->width = ranged_cloud->points.size();
  ranged_cloud->height = 1;
  ranged_cloud->is_dense = true;

  // downsample pointcloud to reduce ransac calculation cost
  pcl::PointCloud<PointType>::Ptr downsampled_cloud(new pcl::PointCloud<PointType>);
  if (use_downsample_) {
    downsampled_cloud->points.reserve(ranged_cloud->points.size());

    if (use_adaptive_voxel_) {
      // 距离自适应体素降采样：体素边长随水平距离增大，
      // 使近处与远处的点密度趋于一致，避免近处点过密主导 RANSAC 平面拟合。
      std::unordered_map<VoxelKey, int, VoxelKeyHash> voxel_map;
      voxel_map.reserve(ranged_cloud->points.size());
      for (std::size_t i = 0; i < ranged_cloud->points.size(); ++i) {
        const auto & p = ranged_cloud->points[i];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
          continue;
        }
        const double r = std::sqrt(p.x * p.x + p.y * p.y);
        const double grow = adaptive_voxel_ratio_ * r;
        double sx = voxel_size_x_ + grow;
        double sy = voxel_size_y_ + grow;
        double sz = voxel_size_z_ + grow;
        if (adaptive_voxel_max_size_ > 0.0) {
          sx = std::min(sx, adaptive_voxel_max_size_);
          sy = std::min(sy, adaptive_voxel_max_size_);
          sz = std::min(sz, adaptive_voxel_max_size_);
        }
        const auto ix = static_cast<std::int64_t>(std::floor(p.x / sx));
        const auto iy = static_cast<std::int64_t>(std::floor(p.y / sy));
        const auto iz = static_cast<std::int64_t>(std::floor(p.z / sz));
        const VoxelKey key{ix, iy, iz};
        // 保留离体素中心最近的点
        const double cx = static_cast<double>(ix) * sx;
        const double cy = static_cast<double>(iy) * sy;
        const double cz = static_cast<double>(iz) * sz;
        const double dx = p.x - cx;
        const double dy = p.y - cy;
        const double dz = p.z - cz;
        const double dist2 = dx * dx + dy * dy + dz * dz;
        auto it = voxel_map.find(key);
        if (it == voxel_map.end()) {
          voxel_map.emplace(key, static_cast<int>(i));
        } else {
          const auto & q = ranged_cloud->points[it->second];
          const double qx = q.x - cx;
          const double qy = q.y - cy;
          const double qz = q.z - cz;
          if (dist2 < qx * qx + qy * qy + qz * qz) {
            it->second = static_cast<int>(i);
          }
        }
      }
      for (const auto & kv : voxel_map) {
        downsampled_cloud->points.push_back(ranged_cloud->points[kv.second]);
      }
    } else {
      pcl::VoxelGrid<PointType> filter;
      filter.setInputCloud(ranged_cloud);
      filter.setLeafSize(voxel_size_x_, voxel_size_y_, voxel_size_z_);
      filter.filter(*downsampled_cloud);
    }

    downsampled_cloud->width = downsampled_cloud->points.size();
    downsampled_cloud->height = 1;
    downsampled_cloud->is_dense = true;
  } else {
    // 彻底跳过降采样：直接使用全分辨率点云（ranged_cloud）做 RANSAC 平面拟合
    downsampled_cloud = ranged_cloud;
  }

  // apply ransac
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  applyRANSAC(downsampled_cloud, inliers, coefficients);

  if (coefficients->values.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "failed to find a plane");
    // 找不到平面：无法划分地面，把整帧作为非地面（保底，不删点）
    no_ground_raw_output = *input_transformed_ptr;
    no_ground_filter_output = *input_transformed_ptr;
    all_filter_output = *input_transformed_ptr;
    return true;
  }

  // filter too tilt plane to avoid mis-fitting (e.g. fitting to wall plane)
  Eigen::Vector3d plane_normal(
    coefficients->values[0], coefficients->values[1], coefficients->values[2]);
  {
    const auto plane_slope = std::abs(
      std::acos(plane_normal.dot(unit_vec_) / (plane_normal.norm() * unit_vec_.norm())) * 180 /
      M_PI);
    if (plane_slope > plane_slope_threshold_) {
      // 平面过倾：不误删，整帧作为非地面
      no_ground_raw_output = *input_transformed_ptr;
      no_ground_filter_output = *input_transformed_ptr;
      all_filter_output = *input_transformed_ptr;
      return true;
    }
  }

  // extract pointcloud from indices（仅用于求平面质心与调试地面点云）
  pcl::PointCloud<PointType>::Ptr segment_ground_cloud_ptr(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr segment_no_ground_cloud_ptr(new pcl::PointCloud<PointType>);
  extractPointsIndices(
    downsampled_cloud, *inliers, segment_ground_cloud_ptr, segment_no_ground_cloud_ptr);
  const Eigen::Affine3d plane_affine = getPlaneAffine(*segment_ground_cloud_ptr, plane_normal);
  const Eigen::Affine3d plane_affine_inv = plane_affine.inverse();

  // 对“调平后原始点云”逐点划分地面/非地面（全分辨率、逐字节保留字段、不删除任何点）：
  //   - 有限点且水平距离 ≤ max_range_：按 RANSAC 平面距离判定，|z'| ≤ height_threshold_ → 地面；
  //   - 水平距离 > max_range_ 或非有限点：归入非地面（保底）。
  sensor_msgs::PointCloud2ConstIterator<float> it_x(*input_transformed_ptr, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(*input_transformed_ptr, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(*input_transformed_ptr, "z");

  std::vector<size_t> ground_idx;
  std::vector<size_t> no_ground_idx;
  const size_t num_points =
    static_cast<size_t>(input_transformed_ptr->width) * input_transformed_ptr->height;
  ground_idx.reserve(num_points);
  no_ground_idx.reserve(num_points);

  size_t i = 0;
  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++i) {
    const float x = *it_x, y = *it_y, z = *it_z;
    bool is_ground = false;
    if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z) &&
      x * x + y * y <= max_range_squared)
    {
      const Eigen::Vector3d transformed_point = plane_affine_inv * Eigen::Vector3d(x, y, z);
      is_ground = std::abs(transformed_point.z()) <= height_threshold_;
    }
    if (is_ground) {
      ground_idx.push_back(i);
    } else {
      no_ground_idx.push_back(i);
    }
  }

  // 地面点云 / 非地面点云（全分辨率，保留 intensity 等原始字段）
  sensor_msgs::msg::PointCloud2 ground_cloud =
    buildCloudFromIndices(*input_transformed_ptr, ground_idx);
  sensor_msgs::msg::PointCloud2 no_ground_cloud =
    buildCloudFromIndices(*input_transformed_ptr, no_ground_idx);

  // 扬尘过滤前的非地面点云：单独发布（调试观察用，供对比过滤效果）
  no_ground_raw_output = no_ground_cloud;

  // 对非地面点云做扬尘过滤
  sensor_msgs::msg::PointCloud2 no_ground_filtered;
  if (enable_dust_filter_) {
    applyDustFilter(no_ground_cloud, no_ground_filtered);
  } else {
    no_ground_filtered = no_ground_cloud;
  }

  // 合并：地面点云 + 过滤后的非地面点云（同一帧、同一坐标系、字段一致）
  all_filter_output = concatenateClouds(ground_cloud, no_ground_filtered);

  no_ground_filter_output = no_ground_filtered;
  no_ground_filter_output.header.frame_id = base_frame_;
  all_filter_output.header.frame_id = base_frame_;

  // output debug plane coords and ground pointcloud when debug flag is set
  if (debug_) {
    const geometry_msgs::msg::Pose debug_pose = getDebugPose(plane_affine);
    geometry_msgs::msg::PoseArray debug_pose_array;
    debug_pose_array.header.frame_id = base_frame_;
    debug_pose_array.poses.push_back(debug_pose);
    publishDebugMessage(debug_pose_array, *segment_ground_cloud_ptr, input->header);
  }

  return true;
}

void RANSACGroundFilterComponent::applyDustFilter(
  const sensor_msgs::msg::PointCloud2 & input, sensor_msgs::msg::PointCloud2 & output)
{
  constexpr double kDeg2Rad = 0.017453292519943295;  // π/180
  const double horiz_angle_res = dust_horiz_angle_res_deg_ * kDeg2Rad;
  const double vert_angle_res = dust_vert_angle_res_deg_ * kDeg2Rad;

  // 单帧网格密度法：统计每个球坐标 bin 内的点数，
  // 点数 < dust_min_density_ 的网格判为扬尘，剔除该网格内全部点。
  sensor_msgs::PointCloud2ConstIterator<float> it_x(input, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(input, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(input, "z");

  const size_t num_points = static_cast<size_t>(input.width) * input.height;
  std::vector<std::int64_t> point_keys(num_points, dust::kInvalidKey);
  std::unordered_map<std::int64_t, std::uint32_t> bin_count;
  bin_count.reserve(num_points);

  // 第一遍：计算每个点的球坐标 bin，并累计每个 bin 的点数
  size_t i = 0;
  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++i) {
    const float x = *it_x, y = *it_y, z = *it_z;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;  // 非有限点不参与统计，第二遍原样保留
    }

    const double range = std::sqrt(x * x + y * y);
    const double azimuth = std::atan2(y, x);
    const double elevation = std::atan2(z, range);
    // 距离自适应：bin 宽随距离线性渐变（每 1m 增加 dust_range_growth_ m）
    const double eff_range_res = dust_range_res_ + dust_range_growth_ * range;

    const std::int64_t ia = static_cast<std::int64_t>(std::floor(azimuth / horiz_angle_res));
    const std::int64_t ie = static_cast<std::int64_t>(std::floor(elevation / vert_angle_res));
    const std::int64_t ir = static_cast<std::int64_t>(std::floor(range / eff_range_res));
    const std::int64_t key = dust::encodeBins(ia, ie, ir);
    point_keys[i] = key;
    ++bin_count[key];
  }

  // 第二遍：密度低于阈值的网格判为扬尘，剔除；非有限点原样保留
  std::vector<std::size_t> keep_idx;
  keep_idx.reserve(num_points);
  for (size_t j = 0; j < num_points; ++j) {
    const std::int64_t key = point_keys[j];
    if (key == dust::kInvalidKey) {
      keep_idx.push_back(j);
      continue;
    }
    const auto it = bin_count.find(key);
    if (it != bin_count.end() &&
      it->second >= static_cast<std::uint32_t>(dust_min_density_))
    {
      keep_idx.push_back(j);
    }
  }

  // 输出过滤后的点云（逐字节拷贝，完整保留 intensity/ring/time 等字段）
  output = buildCloudFromIndices(input, keep_idx);
  output.header = input.header;
}

rcl_interfaces::msg::SetParametersResult RANSACGroundFilterComponent::paramCallback(
  const std::vector<rclcpp::Parameter> & p)
{
  std::scoped_lock lock(mutex_);

  if (get_param(p, "base_frame", base_frame_)) {
    RCLCPP_DEBUG(get_logger(), "Setting base_frame to: %s.", base_frame_.c_str());
  }
  if (get_param(p, "max_iterations", max_iterations_)) {
    RCLCPP_DEBUG(get_logger(), "Setting max_iterations to: %d.", max_iterations_);
  }
  if (get_param(p, "min_inliers", min_inliers_)) {
    RCLCPP_DEBUG(get_logger(), "Setting min_inliers to: %d.", min_inliers_);
  }
  if (get_param(p, "min_points", min_points_)) {
    RCLCPP_DEBUG(get_logger(), "Setting min_points to: %d.", min_points_);
  }
  if (get_param(p, "outlier_threshold", outlier_threshold_)) {
    RCLCPP_DEBUG(get_logger(), "Setting outlier_threshold to: %lf.", outlier_threshold_);
  }
  if (get_param(p, "height_threshold", height_threshold_)) {
    RCLCPP_DEBUG(get_logger(), "Setting height_threshold_ to: %lf.", height_threshold_);
  }
  if (get_param(p, "max_range", max_range_)) {
    RCLCPP_DEBUG(get_logger(), "Setting max_range to: %lf.", max_range_);
  }
  if (get_param(p, "plane_slope_threshold", plane_slope_threshold_)) {
    RCLCPP_DEBUG(get_logger(), "Setting plane_slope_threshold to: %lf.", plane_slope_threshold_);
  }
  if (get_param(p, "voxel_size_x", voxel_size_x_)) {
    RCLCPP_DEBUG(get_logger(), "Setting voxel_size_x to: %lf.", voxel_size_x_);
  }
  if (get_param(p, "voxel_size_y", voxel_size_y_)) {
    RCLCPP_DEBUG(get_logger(), "Setting voxel_size_y to: %lf.", voxel_size_y_);
  }
  if (get_param(p, "voxel_size_z", voxel_size_z_)) {
    RCLCPP_DEBUG(get_logger(), "Setting voxel_size_z to: %lf.", voxel_size_z_);
  }
  if (get_param(p, "use_downsample", use_downsample_)) {
    RCLCPP_DEBUG(get_logger(), "Setting use_downsample to: %d.", use_downsample_);
  }
  if (get_param(p, "use_adaptive_voxel", use_adaptive_voxel_)) {
    RCLCPP_DEBUG(get_logger(), "Setting use_adaptive_voxel to: %d.", use_adaptive_voxel_);
  }
  if (get_param(p, "adaptive_voxel_ratio", adaptive_voxel_ratio_)) {
    RCLCPP_DEBUG(get_logger(), "Setting adaptive_voxel_ratio to: %lf.", adaptive_voxel_ratio_);
  }
  if (get_param(p, "adaptive_voxel_max_size", adaptive_voxel_max_size_)) {
    RCLCPP_DEBUG(get_logger(), "Setting adaptive_voxel_max_size to: %lf.", adaptive_voxel_max_size_);
  }

  {
    bool dust_changed = false;
    if (get_param(p, "enable_dust_filter", enable_dust_filter_)) {
      RCLCPP_DEBUG(get_logger(), "Setting enable_dust_filter to: %d.", enable_dust_filter_);
    }
    if (get_param(p, "dust_filter_horiz_angle_resolution", dust_horiz_angle_res_deg_)) {
      RCLCPP_DEBUG(
        get_logger(), "Setting dust_filter_horiz_angle_resolution to: %lf.",
        dust_horiz_angle_res_deg_);
      dust_changed = true;
    }
    if (get_param(p, "dust_filter_vert_angle_resolution", dust_vert_angle_res_deg_)) {
      RCLCPP_DEBUG(
        get_logger(), "Setting dust_filter_vert_angle_resolution to: %lf.",
        dust_vert_angle_res_deg_);
      dust_changed = true;
    }
    if (get_param(p, "dust_filter_range_resolution", dust_range_res_)) {
      RCLCPP_DEBUG(get_logger(), "Setting dust_filter_range_resolution to: %lf.", dust_range_res_);
      dust_changed = true;
    }
    if (get_param(p, "dust_filter_range_growth", dust_range_growth_)) {
      RCLCPP_DEBUG(get_logger(), "Setting dust_filter_range_growth to: %lf.", dust_range_growth_);
      dust_changed = true;
    }
    if (get_param(p, "dust_filter_min_density", dust_min_density_)) {
      RCLCPP_DEBUG(get_logger(), "Setting dust_filter_min_density to: %d.", dust_min_density_);
    }

    if (dust_changed) {
      if (dust_range_res_ <= 0.0) dust_range_res_ = 0.1;
      if (dust_range_growth_ < 0.0) dust_range_growth_ = 0.0;
    }
    if (dust_min_density_ < 1) dust_min_density_ = 1;
  }

  if (get_param(p, "debug", debug_)) {
    RCLCPP_DEBUG(get_logger(), "Setting debug to: %d.", debug_);
  }

  if (debug_) {
    setDebugPublisher();
  }

  if (get_param(p, "unit_axis", unit_axis_)) {
    if (unit_axis_ == "x") {
      unit_vec_ = Eigen::Vector3d::UnitX();
    } else if (unit_axis_ == "y") {
      unit_vec_ = Eigen::Vector3d::UnitY();
    } else if (unit_axis_ == "z") {
      unit_vec_ = Eigen::Vector3d::UnitZ();
    } else {
      unit_vec_ = Eigen::Vector3d::UnitZ();
    }
    RCLCPP_DEBUG(get_logger(), "Setting unit_axis to: %s.", unit_axis_.c_str());
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}

}  // namespace ground_segmentation

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ground_segmentation::RANSACGroundFilterComponent)
