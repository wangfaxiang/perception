/**
 * @file range_image_segmentation.cpp
 * @brief 基于距离图像(range image)的地面分割 + 非地面障碍物聚类
 *
 * 方法参考 LeGO-LOAM imageProjection.cpp 的 projectPointCloud / groundRemoval /
 * cloudSegmentation，并针对速腾聚创 E1R 全固态雷达做适配:
 *   - E1R 为 120°(水平) x 90°(垂直) 前视雷达，非 360° 环绕，故列映射
 *     改为前视方位角区间，分割时水平边界不做 wrap-around。
 *   - E1R 输出 144 线规则结构点云，但 lidar_leveling 输出的调平点云
 *     为 PointXYZI（无 ring 字段），因此行号由俯仰角 asin(z/r) 反算，
 *     列号由方位角 atan2(y, x) 反算。
 *
 * 输入 : 调平点云 /<lidar_name>/rslidar_points_leveled
 * 输出 : 地面点云      /<lidar_name>/ground_cloud
 *        聚类点云      /<lidar_name>/segmented_cloud_pure (intensity=簇号)
 *        聚类包围盒    /<lidar_name>/box/cluster/range_image (box_msg::Boxs)
 *        包围盒 Marker /<lidar_name>/box/cluster/range_image/marker
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <box_msg/msg/box.hpp>
#include <box_msg/msg/boxs.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <queue>
#include <string>
#include <vector>

namespace
{
constexpr double kDeg2Rad = M_PI / 180.0;
constexpr int kInvalidLabel = 999999;
}  // namespace

class RangeImageSegmentation : public rclcpp::Node
{
public:
  RangeImageSegmentation()
  : Node("range_image_segmentation")
  {
    const std::string lidar_name = declare_parameter<std::string>("lidar_name", "");
    const std::string prefix = lidar_name.empty() ? "" : ("/" + lidar_name);

    const std::string input_topic =
      declare_parameter<std::string>("input_topic", prefix + "/rslidar_points_leveled");
    const std::string ground_topic =
      declare_parameter<std::string>("ground_topic", prefix + "/ground_cloud");
    const std::string segmented_topic =
      declare_parameter<std::string>("segmented_topic", prefix + "/segmented_cloud_pure");
    const std::string boxs_topic =
      declare_parameter<std::string>("boxs_topic", prefix + "/box/cluster/range_image");
    const std::string marker_topic =
      declare_parameter<std::string>("marker_topic", prefix + "/box/cluster/range_image/marker");

    // ---- 投影参数 ---- //
    num_vertical_scans_ = declare_parameter<int>("num_vertical_scans", 144);
    num_horizontal_scans_ = declare_parameter<int>("num_horizontal_scans", 180);
    vertical_angle_bottom_ = declare_parameter<double>("vertical_angle_bottom", -75.0);
    vertical_angle_top_ = declare_parameter<double>("vertical_angle_top", 15.0);
    horizontal_fov_ = declare_parameter<double>("horizontal_fov", 120.0);
    min_range_ = declare_parameter<double>("min_range", 0.2);

    // ---- 地面分割参数 ---- //
    ground_scan_index_ = declare_parameter<int>("ground_scan_index", 100);
    sensor_mount_angle_ = declare_parameter<double>("sensor_mount_angle", 0.0);
    ground_angle_threshold_ = declare_parameter<double>("ground_angle_threshold", 10.0);
    ground_z_threshold_ = declare_parameter<double>("ground_z_threshold", -1.1);

    // ---- 分割参数 ---- //
    segment_theta_ = declare_parameter<double>("segment_theta", 60.0);
    segment_valid_point_num_ = declare_parameter<int>("segment_valid_point_num", 5);
    segment_valid_line_num_ = declare_parameter<int>("segment_valid_line_num", 3);
    min_cluster_size_ = declare_parameter<int>("min_cluster_size", 30);

    if (ground_scan_index_ >= num_vertical_scans_) {
      ground_scan_index_ = num_vertical_scans_ - 1;
    }

    size_ = static_cast<size_t>(num_vertical_scans_) * num_horizontal_scans_;
    ang_res_x_ = horizontal_fov_ * kDeg2Rad / num_horizontal_scans_;
    ang_res_y_ =
      (vertical_angle_top_ - vertical_angle_bottom_) * kDeg2Rad / (num_vertical_scans_ - 1);
    ang_bottom_ = vertical_angle_bottom_ * kDeg2Rad;
    half_fov_ = horizontal_fov_ * kDeg2Rad / 2.0;
    sensor_mount_angle_rad_ = sensor_mount_angle_ * kDeg2Rad;
    ground_angle_threshold_rad_ = ground_angle_threshold_ * kDeg2Rad;
    segment_theta_tan_ = std::tan(segment_theta_ * kDeg2Rad);

    nan_point_.x = nan_point_.y = nan_point_.z = std::numeric_limits<float>::quiet_NaN();
    nan_point_.intensity = -1.0f;

    full_cloud_.assign(size_, nan_point_);
    range_mat_.assign(size_, std::numeric_limits<float>::max());
    ground_mat_.assign(size_, 0);
    label_mat_.assign(size_, 0);

    ground_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    segmented_pure_.reset(new pcl::PointCloud<pcl::PointXYZI>());

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::SensorDataQoS(),
      std::bind(&RangeImageSegmentation::onCloud, this, std::placeholders::_1));
    pub_ground_ = create_publisher<sensor_msgs::msg::PointCloud2>(ground_topic, rclcpp::QoS(10));
    pub_segmented_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(segmented_topic, rclcpp::QoS(10));
    pub_boxs_ = create_publisher<box_msg::msg::Boxs>(boxs_topic, rclcpp::QoS(10));
    pub_marker_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic, rclcpp::QoS(10));

    RCLCPP_INFO(
      get_logger(),
      "range_image_segmentation started (lidar_name=%s): input=%s, grid=%dx%d, "
      "fov=%.1f deg, vertical=[%.1f, %.1f] deg, ground_scan_index=%d",
      lidar_name.c_str(), input_topic.c_str(), num_vertical_scans_, num_horizontal_scans_,
      horizontal_fov_, vertical_angle_bottom_, vertical_angle_top_, ground_scan_index_);
  }

private:
  void resetMatrices()
  {
    range_mat_.assign(size_, std::numeric_limits<float>::max());
    ground_mat_.assign(size_, 0);
    label_mat_.assign(size_, 0);
    std::fill(full_cloud_.begin(), full_cloud_.end(), nan_point_);
    ground_cloud_->clear();
    segmented_pure_->clear();
    label_count_ = 1;
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *cloud);

    // 去除无效点（NaN/Inf）
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_clean(new pcl::PointCloud<pcl::PointXYZI>());
    cloud_clean->reserve(cloud->size());
    for (const auto & p : cloud->points) {
      if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
        cloud_clean->push_back(p);
      }
    }

    resetMatrices();
    laser_cloud_in_ = cloud_clean;

    projectPointCloud();
    groundRemoval();
    cloudSegmentation();
    publishClouds(msg->header);
  }

  void projectPointCloud()
  {
    for (const auto & pt : laser_cloud_in_->points) {
      const float range = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
      if (range < min_range_) {
        continue;
      }

      // 行号：俯仰角 asin(z / r)
      const float vertical_angle = std::asin(pt.z / range);
      const int row = static_cast<int>(std::round((vertical_angle - ang_bottom_) / ang_res_y_));
      if (row < 0 || row >= num_vertical_scans_) {
        continue;
      }

      // 列号：方位角 atan2(y, x)，前视 120°，左右各 60°
      const float azimuth = std::atan2(pt.y, pt.x);
      if (azimuth < -half_fov_ || azimuth > half_fov_) {
        continue;
      }
      int col = static_cast<int>(std::round((azimuth + half_fov_) / ang_res_x_));
      col = std::clamp(col, 0, num_horizontal_scans_ - 1);

      const size_t index = col + row * num_horizontal_scans_;
      range_mat_[index] = range;

      pcl::PointXYZI p = pt;
      // 保留 row/col 信息（与 LeGO-LOAM 一致，便于调试）
      p.intensity = static_cast<float>(row) + static_cast<float>(col) / 10000.0f;
      full_cloud_[index] = p;
    }
  }

  void groundRemoval()
  {
    const float max_float = std::numeric_limits<float>::max();

    // 逐列、逐相邻行判断地面
    for (int col = 0; col < num_horizontal_scans_; ++col) {
      for (int row = 0; row < ground_scan_index_; ++row) {
        const size_t lower = col + row * num_horizontal_scans_;
        const size_t upper = col + (row + 1) * num_horizontal_scans_;

        if (range_mat_[lower] == max_float || range_mat_[upper] == max_float) {
          ground_mat_[lower] = -1;
          continue;
        }

        const float dX = full_cloud_[upper].x - full_cloud_[lower].x;
        const float dY = full_cloud_[upper].y - full_cloud_[lower].y;
        const float dZ = full_cloud_[upper].z - full_cloud_[lower].z;
        const float vertical_angle = std::atan2(dZ, std::sqrt(dX * dX + dY * dY + dZ * dZ));

        // 绝对高度约束：只有 z 低于阈值的点才可作为地面候选，避免高处平坦物误判
        const bool below_z_threshold =
          full_cloud_[lower].z < ground_z_threshold_ &&
          full_cloud_[upper].z < ground_z_threshold_;

        if (below_z_threshold &&
          std::fabs(vertical_angle - sensor_mount_angle_rad_) <= ground_angle_threshold_rad_) {
          ground_mat_[lower] = 1;
          ground_mat_[upper] = 1;
        }
      }
    }

    // 地面点和无效点不再参与后续分割
    for (size_t i = 0; i < size_; ++i) {
      if (ground_mat_[i] == 1 || range_mat_[i] == max_float) {
        label_mat_[i] = -1;
      }
    }

    // 提取地面点云
    for (int row = 0; row <= ground_scan_index_; ++row) {
      for (int col = 0; col < num_horizontal_scans_; ++col) {
        const size_t index = col + row * num_horizontal_scans_;
        if (ground_mat_[index] == 1) {
          ground_cloud_->points.push_back(full_cloud_[index]);
        }
      }
    }
  }

  void cloudSegmentation()
  {
    for (int row = 0; row < num_vertical_scans_; ++row) {
      for (int col = 0; col < num_horizontal_scans_; ++col) {
        if (label_mat_[col + row * num_horizontal_scans_] == 0) {
          labelComponents(row, col);
        }
      }
    }
  }

  void labelComponents(int row, int col)
  {
    const int n_col = num_horizontal_scans_;
    const int n_row = num_vertical_scans_;

    std::queue<int> queue;
    std::vector<int> all_pushed;
    std::vector<bool> line_count_flag(n_row, false);

    const int start = row * n_col + col;
    queue.push(start);
    all_pushed.push_back(start);

    // 4 邻域：左 / 上 / 下 / 右（行偏移, 列偏移）
    static const int dR[4] = {0, -1, 1, 0};
    static const int dC[4] = {-1, 0, 0, 1};

    while (!queue.empty()) {
      const int idx = queue.front();
      queue.pop();
      label_mat_[idx] = label_count_;

      const int r = idx / n_col;
      const int c = idx % n_col;

      for (int k = 0; k < 4; ++k) {
        const int nr = r + dR[k];
        const int nc = c + dC[k];

        if (nr < 0 || nr >= n_row) {
          continue;
        }
        // 前视雷达：水平两端是真实边界，不做 wrap-around
        if (nc < 0 || nc >= n_col) {
          continue;
        }

        const int nidx = nr * n_col + nc;
        if (label_mat_[nidx] != 0) {
          continue;
        }

        const float d1 = std::max(range_mat_[idx], range_mat_[nidx]);
        const float d2 = std::min(range_mat_[idx], range_mat_[nidx]);
        const float alpha = (dR[k] == 0) ? ang_res_x_ : ang_res_y_;
        const float tang = d2 * std::sin(alpha) / (d1 - d2 * std::cos(alpha));

        if (tang > segment_theta_tan_) {
          queue.push(nidx);
          label_mat_[nidx] = label_count_;
          line_count_flag[nr] = true;
          all_pushed.push_back(nidx);
        }
      }
    }

    // 判定该簇是否为有效障碍物
    bool feasible = false;
    if (all_pushed.size() >= static_cast<size_t>(min_cluster_size_)) {
      feasible = true;
    } else if (all_pushed.size() >= static_cast<size_t>(segment_valid_point_num_)) {
      int line_count = 0;
      for (const bool f : line_count_flag) {
        if (f) {
          ++line_count;
        }
      }
      if (line_count >= segment_valid_line_num_) {
        feasible = true;
      }
    }

    if (feasible) {
      ++label_count_;
    } else {
      for (const int idx : all_pushed) {
        label_mat_[idx] = kInvalidLabel;
      }
    }
  }

  box_msg::msg::Boxs buildBoxs(const std_msgs::msg::Header & header)
  {
    box_msg::msg::Boxs boxs;
    boxs.header = header;

    // 按簇号收集点，同时填充带簇号的聚类点云
    std::map<int, std::vector<size_t>> clusters;
    for (size_t i = 0; i < size_; ++i) {
      const int lab = label_mat_[i];
      if (lab > 0 && lab != kInvalidLabel) {
        clusters[lab].push_back(i);

        pcl::PointXYZI p = full_cloud_[i];
        p.intensity = static_cast<float>(lab);
        segmented_pure_->points.push_back(p);
      }
    }

    for (const auto & kv : clusters) {
      bool first = true;
      float min_x = 0.0f, min_y = 0.0f, min_z = 0.0f;
      float max_x = 0.0f, max_y = 0.0f, max_z = 0.0f;
      for (const size_t idx : kv.second) {
        const auto & p = full_cloud_[idx];
        if (first) {
          min_x = max_x = p.x;
          min_y = max_y = p.y;
          min_z = max_z = p.z;
          first = false;
        } else {
          min_x = std::min(min_x, p.x);
          max_x = std::max(max_x, p.x);
          min_y = std::min(min_y, p.y);
          max_y = std::max(max_y, p.y);
          min_z = std::min(min_z, p.z);
          max_z = std::max(max_z, p.z);
        }
      }
      if (first) {
        continue;
      }

      box_msg::msg::Box b;
      b.x = (min_x + max_x) / 2.0f;
      b.y = (min_y + max_y) / 2.0f;
      b.z = (min_z + max_z) / 2.0f;
      b.w = max_x - min_x;
      b.l = max_y - min_y;
      b.h = max_z - min_z;
      b.vx = 0.0f;
      b.vy = 0.0f;
      b.rt = 0.0f;
      b.id = kv.first;
      b.score = 0.0f;
      b.track_id = "";
      b.label = "";
      boxs.box.push_back(b);
    }

    return boxs;
  }

  visualization_msgs::msg::MarkerArray boxsToMarkerArray(const box_msg::msg::Boxs & boxs)
  {
    visualization_msgs::msg::MarkerArray marker_array;

    // 先清空该命名空间下的旧 Marker，避免上一帧残留
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header = boxs.header;
    clear_marker.ns = "range_image_seg";
    clear_marker.id = 0;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);

    int id = 1;
    const int n = static_cast<int>(boxs.box.size());
    for (const auto & b : boxs.box) {
      visualization_msgs::msg::Marker marker;
      marker.header = boxs.header;
      marker.ns = "range_image_seg";
      marker.id = id;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = b.x;
      marker.pose.position.y = b.y;
      marker.pose.position.z = b.z;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = std::max(b.w, 0.1f);
      marker.scale.y = std::max(b.l, 0.1f);
      marker.scale.z = std::max(b.h, 0.1f);

      // 不同簇使用不同颜色
      const float h = (n > 1) ? (static_cast<float>(id - 1) / static_cast<float>(n)) : 0.0f;
      const float s = 0.9f, v = 1.0f;
      const int hi = static_cast<int>(std::floor(h * 6.0f));
      const float f = h * 6.0f - hi;
      const float p = v * (1.0f - s);
      const float q = v * (1.0f - s * f);
      const float t = v * (1.0f - s * (1.0f - f));
      switch (hi % 6) {
        case 0: marker.color.r = v; marker.color.g = t; marker.color.b = p; break;
        case 1: marker.color.r = q; marker.color.g = v; marker.color.b = p; break;
        case 2: marker.color.r = p; marker.color.g = v; marker.color.b = t; break;
        case 3: marker.color.r = p; marker.color.g = q; marker.color.b = v; break;
        case 4: marker.color.r = t; marker.color.g = p; marker.color.b = v; break;
        default: marker.color.r = v; marker.color.g = p; marker.color.b = q; break;
      }
      marker.color.a = 0.5f;
      marker_array.markers.push_back(marker);
      ++id;
    }

    return marker_array;
  }

  void publishClouds(const std_msgs::msg::Header & header)
  {
    // 地面点云
    sensor_msgs::msg::PointCloud2 ground_msg;
    pcl::toROSMsg(*ground_cloud_, ground_msg);
    ground_msg.header = header;
    pub_ground_->publish(ground_msg);

    // 聚类点云 + 包围盒
    const box_msg::msg::Boxs boxs = buildBoxs(header);

    sensor_msgs::msg::PointCloud2 seg_msg;
    pcl::toROSMsg(*segmented_pure_, seg_msg);
    seg_msg.header = header;
    pub_segmented_->publish(seg_msg);

    pub_boxs_->publish(boxs);
    pub_marker_->publish(boxsToMarkerArray(boxs));
  }

  // ---- 订阅 / 发布 ---- //
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_segmented_;
  rclcpp::Publisher<box_msg::msg::Boxs>::SharedPtr pub_boxs_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_;

  // ---- 投影参数 ---- //
  int num_vertical_scans_ = 144;
  int num_horizontal_scans_ = 180;
  double vertical_angle_bottom_ = -75.0;
  double vertical_angle_top_ = 15.0;
  double horizontal_fov_ = 120.0;
  double min_range_ = 0.2;

  // ---- 地面分割参数 ---- //
  int ground_scan_index_ = 100;
  double sensor_mount_angle_ = 0.0;
  double ground_angle_threshold_ = 10.0;
  double ground_z_threshold_ = 1.1;

  // ---- 分割参数 ---- //
  double segment_theta_ = 60.0;
  int segment_valid_point_num_ = 5;
  int segment_valid_line_num_ = 3;
  int min_cluster_size_ = 30;

  // ---- 派生量 ---- //
  size_t size_ = 0;
  double ang_res_x_ = 0.0;
  double ang_res_y_ = 0.0;
  double ang_bottom_ = 0.0;
  double half_fov_ = 0.0;
  double sensor_mount_angle_rad_ = 0.0;
  double ground_angle_threshold_rad_ = 0.0;
  double segment_theta_tan_ = 0.0;

  pcl::PointXYZI nan_point_;

  // ---- 每帧数据 ---- //
  pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_in_;
  std::vector<float> range_mat_;
  std::vector<int8_t> ground_mat_;
  std::vector<int> label_mat_;
  std::vector<pcl::PointXYZI> full_cloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr segmented_pure_;
  int label_count_ = 1;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RangeImageSegmentation>());
  rclcpp::shutdown();
  return 0;
}
