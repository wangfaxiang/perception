/**
 * @file pointcloud_preprocess_node.cpp
 * @brief 调平点云预处理：全量扬尘过滤 → 体素降采样 → 离群点过滤 → 高度平滑
 *
 * 把 ground_segmentation/ransac_ground_filter 中的「扬尘过滤 + 降采样 + 高度平滑」
 * 独立成预处理节点，作为 elevation_obstacle_detection 的上游输入。
 *
 * 流程（全量处理，不做地面/非地面分割）:
 *   1) 订阅调平后点云 /<lidar>/rslidar_points_leveled
 *   2) 全量扬尘过滤：单帧球坐标网格密度法（参考 ransac 的 applyDustFilter），
 *      网格内点数 < dust_filter_min_density 判为扬尘，整格剔除
 *   3) 体素降采样（VoxelGrid，leaf_size 可配置）
 *   4) 统计离群点过滤（StatisticalOutlierRemoval，剔除孤立噪点/扬尘残留）
 *   5) 水平邻域高度平均平滑（参考 ransac 的 smoothHeightCloud，k 近邻可配置）
 *   发布三个中间产物话题：
 *     /<lidar>/leveled_dust_filtered    除尘后点云
 *     /<lidar>/leveled_downsampled      降采样后点云
 *     /<lidar>/leveled_filtered_smoothed 高度平滑后点云（elevation_obstacle_detection 输入）
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{
constexpr double kDeg2Rad = M_PI / 180.0;

// ---- 扬尘过滤（单帧网格密度法）球坐标 bin 编码（参考 ransac 实现）----
constexpr std::int64_t kAxisMask = 0x1FFFFF;
constexpr int kAxisBits = 21;
constexpr std::int64_t kInvalidKey = -1;  // encodeBins 结果恒 >= 0，用 -1 表示无效点

// 三个整数 bin 编码进 int64（负 bin 经掩码按补码保存，邻域正确）
inline std::int64_t encodeBins(std::int64_t ia, std::int64_t ie, std::int64_t ir)
{
  return ((ia & kAxisMask) << (2 * kAxisBits)) |
         ((ie & kAxisMask) << kAxisBits) |
         (ir & kAxisMask);
}

// 按索引逐字节抽取点云子集，完整保留 intensity/ring/time 等原始字段
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
  out.row_step = out.point_step * static_cast<std::uint32_t>(indices.size());
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
}  // namespace

class PointCloudPreprocess : public rclcpp::Node
{
public:
  PointCloudPreprocess()
  : Node("pointcloud_preprocess")
  {
    // 雷达标识（front / rear），用于推导默认话题名
    const std::string lidar_name = declare_parameter<std::string>("lidar_name", "");
    const std::string prefix = lidar_name.empty() ? "" : ("/" + lidar_name);

    input_topic_ = declare_parameter<std::string>(
      "input_topic", prefix + "/rslidar_points_leveled");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", prefix + "/leveled_filtered_smoothed");
    dust_filtered_topic_ = declare_parameter<std::string>(
      "dust_filtered_topic", prefix + "/leveled_dust_filtered");
    downsampled_topic_ = declare_parameter<std::string>(
      "downsampled_topic", prefix + "/leveled_downsampled");
    // 输出 frame_id：为空则沿用输入点云的 frame_id
    frame_id_ = declare_parameter<std::string>("frame_id", "");

    // ---- 扬尘过滤（单帧网格密度法，全量）----
    enable_dust_filter_ = declare_parameter<bool>("enable_dust_filter", true);
    dust_horiz_angle_res_deg_ =
      declare_parameter<double>("dust_filter_horiz_angle_resolution", 1.0);
    dust_vert_angle_res_deg_ =
      declare_parameter<double>("dust_filter_vert_angle_resolution", 1.0);
    dust_range_res_ = declare_parameter<double>("dust_filter_range_resolution", 0.1);
    dust_range_growth_ = declare_parameter<double>("dust_filter_range_growth", 0.02);
    dust_min_density_ = declare_parameter<int>("dust_filter_min_density", 20);
    if (dust_min_density_ < 1) dust_min_density_ = 1;
    if (dust_range_res_ <= 0.0) dust_range_res_ = 0.1;
    if (dust_range_growth_ < 0.0) dust_range_growth_ = 0.0;
    // 时序累积：共 dust_filter_history_frames_ + 1 帧（当前 + 历史）参与密度统计
    dust_filter_history_frames_ =
      declare_parameter<int>("dust_filter_history_frames", 1);
    if (dust_filter_history_frames_ < 0) dust_filter_history_frames_ = 0;

    // ---- 降采样 ----
    enable_downsample_ = declare_parameter<bool>("enable_downsample", true);
    downsample_leaf_size_ = declare_parameter<double>("downsample_leaf_size", 0.05);

    // ---- 统计离群点过滤 ----
    enable_outlier_filter_ = declare_parameter<bool>("enable_outlier_filter", true);
    outlier_mean_k_ = declare_parameter<int>("outlier_mean_k", 20);
    outlier_std_mul_ = declare_parameter<double>("outlier_std_mul", 1.0);
    if (outlier_mean_k_ < 1) outlier_mean_k_ = 1;

    // ---- 高度平滑 ----
    enable_smooth_ = declare_parameter<bool>("enable_smooth", true);
    smooth_k_neighbors_ = declare_parameter<int>("smooth_k_neighbors", 16);
    if (smooth_k_neighbors_ < 1) smooth_k_neighbors_ = 1;

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PointCloudPreprocess::onCloud, this, std::placeholders::_1));
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, rclcpp::QoS(10));
    pub_dust_filtered_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(dust_filtered_topic_, rclcpp::QoS(10));
    pub_downsampled_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(downsampled_topic_, rclcpp::QoS(10));

    RCLCPP_INFO(this->get_logger(),
                "Point cloud preprocessing started (lidar=%s): %s -> dust_filtered=%s -> "
                "downsampled=%s -> smoothed=%s, "
                "dust_filter=%s [%.1f°/%.1f°/%.2fm+%.2f, min_density=%d, history=%d], "
                "downsample=%s [leaf=%.2fm], outlier=%s [k=%d, std=%.1f], "
                "smooth=%s [k=%d], frame=%s",
                lidar_name.empty() ? "single" : lidar_name.c_str(),
                input_topic_.c_str(), dust_filtered_topic_.c_str(),
                downsampled_topic_.c_str(), output_topic_.c_str(),
                enable_dust_filter_ ? "on" : "off",
                dust_horiz_angle_res_deg_, dust_vert_angle_res_deg_,
                dust_range_res_, dust_range_growth_, dust_min_density_,
                dust_filter_history_frames_,
                enable_downsample_ ? "on" : "off", downsample_leaf_size_,
                enable_outlier_filter_ ? "on" : "off", outlier_mean_k_, outlier_std_mul_,
                enable_smooth_ ? "on" : "off", smooth_k_neighbors_,
                frame_id_.empty() ? "<keep input>" : frame_id_.c_str());
  }

private:
  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // 1) 全量扬尘过滤（不做地面分割），逐字节保留原始字段
    sensor_msgs::msg::PointCloud2 dust_filtered;
    if (enable_dust_filter_) {
      applyDustFilter(*msg, dust_filtered);
    } else {
      dust_filtered = *msg;
      dust_history_.clear();  // 关闭除尘时清空历史，避免陈旧帧影响后续开启
    }

    // 发布除尘后点云
    sensor_msgs::msg::PointCloud2 dust_filtered_out = dust_filtered;
    dust_filtered_out.header.stamp = msg->header.stamp;
    dust_filtered_out.header.frame_id =
      frame_id_.empty() ? msg->header.frame_id : frame_id_;
    pub_dust_filtered_->publish(dust_filtered_out);

    // 2) 转 PCL、移除 NaN
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(dust_filtered, *cloud);
    std::vector<int> nan_indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, nan_indices);

    // 3) 体素降采样
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>());
    if (enable_downsample_ && downsample_leaf_size_ > 0.0) {
      pcl::VoxelGrid<pcl::PointXYZI> vg;
      vg.setInputCloud(cloud);
      vg.setLeafSize(static_cast<float>(downsample_leaf_size_),
                     static_cast<float>(downsample_leaf_size_),
                     static_cast<float>(downsample_leaf_size_));
      vg.filter(*downsampled);
    } else {
      downsampled = cloud;
    }

    // 发布降采样后点云（体素降采样，离群点过滤前）
    sensor_msgs::msg::PointCloud2 downsampled_out;
    pcl::toROSMsg(*downsampled, downsampled_out);
    downsampled_out.header.stamp = msg->header.stamp;
    downsampled_out.header.frame_id =
      frame_id_.empty() ? msg->header.frame_id : frame_id_;
    pub_downsampled_->publish(downsampled_out);

    // 4) 统计离群点过滤（剔除孤立噪点/扬尘残留）
    pcl::PointCloud<pcl::PointXYZI>::Ptr denoised(new pcl::PointCloud<pcl::PointXYZI>());
    if (enable_outlier_filter_ &&
        downsampled->size() > static_cast<std::size_t>(outlier_mean_k_))
    {
      pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
      sor.setInputCloud(downsampled);
      sor.setMeanK(outlier_mean_k_);
      sor.setStddevMulThresh(outlier_std_mul_);
      sor.filter(*denoised);
    } else {
      denoised = downsampled;
    }

    // 5) 高度平滑（水平邻域高度平均）
    pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>());
    if (enable_smooth_) {
      smoothHeightCloud(denoised, output);
    } else {
      output = denoised;
    }

    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(*output, out);
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = frame_id_.empty() ? msg->header.frame_id : frame_id_;
    pub_->publish(out);
  }

  void applyDustFilter(
    const sensor_msgs::msg::PointCloud2 & input, sensor_msgs::msg::PointCloud2 & output)
  {
    const double horiz_angle_res = dust_horiz_angle_res_deg_ * kDeg2Rad;
    const double vert_angle_res = dust_vert_angle_res_deg_ * kDeg2Rad;

    // 单帧球坐标网格密度法：把调平点云划分成 (水平角, 垂直角, 距离) 网格，
    // 网格内点数 < dust_min_density_ 判为扬尘，整格剔除（全部点云参与除尘）。
    sensor_msgs::PointCloud2ConstIterator<float> it_x(input, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(input, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(input, "z");

    const std::size_t num_points =
      static_cast<std::size_t>(input.width) * input.height;
    std::vector<std::int64_t> point_keys(num_points, kInvalidKey);
    std::unordered_map<std::int64_t, std::uint32_t> bin_count;
    bin_count.reserve(num_points);

    // 第一遍：计算每个点的球坐标 bin，并累计每个 bin 的点数
    std::size_t i = 0;
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

      const std::int64_t ia =
        static_cast<std::int64_t>(std::floor(azimuth / horiz_angle_res));
      const std::int64_t ie =
        static_cast<std::int64_t>(std::floor(elevation / vert_angle_res));
      const std::int64_t ir =
        static_cast<std::int64_t>(std::floor(range / eff_range_res));
      const std::int64_t key = encodeBins(ia, ie, ir);
      point_keys[i] = key;
      ++bin_count[key];
    }

    // 第二遍：统计「当前帧 + 历史帧」累积密度，低于阈值的网格判为扬尘剔除。
    // 灰尘是浮动的，两帧在同一个网格里不会稳定叠加；固定障碍物两帧都会落入同一网格，
    // 因此累积密度能更彻底地剔除灰尘、保留真实障碍物。非有限点原样保留。
    std::vector<std::size_t> keep_idx;
    keep_idx.reserve(num_points);
    for (std::size_t j = 0; j < num_points; ++j) {
      const std::int64_t key = point_keys[j];
      if (key == kInvalidKey) {
        keep_idx.push_back(j);
        continue;
      }
      std::uint64_t total = 0;
      const auto it = bin_count.find(key);
      if (it != bin_count.end()) {
        total += it->second;
      }
      for (const auto & hist : dust_history_) {
        const auto hit = hist.find(key);
        if (hit != hist.end()) {
          total += hit->second;
        }
      }
      if (total >= static_cast<std::uint64_t>(dust_min_density_)) {
        keep_idx.push_back(j);
      }
    }

    // 保存当前帧 bin 计数作为下一帧的历史（最多保留 dust_filter_history_frames_ 帧）
    if (dust_filter_history_frames_ > 0) {
      dust_history_.push_back(std::move(bin_count));
      while (static_cast<int>(dust_history_.size()) > dust_filter_history_frames_) {
        dust_history_.pop_front();
      }
    }

    output = buildCloudFromIndices(input, keep_idx);
    output.header = input.header;
  }

  void smoothHeightCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr & input,
    pcl::PointCloud<pcl::PointXYZI>::Ptr & output)
  {
    output->clear();
    if (input->empty()) return;

    // 水平方向 k 近邻高度平均：把 z 置 0 构建 2D kdtree，找 k 个水平最近邻取 z 平均
    // （只平滑高度，intensity 保持降采样后的值）
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xy(new pcl::PointCloud<pcl::PointXYZI>());
    cloud_xy->resize(input->size());
    for (std::size_t i = 0; i < input->size(); ++i) {
      (*cloud_xy)[i] = (*input)[i];
      (*cloud_xy)[i].z = 0.0f;
    }

    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(cloud_xy);

    const int k = std::max(1, smooth_k_neighbors_);
    output->resize(input->size());
    std::vector<int> neighbors(k);
    std::vector<float> dist2(k);

    for (std::size_t i = 0; i < input->size(); ++i) {
      const auto & p = (*input)[i];
      float z_sum = p.z;
      int count = 1;
      const int found = kdtree.nearestKSearch((*cloud_xy)[i], k, neighbors, dist2);
      for (int j = 0; j < found; ++j) {
        if (neighbors[j] != static_cast<int>(i)) {
          z_sum += (*input)[neighbors[j]].z;
          ++count;
        }
      }
      (*output)[i] = p;  // 保留 intensity 等字段
      (*output)[i].z = z_sum / static_cast<float>(count);
    }
    output->width = output->size();
    output->height = 1;
    output->is_dense = true;
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_dust_filtered_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_downsampled_;

  std::string input_topic_, output_topic_, frame_id_;
  std::string dust_filtered_topic_, downsampled_topic_;

  bool enable_dust_filter_;
  double dust_horiz_angle_res_deg_, dust_vert_angle_res_deg_;
  double dust_range_res_, dust_range_growth_;
  int dust_min_density_;
  int dust_filter_history_frames_;
  std::deque<std::unordered_map<std::int64_t, std::uint32_t>> dust_history_;

  bool enable_downsample_;
  double downsample_leaf_size_;

  bool enable_outlier_filter_;
  int outlier_mean_k_;
  double outlier_std_mul_;

  bool enable_smooth_;
  int smooth_k_neighbors_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudPreprocess>());
  rclcpp::shutdown();
  return 0;
}
