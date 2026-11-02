/**
 * @file lidar_leveling_node.cpp
 * @brief 根据雷达安装欧拉角将点云旋转回水平，输出调平后的点云话题
 *
 * 适用: 前/后速腾聚创 E1R 雷达（frame: front_rslidar / rear_rslidar）。
 *
 * 原理:
 *   雷达安装存在俯仰/横滚/偏航时，地面在雷达坐标系中是倾斜的。
 *   本节点按安装欧拉角构造旋转矩阵 R，把每个点从雷达坐标系变换到
 *   水平（整车）坐标系:
 *       p_level = R * p_lidar
 *       R = Rz(yaw) * Ry(pitch) * Rx(roll)
 *   其中 roll 绕 X(前)、pitch 绕 Y(左)、yaw 绕 Z(上)，均遵循右手定则。
 *
 * 调平示例:
 *   若雷达向下俯仰安装 θ（光轴指向地面），则地面远点 z 随 x 增大而增大，
 *   设 pitch_deg = +θ 即可把地面调平（即对点云绕 Y 轴正向旋转 θ）。
 *
 * 支持多雷达: 通过 lidar_name 参数（front/rear）区分前/后雷达，同一节点可启动多个实例。
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <cstdint>
#include <string>
#include <unordered_map>

namespace
{
constexpr double kDeg2Rad = M_PI / 180.0;

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
    std::size_t h = std::hash<std::int64_t>{}(key.ix);
    h = h * 1315423911u + std::hash<std::int64_t>{}(key.iy);
    h = h * 1315423911u + std::hash<std::int64_t>{}(key.iz);
    return h;
  }
};
}

class LidarLevelingNode : public rclcpp::Node
{
public:
  LidarLevelingNode() : Node("lidar_leveling")
  {
    // 雷达标识（front / rear），用于推导默认话题名
    std::string lidar_name;
    this->declare_parameter<std::string>("lidar_name", "");
    this->get_parameter("lidar_name", lidar_name);

    const std::string prefix = lidar_name.empty() ? "" : ("/" + lidar_name);
    this->declare_parameter<std::string>("input_topic", prefix + "/rslidar_points");
    this->declare_parameter<std::string>("output_topic", prefix + "/rslidar_points_leveled");
    // 输出 frame_id，为空则沿用输入点云的 frame_id
    this->declare_parameter<std::string>("frame_id", "");

    // 雷达安装欧拉角（度）
    this->declare_parameter("roll_deg", 0.0);
    this->declare_parameter("pitch_deg", 30.0);
    this->declare_parameter("yaw_deg", 0.0);

    // 点云降采样（距离自适应体素）
    this->declare_parameter("use_downsample", false);
    this->declare_parameter("voxel_size_x", 0.1);
    this->declare_parameter("voxel_size_y", 0.1);
    this->declare_parameter("voxel_size_z", 0.1);
    this->declare_parameter("use_adaptive_voxel", true);
    this->declare_parameter("adaptive_voxel_ratio", 0.015);
    this->declare_parameter("adaptive_voxel_max_size", 0.4);

    this->get_parameter("input_topic", input_topic_);
    this->get_parameter("output_topic", output_topic_);
    this->get_parameter("frame_id", frame_id_);

    double roll_deg = 0.0, pitch_deg = 0.0, yaw_deg = 0.0;
    this->get_parameter("roll_deg", roll_deg);
    this->get_parameter("pitch_deg", pitch_deg);
    this->get_parameter("yaw_deg", yaw_deg);
    buildRotation(roll_deg, pitch_deg, yaw_deg);

    this->get_parameter("use_downsample", use_downsample_);
    this->get_parameter("voxel_size_x", voxel_size_x_);
    this->get_parameter("voxel_size_y", voxel_size_y_);
    this->get_parameter("voxel_size_z", voxel_size_z_);
    this->get_parameter("use_adaptive_voxel", use_adaptive_voxel_);
    this->get_parameter("adaptive_voxel_ratio", adaptive_voxel_ratio_);
    this->get_parameter("adaptive_voxel_max_size", adaptive_voxel_max_size_);

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LidarLevelingNode::onCloud, this, std::placeholders::_1));
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);

    RCLCPP_INFO(this->get_logger(),
                "lidar_leveling started: %s -> %s (roll=%.2f, pitch=%.2f, yaw=%.2f deg, "
                "frame_id=%s, downsample=%s)",
                input_topic_.c_str(), output_topic_.c_str(), roll_deg, pitch_deg, yaw_deg,
                frame_id_.empty() ? "<keep input>" : frame_id_.c_str(),
                use_downsample_ ? "on" : "off");
  }

private:
  void buildRotation(double roll_deg, double pitch_deg, double yaw_deg)
  {
    const double roll = roll_deg * kDeg2Rad;
    const double pitch = pitch_deg * kDeg2Rad;
    const double yaw = yaw_deg * kDeg2Rad;

    const Eigen::Matrix3d Rx = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
    const Eigen::Matrix3d Ry =
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();
    const Eigen::Matrix3d Rz = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    rotation_ = Rz * Ry * Rx;
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *cloud);

    for (auto & pt : cloud->points)
    {
      const Eigen::Vector3d p(pt.x, pt.y, pt.z);
      const Eigen::Vector3d q = rotation_ * p;
      pt.x = static_cast<float>(q.x());
      pt.y = static_cast<float>(q.y());
      pt.z = static_cast<float>(q.z());
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud = cloud;
    if (use_downsample_)
    {
      out_cloud = downsample(cloud);
    }

    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(*out_cloud, out);
    out.header = msg->header;
    if (!frame_id_.empty())
    {
      out.header.frame_id = frame_id_;
    }
    pub_->publish(out);
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr downsample(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr & input)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>());
    if (!use_adaptive_voxel_)
    {
      // 固定体素降采样
      pcl::VoxelGrid<pcl::PointXYZI> filter;
      filter.setInputCloud(input);
      filter.setLeafSize(voxel_size_x_, voxel_size_y_, voxel_size_z_);
      filter.filter(*output);
      return output;
    }

    // 距离自适应体素降采样：体素边长随水平距离增大，
    // 使近处与远处的点密度趋于一致。
    std::unordered_map<VoxelKey, std::size_t, VoxelKeyHash> voxel_map;
    voxel_map.reserve(input->points.size());
    for (std::size_t i = 0; i < input->points.size(); ++i)
    {
      const auto & p = input->points[i];
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
      {
        continue;
      }
      const double r = std::sqrt(p.x * p.x + p.y * p.y);
      const double grow = adaptive_voxel_ratio_ * r;
      double sx = voxel_size_x_ + grow;
      double sy = voxel_size_y_ + grow;
      double sz = voxel_size_z_ + grow;
      if (adaptive_voxel_max_size_ > 0.0)
      {
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
      if (it == voxel_map.end())
      {
        voxel_map.emplace(key, i);
      }
      else
      {
        const auto & q = input->points[it->second];
        const double qx = q.x - cx;
        const double qy = q.y - cy;
        const double qz = q.z - cz;
        if (dist2 < qx * qx + qy * qy + qz * qz)
        {
          it->second = i;
        }
      }
    }
    output->points.reserve(voxel_map.size());
    for (const auto & kv : voxel_map)
    {
      output->points.push_back(input->points[kv.second]);
    }
    output->width = output->points.size();
    output->height = 1;
    output->is_dense = true;
    return output;
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  Eigen::Matrix3d rotation_ = Eigen::Matrix3d::Identity();

  bool use_downsample_ = false;
  double voxel_size_x_ = 0.1;
  double voxel_size_y_ = 0.1;
  double voxel_size_z_ = 0.1;
  bool use_adaptive_voxel_ = true;
  double adaptive_voxel_ratio_ = 0.015;
  double adaptive_voxel_max_size_ = 0.4;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarLevelingNode>());
  rclcpp::shutdown();
  return 0;
}
