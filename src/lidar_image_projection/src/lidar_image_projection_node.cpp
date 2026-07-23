/**
 * @file lidar_image_projection_node.cpp
 * @brief 将速腾聚创E1固态雷达点云投影为距离/强度图像
 *
 * E1雷达规格:
 *   - 水平视场角: 120° (-60° ~ +60°)
 *   - 垂直视场角: 90°  (-45° ~ +45°)
 *
 * 参考: LeGO-LOAM imageProjection.cpp 的点云投影思路
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

class LidarImageProjection : public rclcpp::Node
{
public:
  LidarImageProjection()
  : Node("lidar_image_projection")
  {
    // 声明参数
    this->declare_parameter("input_topic", "/rslidar_points");
    this->declare_parameter("range_image_topic", "/lidar_range_image");
    this->declare_parameter("intensity_image_topic", "/lidar_intensity_image");
    this->declare_parameter("height_image_topic", "/lidar_height_image");
    this->declare_parameter("frame_id", "rslidar");

    // E1 雷达视场角参数
    this->declare_parameter("horiz_fov_deg", 120.0);    // 水平视场角（度）
    this->declare_parameter("vert_fov_deg", 90.0);       // 垂直视场角（度）
    this->declare_parameter("horiz_resolution_deg", 0.2);  // 水平分辨率（度/像素）
    this->declare_parameter("vert_resolution_deg", 0.2);   // 垂直分辨率（度/像素）

    // 距离归一化参数
    this->declare_parameter("max_range", 100.0);         // 最大距离（米）
    this->declare_parameter("min_range", 0.2);           // 最小距离（米）

    // 高度映射参数: z 从 height_top→gray_top, 到 height_bottom→gray_bottom
    this->declare_parameter("height_top", 0.0);
    this->declare_parameter("height_bottom", -10.0);
    this->declare_parameter("height_top_gray", 255);
    this->declare_parameter("height_bottom_gray", 120);

    // 后处理参数
    this->declare_parameter("dilate_kernel_size", 3);    // 膨胀核大小（0=禁用）
    this->declare_parameter("fill_holes", true);         // 填充所有空洞

    // 读取参数
    std::string input_topic, range_topic, intensity_topic, height_topic;
    this->get_parameter("input_topic", input_topic);
    this->get_parameter("range_image_topic", range_topic);
    this->get_parameter("intensity_image_topic", intensity_topic);
    this->get_parameter("height_image_topic", height_topic);
    this->get_parameter("frame_id", frame_id_);

    this->get_parameter("horiz_fov_deg", horiz_fov_deg_);
    this->get_parameter("vert_fov_deg", vert_fov_deg_);
    this->get_parameter("horiz_resolution_deg", horiz_res_deg_);
    this->get_parameter("vert_resolution_deg", vert_res_deg_);
    this->get_parameter("max_range", max_range_);
    this->get_parameter("min_range", min_range_);
    this->get_parameter("height_top", height_top_);
    this->get_parameter("height_bottom", height_bottom_);
    this->get_parameter("height_top_gray", height_top_gray_);
    this->get_parameter("height_bottom_gray", height_bottom_gray_);
    this->get_parameter("dilate_kernel_size", dilate_ks_);
    this->get_parameter("fill_holes", fill_holes_);

    // 计算图像尺寸
    img_cols_ = static_cast<int>(horiz_fov_deg_ / horiz_res_deg_);
    img_rows_ = static_cast<int>(vert_fov_deg_ / vert_res_deg_);

    // 预计算弧度值
    horiz_min_rad_ = -horiz_fov_deg_ / 2.0 * M_PI / 180.0;
    vert_max_rad_  =  vert_fov_deg_ / 2.0 * M_PI / 180.0;
    horiz_res_rad_ = horiz_res_deg_ * M_PI / 180.0;
    vert_res_rad_  = vert_res_deg_ * M_PI / 180.0;

    RCLCPP_INFO(this->get_logger(),
      "Image size: %d x %d (H x V), H-FOV=%.1f°, V-FOV=%.1f°, Res=%.2f°/pix",
      img_cols_, img_rows_, horiz_fov_deg_, vert_fov_deg_, horiz_res_deg_);

    // 订阅点云
    sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::SensorDataQoS(),
      std::bind(&LidarImageProjection::cloudCallback, this, std::placeholders::_1));

    // 发布图像
    pub_range_img_ = this->create_publisher<sensor_msgs::msg::Image>(
      range_topic, 10);
    pub_intensity_img_ = this->create_publisher<sensor_msgs::msg::Image>(
      intensity_topic, 10);
    pub_height_img_ = this->create_publisher<sensor_msgs::msg::Image>(
      height_topic, 10);

    RCLCPP_INFO(this->get_logger(), "LidarImageProjection node started.");
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // 转换为 PCL 点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *cloud);

    // 移除 NaN 点
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

    // 创建图像 (CV_32FC1 用于累积，CV_8UC1 用于最终输出)
    cv::Mat range_mat  = cv::Mat::zeros(img_rows_, img_cols_, CV_32FC1);
    cv::Mat intens_mat = cv::Mat::zeros(img_rows_, img_cols_, CV_32FC1);
    cv::Mat height_mat = cv::Mat::zeros(img_rows_, img_cols_, CV_32FC1);
    cv::Mat count_mat  = cv::Mat::zeros(img_rows_, img_cols_, CV_32FC1);

    for (const auto& pt : cloud->points) {
      float range = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);

      // 距离过滤
      if (range < min_range_ || range > max_range_) {
        continue;
      }

      // 计算垂直角: asin(z / range)
      float vert_angle = std::asin(pt.z / range);

      // 计算水平角: atan2(y, x)
      float horiz_angle = std::atan2(pt.y, pt.x);

      // 视场角过滤
      if (vert_angle > vert_max_rad_ || vert_angle < vert_max_rad_ - vert_fov_deg_ * M_PI / 180.0) {
        continue;
      }
      if (horiz_angle < horiz_min_rad_ || horiz_angle > -horiz_min_rad_) {
        continue;
      }

      // 计算像素坐标
      // row: 顶部(最大垂直角) = 第0行, 底部(最小垂直角) = 最后一行
      int row = static_cast<int>((vert_max_rad_ - vert_angle) / vert_res_rad_);
      int col = static_cast<int>((horiz_angle - horiz_min_rad_) / horiz_res_rad_);

      if (row < 0 || row >= img_rows_ || col < 0 || col >= img_cols_) {
        continue;
      }

      // 累加（处理多个点落入同一像素的情况）
      range_mat.at<float>(row, col)  += range;
      intens_mat.at<float>(row, col) += pt.intensity;
      height_mat.at<float>(row, col) += pt.z;
      count_mat.at<float>(row, col)  += 1.0f;
    }

    // 归一化并生成最终图像
    cv::Mat range_img  = cv::Mat::zeros(img_rows_, img_cols_, CV_8UC1);
    cv::Mat intens_img = cv::Mat::zeros(img_rows_, img_cols_, CV_8UC1);
    cv::Mat height_img = cv::Mat::zeros(img_rows_, img_cols_, CV_8UC1);

    for (int r = 0; r < img_rows_; ++r) {
      for (int c = 0; c < img_cols_; ++c) {
        float cnt = count_mat.at<float>(r, c);
        if (cnt > 0) {
          // 距离归一化到 0-255 (近→亮, 远→暗)
          float range_val = range_mat.at<float>(r, c) / cnt;
          float range_norm = 1.0f - (range_val - min_range_) / (max_range_ - min_range_);
          range_norm = std::clamp(range_norm, 0.0f, 1.0f);
          range_img.at<uint8_t>(r, c) = static_cast<uint8_t>(range_norm * 255.0f);

          // 强度归一化到 0-255
          float intens_val = intens_mat.at<float>(r, c) / cnt;
          float intens_norm = std::min(intens_val / 255.0f, 1.0f);
          intens_img.at<uint8_t>(r, c) = static_cast<uint8_t>(intens_norm * 255.0f);

          // 高度映射: z∈[height_bottom, height_top] → 灰度 [gray_bottom, gray_top]
          float h = height_mat.at<float>(r, c) / cnt;
          int gray = 0;
          if (h <= height_top_ && h >= height_bottom_) {
            float t = (height_top_ - h) / (height_top_ - height_bottom_);
            gray = static_cast<int>(height_top_gray_ - t * (height_top_gray_ - height_bottom_gray_));
          }
          height_img.at<uint8_t>(r, c) = static_cast<uint8_t>(gray);
        }
      }
    }

    // 形态学膨胀填充间隙
    if (dilate_ks_ > 0) {
      cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE, cv::Size(dilate_ks_, dilate_ks_));
      cv::dilate(range_img, range_img, kernel);
      cv::dilate(intens_img, intens_img, kernel);
      cv::dilate(height_img, height_img, kernel);
    }

    // 图像修补：用更大膨胀填充所有剩余空洞
    if (fill_holes_) {
      cv::Mat kernel2 = cv::getStructuringElement(
        cv::MORPH_ELLIPSE, cv::Size(9, 9));
      cv::dilate(range_img, range_img, kernel2);
      cv::dilate(intens_img, intens_img, kernel2);
      cv::dilate(height_img, height_img, kernel2);
    }

    // 发布图像
    auto timestamp = this->now();

    auto range_msg = cv_bridge::CvImage(
      std_msgs::msg::Header(), "mono8", range_img).toImageMsg();
    range_msg->header.stamp = timestamp;
    range_msg->header.frame_id = frame_id_;
    pub_range_img_->publish(*range_msg);

    auto intens_msg = cv_bridge::CvImage(
      std_msgs::msg::Header(), "mono8", intens_img).toImageMsg();
    intens_msg->header.stamp = timestamp;
    intens_msg->header.frame_id = frame_id_;
    pub_intensity_img_->publish(*intens_msg);

    auto height_msg = cv_bridge::CvImage(
      std_msgs::msg::Header(), "mono8", height_img).toImageMsg();
    height_msg->header.stamp = timestamp;
    height_msg->header.frame_id = frame_id_;
    pub_height_img_->publish(*height_msg);
  }

  // 订阅和发布
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_range_img_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_intensity_img_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_height_img_;

  // 参数
  std::string frame_id_;
  double horiz_fov_deg_, vert_fov_deg_;
  double horiz_res_deg_, vert_res_deg_;
  double max_range_, min_range_;
  double height_top_, height_bottom_;
  int height_top_gray_, height_bottom_gray_;
  int dilate_ks_;
  bool fill_holes_;
  int img_cols_, img_rows_;
  double horiz_min_rad_, vert_max_rad_;
  double horiz_res_rad_, vert_res_rad_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarImageProjection>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
