/**
 * @file lidar_image_projection_node.cpp
 * @brief 将速腾聚创E1固态雷达点云投影为高度图像（1200×144），并提取高度边缘（防矿卡坠坡）
 *
 * E1雷达规格:
 *   - 水平视场角: 120° (-60° ~ +60°), 分辨率 0.1° → 1200 列
 *   - 垂直视场角: 90°  (-45° ~ +45°), 线数 144 → 144 行
 *
 * 边缘检测: 对高度图像做Canny边缘检测，将高度突变处（边坡边缘）标记为红色，
 *           叠加在高度图上发布，用于防止矿卡坠坡。
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
#include <pcl/filters/voxel_grid.h>

#include <string>
#include <vector>
#include <map>
#include <unordered_map>

#include <algorithm>
#include <cmath>
#include <limits>
#include <filesystem>

class LidarImageProjection : public rclcpp::Node
{
public:
  LidarImageProjection()
      : Node("lidar_image_projection")
  {
    // 声明参数
    this->declare_parameter("input_topic", "/rslidar_points");
    this->declare_parameter("height_image_topic", "/lidar_height_image");
    this->declare_parameter("dilated_height_image_topic", "/lidar_dilated_height_image");
    this->declare_parameter("frame_id", "rslidar");

    // E1 雷达视场角参数（1200×144）
    this->declare_parameter("horiz_fov_deg", 120.0);      // 水平视场角（度）
    this->declare_parameter("vert_fov_deg", 90.0);        // 垂直视场角（度）
    this->declare_parameter("horiz_resolution_deg", 1.0); // 水平分辨率 120°/1200=0.1°/像素
    this->declare_parameter("vert_resolution_deg", 1.0);  // 垂直分辨率 90°/144=0.625°/像素

    // 距离归一化参数
    this->declare_parameter("max_range", 100.0); // 最大距离（米）
    this->declare_parameter("min_range", 0.2);   // 最小距离（米）

    // 高度映射参数: z 归一化（height_top(0m)→灰度255, height_bottom(-50m)→灰度122）
    this->declare_parameter("height_top", 0.0);
    this->declare_parameter("height_bottom", -50.0);

    // 后处理参数
    this->declare_parameter("dilate_kernel_size", 3); // 膨胀核大小（0=禁用）
    this->declare_parameter("fill_holes", true);      // 填充所有空洞

    // 体素降采样参数
    this->declare_parameter("voxel_leaf_size", 0.03); // 体素栅格边长（米），0=禁用

    // 边缘检测参数（防矿卡坠坡）
    this->declare_parameter("edge_image_topic", "/lidar_edge_image"); // 边缘图像话题
    this->declare_parameter("edge_cloud_topic", "/lidar_edge_cloud"); // 边缘点云话题
    this->declare_parameter("enable_edge_detection", true);           // 是否启用边缘检测
    this->declare_parameter("canny_low_thresh", 120);                 // Canny 低阈值
    this->declare_parameter("canny_high_thresh", 300);                // Canny 高阈值
    this->declare_parameter("edge_min_height_diff", 0.3);             // 最小高度差（米），大于此值才认为是边缘

    // 本地保存参数
    this->declare_parameter("save_dir", "./"); // 图片保存目录（当前目录）

    // 读取参数
    std::string input_topic, height_topic, dilated_height_topic, edge_topic, edge_cloud_topic;
    this->get_parameter("input_topic", input_topic);
    this->get_parameter("height_image_topic", height_topic);
    this->get_parameter("dilated_height_image_topic", dilated_height_topic);
    this->get_parameter("edge_image_topic", edge_topic);
    this->get_parameter("edge_cloud_topic", edge_cloud_topic);
    this->get_parameter("frame_id", frame_id_);

    this->get_parameter("horiz_fov_deg", horiz_fov_deg_);
    this->get_parameter("vert_fov_deg", vert_fov_deg_);
    this->get_parameter("horiz_resolution_deg", horiz_res_deg_);
    this->get_parameter("vert_resolution_deg", vert_res_deg_);
    this->get_parameter("max_range", max_range_);
    this->get_parameter("min_range", min_range_);
    this->get_parameter("height_top", height_top_);
    this->get_parameter("height_bottom", height_bottom_);
    this->get_parameter("dilate_kernel_size", dilate_ks_);
    this->get_parameter("fill_holes", fill_holes_);
    this->get_parameter("voxel_leaf_size", voxel_leaf_size_);
    this->get_parameter("enable_edge_detection", enable_edge_);
    this->get_parameter("canny_low_thresh", canny_low_);
    this->get_parameter("canny_high_thresh", canny_high_);
    this->get_parameter("save_dir", save_dir_);

    // 计算图像尺寸
    img_cols_ = static_cast<int>(horiz_fov_deg_ / horiz_res_deg_);
    img_rows_ = static_cast<int>(vert_fov_deg_ / vert_res_deg_);

    // 预计算弧度值
    horiz_min_rad_ = -horiz_fov_deg_ / 2.0 * M_PI / 180.0; // -60° (下界)
    vert_max_rad_ = vert_fov_deg_ / 2.0 * M_PI / 180.0;    // +45° (上界)
    vert_min_rad_ = -vert_fov_deg_ / 2.0 * M_PI / 180.0;   // -45° (下界)
    horiz_res_rad_ = horiz_res_deg_ * M_PI / 180.0;
    vert_res_rad_ = vert_res_deg_ * M_PI / 180.0;

    RCLCPP_INFO(this->get_logger(),
                "Image size: %d x %d (H x V), H-FOV=%.1f°, V-FOV=%.1f°, Res=%.2f°/pix",
                img_cols_, img_rows_, horiz_fov_deg_, vert_fov_deg_, horiz_res_deg_);

    // 订阅点云
    sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic, rclcpp::SensorDataQoS(),
        std::bind(&LidarImageProjection::cloudCallback, this, std::placeholders::_1));

    // 发布图像（高度图 → 膨胀高度图 → 边缘图）
    pub_height_img_ = this->create_publisher<sensor_msgs::msg::Image>(
        height_topic, 10);
    pub_dilated_height_img_ = this->create_publisher<sensor_msgs::msg::Image>(
        dilated_height_topic, 10);

    // 边缘图像发布器（防坠坡）
    if (enable_edge_)
    {
      pub_edge_img_ = this->create_publisher<sensor_msgs::msg::Image>(
          edge_topic, 10);
      pub_edge_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
          edge_cloud_topic, 10);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Edge detection DISABLED.");
    }

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

    // 体素降采样（减少点云密度，加速后续处理）
    if (voxel_leaf_size_ > 0.0)
    {
      pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
      voxel_filter.setInputCloud(cloud);
      voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());
      voxel_filter.filter(*cloud_filtered);
      cloud = cloud_filtered;
    }

    // 创建图像 (CV_32FC1 用于累积)
    cv::Mat height_mat = cv::Mat::zeros(img_rows_, img_cols_, CV_32FC1);
    // cv::Mat intens_mat = cv::Mat::zeros(img_rows_, img_cols_, CV_32FC1);
    cv::Mat count_mat = cv::Mat::zeros(img_rows_, img_cols_, CV_32FC1);

    // 像素索引 → 三维点坐标的映射（key = row * img_cols_ + col，扁平化索引）
    // 同一像素多个点时只保留 z 最高的点，与 height_mat 保持一致
    std::unordered_map<int, pcl::PointXYZI> pixel_to_point;

    // 像素索引 → 该像素对应的所有三维点（用于需要全部点的场景）
    std::map<int, std::vector<pcl::PointXYZI>> pixel_to_all_points;

    for (const auto &pt : cloud->points)
    {
      float range = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);

      // 距离过滤
      if (range < min_range_ || range > max_range_)
      {
        continue;
      }
      if (pt.z < -3.0) {
        continue;
      }
      // 计算垂直角: asin(z / range)
      float vert_angle = std::asin(pt.z / range);

      // 计算水平角: atan2(y, x)
      float horiz_angle = std::atan2(pt.y, pt.x);

      // 视场角过滤
      if (vert_angle > vert_max_rad_ || vert_angle < vert_min_rad_)
      {
        continue;
      }
      if (horiz_angle < horiz_min_rad_ || horiz_angle > -horiz_min_rad_)
      {
        continue;
      }

      // 计算像素坐标
      // row: 顶部(最大垂直角) = 第0行, 底部(最小垂直角) = 最后一行
      int row = static_cast<int>((vert_max_rad_ - vert_angle) / vert_res_rad_);
      int col = static_cast<int>((horiz_angle - horiz_min_rad_) / horiz_res_rad_);

      if (row < 0 || row >= img_rows_ || col < 0 || col >= img_cols_)
      {
        continue;
      }

      // 统计该像素点数并存储所有三维点（后续统一处理）
      count_mat.at<float>(row, col) += 1.0f;
      pixel_to_all_points[row * img_cols_ + col].push_back(pt);
    }

    // ===== 逐像素处理：按垂直角排序，取相邻z差值最大处较小角度的z =====
    for (auto &[pixel_idx, points] : pixel_to_all_points)
    {
      if (points.empty())
        continue;

      int r = pixel_idx / img_cols_;
      int c = pixel_idx % img_cols_;

      pcl::PointXYZI chosen;

      if (points.size() == 1)
      {
        // 单点：直接取
        chosen = points[0];
      }
      else
      {
        // 按垂直角（asin(z/range)）由大到小排列
        std::sort(points.begin(), points.end(),
                  [](const pcl::PointXYZI &a, const pcl::PointXYZI &b)
                  {
                    float ra = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
                    float rb = std::sqrt(b.x * b.x + b.y * b.y + b.z * b.z);
                    float va = std::asin(a.z / ra);
                    float vb = std::asin(b.z / rb);
                    return va > vb; // 降序：垂直角大的在前
                  });

        // 计算相邻三维点的 z 轴差值，找到差值绝对值最大的两个点
        float max_zdiff = -1.0f;
        size_t max_idx = 0;
        for (size_t i = 0; i < points.size() - 1; ++i)
        {
          float zdiff = std::abs(points[i].z - points[i + 1].z);
          if (zdiff > max_zdiff)
          {
            max_zdiff = zdiff;
            max_idx = i;
          }
        }
        // 取这两个点中垂直角较小的点（排序靠后的点，即 i+1）
        chosen = points[max_idx + 1];
      }

      height_mat.at<float>(r, c) = chosen.z;
      // x_mat.at<float>(r, c)      = chosen.x;
      // y_mat.at<float>(r, c)      = chosen.y;
      // intens_mat.at<float>(r, c) = chosen.intensity;
      pixel_to_point[pixel_idx] = chosen;
    }

    // 有效性掩码: +1.0 = 有点云数据, -1.0 = 无数据（空洞）
    cv::Mat valid_mask(img_rows_, img_cols_, CV_32FC1, cv::Scalar(-1.0f));

    // 生成高度图像 (8-bit)
    cv::Mat height_img = cv::Mat::zeros(img_rows_, img_cols_, CV_8UC1);

    for (int r = 0; r < img_rows_; ++r)
    {
      for (int c = 0; c < img_cols_; ++c)
      {
        float cnt = count_mat.at<float>(r, c);
        if (cnt > 0)
        {
          // 标记为有效像素 (+1)
          valid_mask.at<float>(r, c) = 1.0f;

          // 高度映射: z∈[height_bottom, 0] → z=0→灰度255, z=height_bottom→灰度122
          float h = height_mat.at<float>(r, c);
          float h_clamped = std::max(static_cast<float>(height_bottom_), std::min(0.0f, h));
          int gray = static_cast<int>(255.0f - 127.0f * h_clamped / static_cast<float>(height_bottom_));
          height_img.at<uint8_t>(r, c) = static_cast<uint8_t>(gray);
        }
      }
    }

    // ===== 标记每列从下往上第一个有效像素的三维坐标（雷达最跟前点，不作为斜坡边缘） =====
    std::vector<pcl::PointXYZI> bottom_first_points;
    for (int c = 0; c < img_cols_; ++c)
    {
      for (int r = img_rows_ - 1; r >= 0; --r)
      {
        if (valid_mask.at<float>(r, c) > 0.0f)
        {
          int idx = r * img_cols_ + c;
          auto it = pixel_to_point.find(idx);
          if (it != pixel_to_point.end())
          {
            bottom_first_points.push_back(it->second);
          }
          break;  // 只取每列第一个（最底部）的有效像素
        }
      }
    }

    // 辅助函数：判断一个点是否属于底部首点（三维坐标比较）
    auto isBottomFirst = [&](const pcl::PointXYZI &pt) -> bool
    {
      const float eps = 1e-6f;
      for (const auto &bp : bottom_first_points)
      {
        if (std::abs(bp.x - pt.x) < eps &&
            std::abs(bp.y - pt.y) < eps &&
            std::abs(bp.z - pt.z) < eps)
        {
          return true;
        }
      }
      return false;
    };

    // 保存原始高度图（膨胀前）
    cv::Mat height_img_raw = height_img.clone();

    // ===== 辅助函数：膨胀/修补后同步扩展 pixel_to_point =====
    // 对从无效变为有效的像素，从原始有效邻域中找到z最大的像素，复制其三维点
    auto expandPixelToPoint = [&](const cv::Mat &valid_before, int kernel_size)
    {
      int half_k = kernel_size / 2;
      for (int r = 0; r < img_rows_; ++r)
      {
        for (int c = 0; c < img_cols_; ++c)
        {
          // 只处理：之前无效(-1) → 现在有效(+1)
          if (valid_before.at<float>(r, c) > 0.0f || valid_mask.at<float>(r, c) <= 0.0f)
          {
            continue;
          }
          // 在邻域内搜索原始有效像素中 z 最大的
          float best_z = -std::numeric_limits<float>::max();
          int best_idx = -1;
          for (int dr = -half_k; dr <= half_k; ++dr)
          {
            for (int dc = -half_k; dc <= half_k; ++dc)
            {
              int nr = r + dr, nc = c + dc;
              if (nr < 0 || nr >= img_rows_ || nc < 0 || nc >= img_cols_)
                continue;
              if (valid_before.at<float>(nr, nc) <= 0.0f)
                continue;
              int idx = nr * img_cols_ + nc;
              auto it = pixel_to_point.find(idx);
              if (it == pixel_to_point.end())
                continue;
              if (it->second.z > best_z)
              {
                best_z = it->second.z;
                best_idx = idx;
              }
            }
          }
          if (best_idx >= 0)
          {
            pixel_to_point[r * img_cols_ + c] = pixel_to_point[best_idx];
          }
        }
      }
    };
    // ===== 辅助函数 END =====

    // 确保保存目录存在
    // std::filesystem::create_directories(save_dir_);
    // cv::imwrite(save_dir_ + "height_raw.png", height_img_raw);

    // 形态学膨胀填充间隙
    if (dilate_ks_ > 0)
    {
      cv::Mat valid_before_dilate = valid_mask.clone(); // 膨胀前快照

      cv::Mat kernel = cv::getStructuringElement(
          cv::MORPH_ELLIPSE, cv::Size(dilate_ks_, dilate_ks_));
      cv::dilate(height_img, height_img, kernel);

      // 膨胀填充的像素标记为有效 (+1)
      cv::dilate(valid_mask, valid_mask, kernel);
      valid_mask.setTo(1.0f, valid_mask > -0.5f); // 膨胀后 > -0.5 的设为 +1

      // 同步扩展 pixel_to_point（膨胀新增的像素）
      expandPixelToPoint(valid_before_dilate, dilate_ks_);
    }

    // 保存膨胀图（仅第一次膨胀后，修补前）
    // cv::Mat height_img_dilated_only = height_img.clone();
    // cv::imwrite(save_dir_ + "height_dilated.png", height_img_dilated_only);

    // 图像修补：用更大膨胀填充所有剩余空洞
    // fill_holes_ = false;  // 默认不填充所有空洞，避免过度膨胀
    if (fill_holes_)
    {
      cv::Mat valid_before_fill = valid_mask.clone(); // 修补前快照

      cv::Mat kernel2 = cv::getStructuringElement(
          cv::MORPH_ELLIPSE, cv::Size(7, 7));
      cv::dilate(height_img, height_img, kernel2);

      // 膨胀填充的像素标记为有效 (+1)
      cv::dilate(valid_mask, valid_mask, kernel2);
      valid_mask.setTo(1.0f, valid_mask > -0.5f);

      // 同步扩展 pixel_to_point（修补新增的像素）
      expandPixelToPoint(valid_before_fill, 5);
    }

    // 保存修补图（所有膨胀/修补处理后的最终高度图）
    // cv::imwrite(save_dir_ + "height_repaired.png", height_img);

    // 保存膨胀后高度图（膨胀填充后）
    cv::Mat height_img_dilated = height_img.clone();

    // 时间戳（图像和点云共用）
    auto timestamp = this->now();

    // ===== 边缘检测：提取高度突变（边坡边缘），红色叠加显示 =====
    cv::Mat edge_img;
    if (enable_edge_)
    {
      // 1. Canny 边缘检测（在膨胀填充后的 8-bit 高度图上）
      cv::Mat canny_edges;
      cv::Canny(height_img, canny_edges, canny_low_, canny_high_);

      // 1.5 屏蔽无效像素 (-1) 上的边缘：只保留有效像素 (+1) 上的边缘
      // {
      //   cv::Mat edge_mask(img_rows_, img_cols_, CV_8UC1);
      //   for (int r = 0; r < img_rows_; ++r)
      //     for (int c = 0; c < img_cols_; ++c)
      //       edge_mask.at<uint8_t>(r, c) = (valid_mask.at<float>(r, c) > 0.0f) ? 255 : 0;
      //   cv::bitwise_and(canny_edges, edge_mask, canny_edges);
      // }

      // 2. 形态学去噪：先闭运算连接断边
      cv::Mat kernel3 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
      cv::morphologyEx(canny_edges, canny_edges, cv::MORPH_CLOSE, kernel3);

      // 3. 去除小面积噪点（< 15 像素）
      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(canny_edges.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
      cv::Mat clean_edges = cv::Mat::zeros(canny_edges.size(), CV_8UC1);
      for (const auto &cnt : contours)
      {
        if (cv::contourArea(cnt) > 15.0)
        {
          cv::drawContours(clean_edges, std::vector<std::vector<cv::Point>>{cnt}, -1,
                           cv::Scalar(255), 1); // 画 1 像素宽轮廓线
        }
      }

      // 4. 将边缘以红色叠加到高度图上（BGR 格式）
      cv::Mat height_bgr;
      cv::cvtColor(height_img, height_bgr, cv::COLOR_GRAY2BGR);

      for (int r = 0; r < img_rows_; ++r)
      {
        for (int c = 0; c < img_cols_; ++c)
        {
          if (clean_edges.at<uint8_t>(r, c) > 0)
          {
            auto it_pt = pixel_to_point.find(r * img_cols_ + c);
            if (it_pt != pixel_to_point.end() && !isBottomFirst(it_pt->second))
            {
              height_bgr.at<cv::Vec3b>(r, c) = cv::Vec3b(0, 0, 255); // 红色
            }
          }
        }
      }

      edge_img = height_bgr;

      // ===== 5. 提取边缘点云：通过 pixel_to_point 获取边缘像素的代表点 =====
      pcl::PointCloud<pcl::PointXYZI> edge_cloud;

      for (int r = 0; r < img_rows_; ++r)
      {
        for (int c = 0; c < img_cols_; ++c)
        {
          if (clean_edges.at<uint8_t>(r, c) == 0)
            continue;

          auto it = pixel_to_point.find(r * img_cols_ + c);
          if (it == pixel_to_point.end())
            continue;

          // 跳过每列底部第一个有效像素（雷达最跟前点，不作为斜坡边缘）
          if (isBottomFirst(it->second))
            continue;

          // z < -2 的点是坡下的点，不作为坡上的边缘，过滤掉
          // if (it->second.z >= -2.0f || it->second.z <= -1.0f) {

          // 计算当前点的垂直角和水平角，过滤视场边缘点（不可能是边坡边缘）
          // float edge_range = std::sqrt(it->second.x * it->second.x + it->second.y * it->second.y + it->second.z * it->second.z);
          // if(edge_range < 1.0f) {
          //   float edge_vert_angle = std::asin(it->second.z / edge_range);
          //   float edge_horiz_angle = std::atan2(it->second.y, it->second.x);
          //   RCLCPP_INFO(this->get_logger(),
          //     "DEBUG pixel[%d,%d] point(%.2f,%.2f,%.2f): range=%.4f, vert_angle=%.4f rad (%.2f deg), horiz_angle=%.4f rad (%.2f deg)",
          //     r, c,
          //     it->second.x, it->second.y, it->second.z,
          //     edge_range,
          //     edge_vert_angle, edge_vert_angle * 180.0 / M_PI,
          //     edge_horiz_angle, edge_horiz_angle * 180.0 / M_PI);
          // }
          edge_cloud.push_back(it->second);
          // }
        }
      }

      // 查找边缘点云中最远点和最近点
      // if (!edge_cloud.empty()) {
      //   float min_range = std::numeric_limits<float>::max();
      //   float max_range = 0.0f;
      //   pcl::PointXYZI nearest_pt, farthest_pt;
      //   for (const auto& pt : edge_cloud) {
      //     float r = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
      //     if (r < min_range) { min_range = r; nearest_pt = pt; }
      //     if (r > max_range) { max_range = r; farthest_pt = pt; }
      //   }
      //   RCLCPP_INFO(this->get_logger(),
      //     "Edge cloud: %zu pts, nearest: (%.2f,%.2f,%.2f) range=%.2fm, farthest: (%.2f,%.2f,%.2f) range=%.2fm",
      //     edge_cloud.size(),
      //     nearest_pt.x, nearest_pt.y, nearest_pt.z, min_range,
      //     farthest_pt.x, farthest_pt.y, farthest_pt.z, max_range);
      // }

      // 发布边缘点云
      if (!edge_cloud.empty())
      {
        sensor_msgs::msg::PointCloud2 edge_cloud_msg;
        pcl::toROSMsg(edge_cloud, edge_cloud_msg);
        edge_cloud_msg.header.stamp = timestamp;
        edge_cloud_msg.header.frame_id = frame_id_;
        pub_edge_cloud_->publish(edge_cloud_msg);
      }
    }
    // ===== 边缘检测 END =====

    // ===== 发布三种图像 =====

    // 1. 原始高度图（膨胀前，mono8）
    auto raw_msg = cv_bridge::CvImage(
                       std_msgs::msg::Header(), "mono8", height_img_raw)
                       .toImageMsg();
    raw_msg->header.stamp = timestamp;
    raw_msg->header.frame_id = frame_id_;
    pub_height_img_->publish(*raw_msg);

    // 2. 膨胀后高度图（膨胀填充后，mono8）
    auto dilated_msg = cv_bridge::CvImage(
                           std_msgs::msg::Header(), "mono8", height_img_dilated)
                           .toImageMsg();
    dilated_msg->header.stamp = timestamp;
    dilated_msg->header.frame_id = frame_id_;
    pub_dilated_height_img_->publish(*dilated_msg);

    // 3. 带边缘叠加的高度图（BGR 格式，红色=边坡边缘）
    if (enable_edge_ && !edge_img.empty())
    {
      auto edge_msg = cv_bridge::CvImage(
                          std_msgs::msg::Header(), "bgr8", edge_img)
                          .toImageMsg();
      edge_msg->header.stamp = timestamp;
      edge_msg->header.frame_id = frame_id_;
      pub_edge_img_->publish(*edge_msg);
    }
    else if (!enable_edge_)
    {
      // 边缘检测关闭时也发布高度图（BGR 灰度）
      cv::Mat height_bgr;
      cv::cvtColor(height_img_dilated, height_bgr, cv::COLOR_GRAY2BGR);
      auto edge_msg = cv_bridge::CvImage(
                          std_msgs::msg::Header(), "bgr8", height_bgr)
                          .toImageMsg();
      edge_msg->header.stamp = timestamp;
      edge_msg->header.frame_id = frame_id_;
      pub_edge_img_->publish(*edge_msg);
    }
  }

  // 订阅和发布
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_height_img_;         // 原始高度图
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_dilated_height_img_; // 膨胀后高度图
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_edge_img_;           // 带边缘叠加的高度图
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_edge_cloud_;   // 边缘点云

  // 参数
  std::string frame_id_;
  double horiz_fov_deg_, vert_fov_deg_;
  double horiz_res_deg_, vert_res_deg_;
  double max_range_, min_range_;
  double height_top_, height_bottom_;
  int dilate_ks_;
  bool fill_holes_;
  double voxel_leaf_size_;
  int img_cols_, img_rows_;
  double horiz_min_rad_, vert_max_rad_, vert_min_rad_;
  double horiz_res_rad_, vert_res_rad_;

  // 边缘检测参数
  bool enable_edge_;
  int canny_low_, canny_high_;

  // 本地保存
  std::string save_dir_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarImageProjection>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
