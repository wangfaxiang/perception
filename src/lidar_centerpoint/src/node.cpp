// Copyright 2021 TIER IV, Inc.
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

#include "lidar_centerpoint/node.hpp"

#include <lidar_centerpoint/centerpoint_config.hpp>
#include <lidar_centerpoint/ros_utils.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace centerpoint
{

LidarCenterPointNode::LidarCenterPointNode(const rclcpp::NodeOptions & node_options)
: Node("lidar_centerpoint", node_options), tf_buffer_(this->get_clock())
{
  using std::placeholders::_1;

  // ---- I/O parameters ----
  input_topic_front_ = this->declare_parameter("input_topic_front", "/front/rslidar_points");
  input_topic_rear_ = this->declare_parameter("input_topic_rear", "/rear/rslidar_points");
  enable_rear_ = this->declare_parameter("enable_rear", true);
  target_frame_ = this->declare_parameter("target_frame", "front_rslidar");
  output_topic_ = this->declare_parameter("output_topic", "/centerpoint_boxs_no_velocity");
  output_frame_id_ = this->declare_parameter("output_frame_id", "");
  sync_tolerance_ms_ = static_cast<int>(this->declare_parameter("sync_tolerance_ms", int64_t{100}));
  score_threshold_ = static_cast<float>(this->declare_parameter("score_threshold", 0.35));
  class_names_ = this->declare_parameter(
    "class_names",
    std::vector<std::string>{"CAR", "TRUCK", "BUS", "BICYCLE", "PEDESTRIAN"});
  {
    const auto target_class_ids =
      this->declare_parameter("target_class_ids", std::vector<int64_t>{});
    target_class_ids_.assign(target_class_ids.begin(), target_class_ids.end());
  }

  // ---- Model parameters ----
  const std::string model_dir = this->declare_parameter("model_dir", "");
  const auto resolve = [&model_dir](const std::string & p) {
    if (!model_dir.empty() && !p.empty() && p[0] != '/') {
      return model_dir + "/" + p;
    }
    return p;
  };
  const std::string encoder_onnx_path =
    resolve(this->declare_parameter("encoder_onnx_path", "model/encoder.onnx"));
  const std::string encoder_engine_path =
    resolve(this->declare_parameter("encoder_engine_path", "model/encoder.plan"));
  const std::string head_onnx_path =
    resolve(this->declare_parameter("head_onnx_path", "model/head.onnx"));
  const std::string head_engine_path =
    resolve(this->declare_parameter("head_engine_path", "model/head.plan"));
  const std::string trt_precision = this->declare_parameter("trt_precision", "fp16");
  const bool build_only = this->declare_parameter("build_only", false);

  // ---- Network / post-process parameters ----
  const auto point_feature_size =
    static_cast<std::size_t>(this->declare_parameter("point_feature_size", int64_t{4}));
  const auto max_voxel_size =
    static_cast<std::size_t>(this->declare_parameter("max_voxel_size", int64_t{40000}));
  const auto point_cloud_range = this->declare_parameter(
    "point_cloud_range", std::vector<double>{-76.8, -76.8, -4.0, 76.8, 76.8, 6.0});
  const auto voxel_size =
    this->declare_parameter("voxel_size", std::vector<double>{0.32, 0.32, 10.0});
  const auto downsample_factor =
    static_cast<std::size_t>(this->declare_parameter("downsample_factor", int64_t{1}));
  const auto encoder_in_feature_size =
    static_cast<std::size_t>(this->declare_parameter("encoder_in_feature_size", int64_t{9}));
  const auto circle_nms_dist_threshold =
    static_cast<float>(this->declare_parameter("circle_nms_dist_threshold", 0.5));
  const auto yaw_norm_thresholds = this->declare_parameter(
    "yaw_norm_thresholds", std::vector<double>{0.3, 0.3, 0.3, 0.3, 0.0});
  const std::string densification_world_frame_id =
    this->declare_parameter("densification_world_frame_id", "map");
  const auto densification_num_past_frames = static_cast<unsigned int>(
    this->declare_parameter("densification_num_past_frames", int64_t{0}));

  NetworkParam encoder_param(encoder_onnx_path, encoder_engine_path, trt_precision);
  NetworkParam head_param(head_onnx_path, head_engine_path, trt_precision);
  DensificationParam densification_param(densification_world_frame_id, densification_num_past_frames);

  CenterPointConfig config(
    class_names_.size(), point_feature_size, max_voxel_size, point_cloud_range, voxel_size,
    downsample_factor, encoder_in_feature_size, score_threshold_, circle_nms_dist_threshold,
    yaw_norm_thresholds);

  detector_ptr_ =
    std::make_unique<CenterPointTRT>(encoder_param, head_param, densification_param, config);

  front_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_front_, rclcpp::SensorDataQoS{}.keep_last(1),
    std::bind(&LidarCenterPointNode::frontCloudCallback, this, _1));
  if (enable_rear_) {
    rear_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_rear_, rclcpp::SensorDataQoS{}.keep_last(1),
      std::bind(&LidarCenterPointNode::rearCloudCallback, this, _1));
  }
  objects_pub_ = this->create_publisher<box_msg::msg::Boxs>(output_topic_, rclcpp::QoS{1});

  RCLCPP_INFO(
    this->get_logger(), "lidar_centerpoint started (front=%s, rear=%s, target_frame=%s)",
    input_topic_front_.c_str(), enable_rear_ ? input_topic_rear_.c_str() : "disabled",
    target_frame_.c_str());

  if (build_only) {
    RCLCPP_INFO(this->get_logger(), "TensorRT engine(s) built, shutting down.");
    rclcpp::shutdown();
  }
}

bool LidarCenterPointNode::transformToTarget(
  const sensor_msgs::msg::PointCloud2 & in, sensor_msgs::msg::PointCloud2 & out)
{
  if (target_frame_.empty() || in.header.frame_id == target_frame_) {
    out = in;
    return true;
  }
  try {
    const auto tf = tf_buffer_.lookupTransform(
      target_frame_, in.header.frame_id, in.header.stamp, rclcpp::Duration::from_seconds(0.5));
    tf2::doTransform(in, out, tf);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "Failed to transform %s -> %s: %s",
      in.header.frame_id.c_str(), target_frame_.c_str(), ex.what());
    return false;
  }
}

bool LidarCenterPointNode::concatPointClouds(
  const sensor_msgs::msg::PointCloud2 & front, const sensor_msgs::msg::PointCloud2 & rear,
  sensor_msgs::msg::PointCloud2 & merged)
{
  if (!pcl::concatenatePointCloud(front, rear, merged)) {
    RCLCPP_WARN(this->get_logger(), "Failed to concatenate point clouds");
    return false;
  }
  merged.header = front.header;
  return true;
}

void LidarCenterPointNode::rearCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  sensor_msgs::msg::PointCloud2 transformed;
  if (transformToTarget(*msg, transformed)) {
    latest_rear_ = transformed;
  }
}

void LidarCenterPointNode::frontCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  sensor_msgs::msg::PointCloud2 front_transformed;
  if (!transformToTarget(*msg, front_transformed)) {
    return;
  }

  sensor_msgs::msg::PointCloud2 input = front_transformed;
  if (enable_rear_ && latest_rear_.has_value()) {
    const double front_t = rclcpp::Time(front_transformed.header.stamp).seconds();
    const double rear_t = rclcpp::Time(latest_rear_->header.stamp).seconds();
    const double tolerance = static_cast<double>(sync_tolerance_ms_) / 1000.0;
    if (std::abs(front_t - rear_t) <= tolerance) {
      sensor_msgs::msg::PointCloud2 merged;
      if (concatPointClouds(front_transformed, *latest_rear_, merged)) {
        input = merged;
      }
    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "front/rear timestamps differ by %.3fs (> %.3fs), running on front only",
        front_t - rear_t, tolerance);
    }
  }

  detect(input);
}

void LidarCenterPointNode::detect(const sensor_msgs::msg::PointCloud2 & input_msg)
{
  const auto sub_count =
    objects_pub_->get_subscription_count() + objects_pub_->get_intra_process_subscription_count();
  if (sub_count < 1) {
    return;
  }

  std::vector<Box3D> det_boxes3d;
  if (!detector_ptr_->detect(input_msg, tf_buffer_, det_boxes3d)) {
    return;
  }

  box_msg::msg::Boxs output_msg;
  output_msg.header.stamp = input_msg.header.stamp;
  output_msg.header.frame_id = output_frame_id_.empty() ? target_frame_ : output_frame_id_;

  for (const auto & box3d : det_boxes3d) {
    if (box3d.score < score_threshold_) {
      continue;
    }
    if (!target_class_ids_.empty() &&
        std::find(target_class_ids_.begin(), target_class_ids_.end(), box3d.label) ==
          target_class_ids_.end()) {
      continue;
    }
    box_msg::msg::Box box;
    box3DToBox(box3d, class_names_, box);
    output_msg.box.push_back(box);
  }

  objects_pub_->publish(output_msg);
}

}  // namespace centerpoint

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<centerpoint::LidarCenterPointNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
