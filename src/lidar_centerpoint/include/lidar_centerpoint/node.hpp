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

#ifndef LIDAR_CENTERPOINT__NODE_HPP_
#define LIDAR_CENTERPOINT__NODE_HPP_

#include <lidar_centerpoint/centerpoint_trt.hpp>

#include <box_msg/msg/boxs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace centerpoint
{

class LidarCenterPointNode : public rclcpp::Node
{
public:
  explicit LidarCenterPointNode(const rclcpp::NodeOptions & node_options);

private:
  void frontCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void rearCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  bool transformToTarget(
    const sensor_msgs::msg::PointCloud2 & in, sensor_msgs::msg::PointCloud2 & out);
  bool concatPointClouds(
    const sensor_msgs::msg::PointCloud2 & front, const sensor_msgs::msg::PointCloud2 & rear,
    sensor_msgs::msg::PointCloud2 & merged);
  void detect(const sensor_msgs::msg::PointCloud2 & input_msg);

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_{tf_buffer_};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr front_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr rear_sub_;
  rclcpp::Publisher<box_msg::msg::Boxs>::SharedPtr objects_pub_;

  std::unique_ptr<CenterPointTRT> detector_ptr_{nullptr};

  // parameters
  std::string input_topic_front_;
  std::string input_topic_rear_;
  bool enable_rear_{true};
  std::string target_frame_;
  std::string output_topic_;
  std::string output_frame_id_;
  int sync_tolerance_ms_{100};
  float score_threshold_{0.35f};
  std::vector<std::string> class_names_;
  std::vector<int> target_class_ids_;

  // latest rear cloud already transformed into target_frame_
  std::optional<sensor_msgs::msg::PointCloud2> latest_rear_;
};

}  // namespace centerpoint

#endif  // LIDAR_CENTERPOINT__NODE_HPP_
