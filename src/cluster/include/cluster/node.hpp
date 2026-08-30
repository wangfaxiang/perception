#include "rclcpp/rclcpp.hpp"


#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/common/common.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/features/normal_3d.h>
#include "box_msg/msg/boxs.hpp"
#include "box_msg/msg/box.hpp"
#include <std_msgs/msg/string.hpp>
class Cluster : public rclcpp::Node
{
public:
    Cluster();

private:
    void onPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg);
    void onSubjson(const std_msgs::msg::String::ConstSharedPtr input_msg);
    float cluster_tolerance;
    int min_cluster_size;
    int max_cluster_size;
    int region_rowing_min_cluster_size;
    int region_rowing_max_cluster_size;
    float smoothness_threshold;
    float curvature_threshold;
    int number_of_neighbours;
    float radius_search;
    std::string euclidean_cluster;
    std::string region_rowing;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subjson;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointsubscribe;
    rclcpp::Publisher<box_msg::msg::Boxs>::SharedPtr publisher_cluster_euclidean;
    rclcpp::Publisher<box_msg::msg::Boxs>::SharedPtr publisher_cluster_region_rowing;
};