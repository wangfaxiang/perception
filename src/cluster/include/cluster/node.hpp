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
#include <pcl/filters/voxel_grid.h>
#include <pcl/PointIndices.h>
#include "box_msg/msg/boxs.hpp"
#include "box_msg/msg/box.hpp"
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
class Cluster : public rclcpp::Node
{
public:
    Cluster();

private:
    void onPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg);
    void onSubjson(const std_msgs::msg::String::ConstSharedPtr input_msg);
    visualization_msgs::msg::MarkerArray boxsToMarkerArray(const box_msg::msg::Boxs& boxs, const std::string& frame_id);
    box_msg::msg::Boxs clustersToBoxs(const std::vector<pcl::PointIndices>& cluster_indices,
                                      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);
    void printBoxInfo(const box_msg::msg::Boxs& boxarray);
    float cluster_tolerance;
    int min_cluster_size;
    int max_cluster_size;
    int region_rowing_min_cluster_size;
    int region_rowing_max_cluster_size;
    float smoothness_threshold;
    float curvature_threshold;
    int number_of_neighbours;
    float radius_search;
    float leaf_size;
    std::string lidar_name;
    std::string downsample;
    std::string euclidean_cluster;
    bool region_rowing;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subjson;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointsubscribe;
    rclcpp::Publisher<box_msg::msg::Boxs>::SharedPtr publisher_cluster_euclidean;
    rclcpp::Publisher<box_msg::msg::Boxs>::SharedPtr publisher_cluster_region_rowing;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_cluster_euclidean_marker;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_cluster_region_rowing_marker;
};