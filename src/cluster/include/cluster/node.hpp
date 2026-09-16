#include "rclcpp/rclcpp.hpp"


#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
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
#include <dth_messages/msg/obstacle_warning.hpp>
// 共用件：运动方向判定与定向检测门控（与 pothole_detection 边坡检测同一份实现）
#include <perception_common/motion_state.hpp>
class Cluster : public rclcpp::Node
{
public:
    Cluster();

private:
    // 每个保留簇的几何摘要（与 Boxs.box 一一对应、同序），用于挑最近威胁物
    struct BoxGeom
    {
        float near_x{0.0f};      // 距雷达最近的点（水平距离最小）
        float near_y{0.0f};
        float near_z{0.0f};
        float near_dist{0.0f};   // hypot(near_x, near_y)，与契约 dist_m 自洽（§5.3）
        size_t points{0};        // 簇内点数
    };

    void onPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg);
    void onSubjson(const std_msgs::msg::String::ConstSharedPtr input_msg);
    visualization_msgs::msg::MarkerArray boxsToMarkerArray(const box_msg::msg::Boxs& boxs, const std::string& frame_id);
    box_msg::msg::Boxs clustersToBoxs(const std::vector<pcl::PointIndices>& cluster_indices,
                                      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                                      std::vector<BoxGeom>* geoms);
    void printBoxInfo(const box_msg::msg::Boxs& boxarray);

    // ---- 障碍物告警（契约 §3.1/§5.3）：本实例只发「单雷达原始告警」到节点私有话题，
    // 由 perception_common 包的 warning_fusion 取类别优先者合并为 /perception/obstacle_warning ----
    void publishObstacleWarning(const box_msg::msg::Boxs& boxarray,
                               const std::vector<BoxGeom>& geoms,
                               const std::string& frame_id);
    void publishSafeObstacle();
    void publishObstacleLocked(dth_messages::msg::ObstacleWarning& msg);
    void onObstacleHeartbeat();
    std::string describeThreat(const BoxGeom& geom, const box_msg::msg::Box& box) const;
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
    rclcpp::Publisher<dth_messages::msg::ObstacleWarning>::SharedPtr publisher_obstacle_warning;

    // ---- 障碍物告警状态与心跳 ----
    std::string obstacle_topic_;
    std::string obstacle_source_;   // 哪一路聚类结果作为告警来源: euclidean / region_rowing
    std::string frame_id_;          // 无点云帧时的占位 frame_id（有点云时用输入点云的 frame）
    double obstacle_heartbeat_hz_{10.0};
    double obstacle_heartbeat_period_s_{0.1};
    double obstacle_timeout_s_{0.5};
    double obstacle_warn_dist_{0.0};  // 威胁物上报距离（米），来源: 共享参数文件（无代码默认值）
    int obstacle_type_{3};            // 强制指定的类别（1 人员 / 2 车辆 / 3 其他）；cluster 无分类能力
    rclcpp::TimerBase::SharedPtr obstacle_heartbeat_timer_;
    std::mutex obstacle_mutex_;
    dth_messages::msg::ObstacleWarning last_obstacle_;
    rclcpp::Time last_obstacle_publish_time_;
    rclcpp::Time last_cloud_time_;
    bool cloud_seen_{false};
    bool obstacle_stale_warned_{false};

    // ---- 运动方向定向检测门控（共用头 perception_common/motion_state.hpp）----
    std::unique_ptr<perception_common::MotionGate> gate_;
    bool motion_gate_enable_{true};
    perception_common::MotionState last_motion_{perception_common::MotionState::kStopped};
};