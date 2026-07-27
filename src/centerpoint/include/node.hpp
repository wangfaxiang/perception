#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64.hpp>
#include "box_msg/msg/boxs.hpp"
#include <std_msgs/msg/string.hpp>
#include "common.h"
#include "centerpoint.h"
#include <nlohmann/json.hpp>

using json = nlohmann::json;
cudaStream_t stream ;
CenterPoint* centerpoint;
float *d_points;
class CenterpointRos : public rclcpp::Node
{
public:
    CenterpointRos();

private:
    void onPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg);
    void onUi2Ros(const std_msgs::msg::String::ConstSharedPtr input_msg);
    Params params;
    float conf_thres;
    std::string detect;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointsubscribe;
    rclcpp::Publisher<box_msg::msg::Boxs>::SharedPtr publisher_pose;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscribe_ui2Ros;
};

