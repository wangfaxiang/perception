/**
 * @file lidar_status_monitor_node.cpp
 * @brief 雷达数据流监测节点：订阅雷达点云话题，周期发布 LidarStatus，
 *        跟踪雷达是否有数据 / 是否断流（网线断开、电源故障等）。
 *
 * 输入  ：点云话题（默认 rslidar_points，挂节点 namespace 下）
 * 输出  ：lidar_status_monitor/msg/LidarStatus（默认 lidar_status，挂节点 namespace 下）
 *
 * 判据  ：自上次收到点云起，超过 timeout_s 秒未再收到 -> STATE_TIMEOUT（断流）；
 *         重新收到数据 -> STATE_OK（恢复）；从未收到过任何数据 -> STATE_UNKNOWN。
 *         状态沿（UNKNOWN/OK -> TIMEOUT 视为故障，TIMEOUT -> OK 视为恢复）会在日志中打印，
 *         后续 CAN 上报节点可订阅本话题，按同样沿检测发送故障/恢复帧。
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <lidar_status_monitor/msg/lidar_status.hpp>

#include <chrono>
#include <memory>
#include <string>

class LidarStatusMonitorNode : public rclcpp::Node
{
public:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using LidarStatus = lidar_status_monitor::msg::LidarStatus;

  explicit LidarStatusMonitorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("lidar_status_monitor_node", options),
    has_data_(false),
    last_state_(LidarStatus::STATE_UNKNOWN)
  {
    this->declare_parameter<std::string>("lidar_name", "lidar");
    this->declare_parameter<std::string>("input_topic", "rslidar_points");
    this->declare_parameter<std::string>("status_topic", "lidar_status");
    this->declare_parameter<std::string>("frame_id", "base_link");
    this->declare_parameter<double>("timeout_s", 1.0);
    this->declare_parameter<double>("publish_hz", 10.0);

    this->get_parameter("lidar_name", lidar_name_);
    this->get_parameter("input_topic", input_topic_);
    this->get_parameter("status_topic", status_topic_);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("timeout_s", timeout_s_);
    this->get_parameter("publish_hz", publish_hz_);

    // 镭神 LS 驱动以默认可靠 QoS(10) 发布点云，订阅端必须一致（不能用 best_effort/SensorDataQoS）
    sub_ = this->create_subscription<PointCloud2>(
      input_topic_, rclcpp::QoS(10),
      [this](const PointCloud2::SharedPtr msg) { this->onCloud(msg); });

    pub_ = this->create_publisher<LidarStatus>(status_topic_, 10);

    const double period_s = 1.0 / publish_hz_;
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(period_s),
      [this]() { this->publishStatus(); });

    RCLCPP_INFO(this->get_logger(),
      "[%s] monitoring '%s' -> '%s' (timeout=%.2fs, publish=%.1fHz)",
      lidar_name_.c_str(), input_topic_.c_str(), status_topic_.c_str(),
      timeout_s_, publish_hz_);
  }

private:
  void onCloud(const PointCloud2::SharedPtr /*msg*/)
  {
    last_receive_time_ = this->now();
    if (!has_data_) {
      has_data_ = true;
      RCLCPP_INFO(this->get_logger(), "[%s] first point cloud received", lidar_name_.c_str());
    }
  }

  void publishStatus()
  {
    LidarStatus status;
    status.header.stamp = this->now();
    status.header.frame_id = frame_id_;
    status.lidar_name = lidar_name_;
    status.has_data = has_data_;

    if (!has_data_) {
      status.state = LidarStatus::STATE_UNKNOWN;
      status.last_data_age_s = -1.0f;
    } else {
      const double age_s = (this->now() - last_receive_time_).seconds();
      status.last_data_age_s = static_cast<float>(age_s);
      status.state = (age_s > timeout_s_)
        ? LidarStatus::STATE_TIMEOUT : LidarStatus::STATE_OK;
    }

    if (status.state != last_state_) {
      if (status.state == LidarStatus::STATE_TIMEOUT) {
        RCLCPP_WARN(this->get_logger(),
          "[%s] STREAM TIMEOUT (no data for %.2fs)", lidar_name_.c_str(), ageOf(status));
      } else if (status.state == LidarStatus::STATE_OK) {
        RCLCPP_INFO(this->get_logger(), "[%s] stream recovered", lidar_name_.c_str());
      }
      last_state_ = status.state;
    }

    pub_->publish(status);
  }

  static double ageOf(const LidarStatus & s)
  {
    return static_cast<double>(s.last_data_age_s);
  }

  std::string lidar_name_;
  std::string input_topic_;
  std::string status_topic_;
  std::string frame_id_;
  double timeout_s_{1.0};
  double publish_hz_{10.0};

  bool has_data_{false};
  uint8_t last_state_{LidarStatus::STATE_UNKNOWN};
  rclcpp::Time last_receive_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<LidarStatus>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarStatusMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
