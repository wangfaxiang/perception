/**
 * @file edge_warning_fusion_node.cpp
 * @brief 前后雷达边坡告警融合 → 契约话题 /perception/edge_warning（AutoDTH 契约 §3.1/§4/§5.2）
 *
 * 本节点是该契约话题的唯一发布者:
 *   订阅前后两侧检测实例的「单雷达原始告警」（~/edge_warning_raw；被门控或断流的一侧为
 *   999.0 + SAFE），每个周期取等级更高的一方发布，同级取距离更近者。
 *
 * 与运动方向解耦: 门控在检测实例内完成（见 motion_state.hpp）——前进时后侧原始告警恒为
 * SAFE，取高等级自然只体现前侧结果；后退反之；停车/原地旋转时两侧都在检测，
 * 取高等级即「综合前后雷达警告级别」。
 *
 * 周期发布（默认 10Hz，契约 §4.3 要求 ≥5Hz）；某侧断流超时按安全默认值参与比较，
 * 两侧全断流即输出安全默认值（999.0 / SAFE）。
 *
 * 日志：每秒一条，带当前运动方向标注（前进/后退/原地旋转/停车/无信号）便于定位：
 *   "融合: 运动=前进 前[DANGER 4.0m] 后[SAFE 999.0m] → DANGER 4.0m (取前)"
 */

#include <rclcpp/rclcpp.hpp>
#include <dth_messages/msg/edge_warning.hpp>

#include "motion_state.hpp"

#include <chrono>
#include <memory>
#include <string>

class EdgeWarningFusion : public rclcpp::Node
{
public:
  using EdgeWarning = dth_messages::msg::EdgeWarning;

  EdgeWarningFusion() : Node("edge_warning_fusion")
  {
    this->declare_parameter<std::string>("edge_warning_topic", "/perception/edge_warning");
    this->declare_parameter<std::string>("front_source_topic",
                                         "/pothole_detection_front/edge_warning_raw");
    this->declare_parameter<std::string>("rear_source_topic",
                                         "/pothole_detection_rear/edge_warning_raw");
    this->declare_parameter<std::string>("frame_id", "base_link");
    this->declare_parameter("edge_heartbeat_hz", 10.0);  // 契约 §4.3 必须 ≥5Hz，建议 10Hz
    this->declare_parameter("source_timeout_s", 0.5);    // 单侧原始告警断流超时（秒）

    this->get_parameter("edge_warning_topic", edge_topic_);
    this->get_parameter("front_source_topic", front_topic_);
    this->get_parameter("rear_source_topic", rear_topic_);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("edge_heartbeat_hz", edge_heartbeat_hz_);
    this->get_parameter("source_timeout_s", source_timeout_s_);
    if (!(edge_heartbeat_hz_ > 0.0)) edge_heartbeat_hz_ = 10.0;

    front_.name = "front";
    rear_.name = "rear";

    // 运动方向镜像：仅用于日志标注（前进/后退/原地旋转/停车/无信号），
    // 融合本身与方向解耦——门控已在检测实例内完成（lidar_name 传空 = 不做门控）
    gate_ = std::make_unique<pothole_detection::MotionGate>(*this, "");

    pub_ = this->create_publisher<EdgeWarning>(edge_topic_, 10);  // 契约 §4.2 默认 QoS
    sub_front_ = this->create_subscription<EdgeWarning>(
        front_topic_, rclcpp::QoS(10),
        [this](EdgeWarning::SharedPtr msg) { onSource(msg, front_); });
    sub_rear_ = this->create_subscription<EdgeWarning>(
        rear_topic_, rclcpp::QoS(10),
        [this](EdgeWarning::SharedPtr msg) { onSource(msg, rear_); });

    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / edge_heartbeat_hz_),
        std::bind(&EdgeWarningFusion::publishFused, this));

    RCLCPP_INFO(this->get_logger(),
                "Edge warning fusion started (front=%s, rear=%s → %s, %.1fHz, source_timeout=%.2fs, "
                "motion_cmd=%s)",
                front_topic_.c_str(), rear_topic_.c_str(), edge_topic_.c_str(),
                edge_heartbeat_hz_, source_timeout_s_, gate_->cmdTopic().c_str());
  }

private:
  /// 一路来源（前 / 后）的最新原始告警
  struct Source
  {
    const char *name{""};  // 日志用
    EdgeWarning msg;       // 最近一帧
    rclcpp::Time stamp;    // 到达时刻（节点时钟）
    bool seen{false};      // 是否收到过
    bool stale_warned{false};
  };

  void onSource(const EdgeWarning::SharedPtr msg, Source &src)
  {
    src.msg = *msg;
    src.stamp = this->now();
    src.seen = true;
  }

  /// 取某侧的可用告警：断流/未收到 → 安全默认值（999.0 / SAFE），断流与恢复各只提示一次
  EdgeWarning usable(Source &src, bool &stale)
  {
    stale = !src.seen || (this->now() - src.stamp).seconds() > source_timeout_s_;
    if (!stale)
    {
      if (src.stale_warned)
      {
        RCLCPP_INFO(this->get_logger(), "原始告警 %s 恢复", src.name);
        src.stale_warned = false;
      }
      return src.msg;
    }

    if (!src.stale_warned)
    {
      RCLCPP_WARN(this->get_logger(), "原始告警 %s 断流超时(>%.2fs)，按安全默认值参与融合",
                  src.name, source_timeout_s_);
      src.stale_warned = true;
    }

    EdgeWarning safe;
    safe.warning = false;
    safe.dist_to_edge_m = 999.0f;  // 无数据哨兵值（契约 §9）
    safe.level = EdgeWarning::LEVEL_SAFE;
    return safe;
  }

  /// 周期融合发布（同时充当契约 §4.3 的心跳）
  void publishFused()
  {
    bool front_stale = false;
    bool rear_stale = false;
    const EdgeWarning front = usable(front_, front_stale);
    const EdgeWarning rear = usable(rear_, rear_stale);

    // 等级高者胜；同级取距离更近者（无数据的 999.0 自然落选）
    const bool front_wins = front.level > rear.level ||
                            (front.level == rear.level &&
                             front.dist_to_edge_m <= rear.dist_to_edge_m);
    EdgeWarning out = front_wins ? front : rear;

    out.header.stamp = this->now();  // 契约 §4.3: 每帧填节点时钟 now()（尊重 use_sim_time）
    out.header.frame_id = frame_id_;
    out.warning = (out.level != EdgeWarning::LEVEL_SAFE);  // 契约 §5.2 固定映射

    pub_->publish(out);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "融合: 运动=%s 前[%s %.1fm%s] 后[%s %.1fm%s] → %s %.1fm (%s)",
                         gate_->shortName(),
                         levelName(front.level), front.dist_to_edge_m, front_stale ? "/断流" : "",
                         levelName(rear.level), rear.dist_to_edge_m, rear_stale ? "/断流" : "",
                         levelName(out.level), out.dist_to_edge_m, front_wins ? "取前" : "取后");
  }

  static const char *levelName(uint8_t level)
  {
    switch (level)
    {
      case EdgeWarning::LEVEL_DANGER:  return "DANGER";
      case EdgeWarning::LEVEL_CAUTION: return "CAUTION";
      default:                         return "SAFE";
    }
  }

  rclcpp::Publisher<EdgeWarning>::SharedPtr pub_;
  rclcpp::Subscription<EdgeWarning>::SharedPtr sub_front_, sub_rear_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<pothole_detection::MotionGate> gate_;  // 仅用于日志标注运动方向

  Source front_, rear_;
  std::string edge_topic_, front_topic_, rear_topic_, frame_id_;
  double edge_heartbeat_hz_{10.0}, source_timeout_s_{0.5};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EdgeWarningFusion>());
  rclcpp::shutdown();
  return 0;
}
