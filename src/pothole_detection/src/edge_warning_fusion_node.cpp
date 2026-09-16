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
 * 急停联动（契约 §5.1/§5.2/§7）：同一周期内，若融合等级达 LEVEL_DANGER，则本节点同时
 * 发布契约话题 /perception/stop_command（stop=true, reason=REASON_EDGE_DANGER）——
 * 契约 §5.2 要求「LEVEL_DANGER 应伴随 stop=true」；CAUTION 仅告警不联动。
 * 急停锁存与解锁由系统侧 autodth_cmd_gate 负责，本节点只维护 stop 电平：风险持续期间
 * 持续发布 stop=true，等级回退后下一周期即恢复 stop=false（操作员复位的前置条件 §7.3）。
 *
 * 日志：每秒一条，带当前运动方向标注（前进/后退/原地旋转/停车/无信号）便于定位：
 *   "融合: 运动=前进 前[DANGER 4.0m] 后[SAFE 999.0m] → DANGER 4.0m (取前)，急停=ON"
 * 急停状态翻转时另有一条独立日志（触发用 WARN、解除用 INFO）。
 */

#include <rclcpp/rclcpp.hpp>
#include <dth_messages/msg/edge_warning.hpp>
#include <dth_messages/msg/stop_command.hpp>

#include "motion_state.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>

class EdgeWarningFusion : public rclcpp::Node
{
public:
  using EdgeWarning = dth_messages::msg::EdgeWarning;
  using StopCommand = dth_messages::msg::StopCommand;

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

    // 急停联动（契约 §3.3/§5.1/§5.2）
    this->declare_parameter<std::string>("stop_command_topic", "/perception/stop_command");
    this->declare_parameter("stop_on_danger", true);     // false = 只告警不联动急停
    // confidence 换算用的 DANGER 阈值：唯一真源 = config/params.yaml 的 /** 共享段（与检测实例同源）
    this->declare_parameter("edge_danger_dist", rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("stop_debounce_frames", 2);  // 连续 N 帧原始告警 DANGER 才置 stop

    this->get_parameter("edge_warning_topic", edge_topic_);
    this->get_parameter("front_source_topic", front_topic_);
    this->get_parameter("rear_source_topic", rear_topic_);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("edge_heartbeat_hz", edge_heartbeat_hz_);
    this->get_parameter("source_timeout_s", source_timeout_s_);
    this->get_parameter("stop_command_topic", stop_topic_);
    this->get_parameter("stop_on_danger", stop_on_danger_);
    this->get_parameter("edge_danger_dist", edge_danger_dist_);
    this->get_parameter("stop_debounce_frames", stop_debounce_frames_);
    if (!(edge_heartbeat_hz_ > 0.0)) edge_heartbeat_hz_ = 10.0;
    if (!(edge_danger_dist_ > 0.0))
    {
      RCLCPP_FATAL(this->get_logger(),
                   "edge_danger_dist=%.1f 非法：须由 config/params.yaml 的 /** 共享段提供",
                   edge_danger_dist_);
      throw std::runtime_error("invalid edge_danger_dist");
    }
    if (stop_debounce_frames_ < 1) stop_debounce_frames_ = 1;  // 1 = 不消抖，下个周期即触发

    front_.name = "front";
    rear_.name = "rear";

    // 运动方向镜像：仅用于日志标注（前进/后退/原地旋转/停车/无信号），
    // 融合本身与方向解耦——门控已在检测实例内完成（lidar_name 传空 = 不做门控）
    gate_ = std::make_unique<pothole_detection::MotionGate>(*this, "");

    pub_ = this->create_publisher<EdgeWarning>(edge_topic_, 10);  // 契约 §4.2 默认 QoS
    pub_stop_ = this->create_publisher<StopCommand>(stop_topic_, 10);  // 契约 §4.2 默认 QoS
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
    RCLCPP_INFO(this->get_logger(),
                "急停联动: DANGER → %s (stop_on_danger=%s, debounce=%d帧, 最坏额外延迟≈%.0fms, "
                "danger_dist=%.1fm)",
                stop_topic_.c_str(), stop_on_danger_ ? "true" : "false", stop_debounce_frames_,
                1000.0 * stop_debounce_frames_ / edge_heartbeat_hz_, edge_danger_dist_);
  }

private:
  /// 一路来源（前 / 后）的最新原始告警
  struct Source
  {
    const char *name{""};  // 日志用
    EdgeWarning msg;       // 最近一帧
    rclcpp::Time stamp;    // 到达时刻（节点时钟）
    uint64_t seq{0};       // 收到的新帧序号（消抖按「新帧」计数，见 publishStop）
    bool seen{false};      // 是否收到过
    bool stale_warned{false};
  };

  void onSource(const EdgeWarning::SharedPtr msg, Source &src)
  {
    src.msg = *msg;
    src.stamp = this->now();
    src.seen = true;
    ++src.seq;
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
    publishStop(out, front_wins);  // 契约 §5.2: LEVEL_DANGER 应伴随 stop=true

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "融合: 运动=%s 前[%s %.1fm%s] 后[%s %.1fm%s] → %s %.1fm (%s)，急停=%s",
                         gate_->shortName(),
                         levelName(front.level), front.dist_to_edge_m, front_stale ? "/断流" : "",
                         levelName(rear.level), rear.dist_to_edge_m, rear_stale ? "/断流" : "",
                         levelName(out.level), out.dist_to_edge_m, front_wins ? "取前" : "取后",
                         last_stop_ ? "ON" : "off");
  }

  /// 急停决策与发布 → 契约话题 /perception/stop_command（契约 §3.3/§5.1/§5.2/§7）
  ///
  /// 置位条件：融合等级 == LEVEL_DANGER 且连续 stop_debounce_frames 个「新帧」成立（§3.3 消抖）。
  /// 与 edge_warning 同周期发布、同频（契约 §4.3 ≥5Hz 心跳；风险持续期间持续发 stop=true，
  /// 锁存由系统侧 autodth_cmd_gate 完成）。等级回退则下一周期立即发 stop=false——不消抖、
  /// 立即恢复是硬要求（§3.1/§7.3：操作员复位的前提是 stop 电平已为 false）。
  void publishStop(const EdgeWarning &fused, bool front_wins)
  {
    // 消抖按「新收到的原始告警帧」计数：本节点会复用最近一帧（最长 source_timeout_s），
    // 若按采样周期计数，检测侧的单帧毛刺会被当成 N 个周期，消抖形同虚设。
    // 键 = 侧别 + 该侧帧序号（两侧序号各自独立，用侧别区分避免撞号）。
    const uint64_t frame_key = (front_wins ? front_.seq : rear_.seq) * 2u + (front_wins ? 0u : 1u);
    if (fused.level == EdgeWarning::LEVEL_DANGER)
    {
      if (frame_key != last_danger_key_)
      {
        last_danger_key_ = frame_key;
        danger_frames_ = std::min(danger_frames_ + 1, stop_debounce_frames_);
      }
    }
    else
    {
      danger_frames_ = 0;
      last_danger_key_ = 0;
    }

    const bool stop = stop_on_danger_ && danger_frames_ >= stop_debounce_frames_;

    StopCommand msg;
    msg.header.stamp = this->now();  // 契约 §4.3: 每帧填节点时钟 now()（尊重 use_sim_time）
    msg.header.frame_id = frame_id_;
    msg.stop = stop;
    if (stop)
    {
      msg.reason = StopCommand::REASON_EDGE_DANGER;  // 本节点只产生边坡类急停（§5.1 枚举）
      msg.confidence = confidenceOf(fused.dist_to_edge_m);
      msg.description = std::string(front_wins ? "前方 " : "后方 ") +
                        formatDist(fused.dist_to_edge_m) + " 边坡";  // §5.1 中文简述，进日志
    }
    else
    {
      msg.reason = StopCommand::REASON_NONE;  // §5.1: stop=false 时必须为 REASON_NONE
      msg.confidence = 0.0f;                  // §5.1: stop=false 时必须填 0.0
    }
    pub_stop_->publish(msg);

    if (stop != last_stop_)  // 状态翻转才记日志（本话题 10Hz，避免刷屏）
    {
      if (stop)
      {
        RCLCPP_WARN(this->get_logger(),
                    "急停触发: stop=true reason=EDGE_DANGER (%s, 置信度 %.2f) —— 系统侧锁存，"
                    "需操作员复位",
                    msg.description.c_str(), msg.confidence);
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "急停解除: stop=false (边坡等级回退到 %s)",
                    levelName(fused.level));
      }
      last_stop_ = stop;
    }
  }

  /// 置信度 [0,1]（契约 §5.1）：由距离线性换算——危险阈值处 0.5，越近越高（0m → 1.0）。
  /// DANGER 档位本身已由检测实例按距离门槛给出，故这里不再另设置信度门槛。
  float confidenceOf(float dist) const
  {
    const float ratio = 1.0f - dist / static_cast<float>(edge_danger_dist_);
    return ratio < 0.5f ? 0.5f : (ratio > 1.0f ? 1.0f : ratio);
  }

  static std::string formatDist(float dist)
  {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.1fm", static_cast<double>(dist));
    return std::string(buf);
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
  rclcpp::Publisher<StopCommand>::SharedPtr pub_stop_;  // 契约 §4.1: /perception/stop_command
  rclcpp::Subscription<EdgeWarning>::SharedPtr sub_front_, sub_rear_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<pothole_detection::MotionGate> gate_;  // 仅用于日志标注运动方向

  Source front_, rear_;
  std::string edge_topic_, front_topic_, rear_topic_, frame_id_, stop_topic_;
  double edge_heartbeat_hz_{10.0}, source_timeout_s_{0.5};
  bool stop_on_danger_{true};      // 是否把 DANGER 联动为急停
  double edge_danger_dist_{0.0};   // DANGER 档距离阈值（米），仅用于置信度换算；由参数文件提供
  int stop_debounce_frames_{2};    // 连续 DANGER 原始帧数门槛（1 = 不消抖）
  int danger_frames_{0};           // 当前连续 DANGER 原始帧计数
  uint64_t last_danger_key_{0};    // 已计数的那一帧（侧别+序号，0 = 无）
  bool last_stop_{false};          // 上一周期 stop 电平（仅用于状态翻转日志）
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EdgeWarningFusion>());
  rclcpp::shutdown();
  return 0;
}
