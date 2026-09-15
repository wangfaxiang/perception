/**
 * @file motion_state.hpp
 * @brief 车辆运动方向判定与「定向检测」门控（AutoDTH 契约 §3.1 / §6.2）
 *
 * 订阅 gate 镜像话题（默认 /control/cmd_gate/cmd_vel，唯一发布者 autodth_cmd_gate，50Hz 恒发），
 * 按契约四态规则判定运动方向，并给出「本雷达当前是否参与检测」：
 *
 *   前进     linear.x ≥ +v_thresh                             → 仅前雷达检测
 *   后退     linear.x ≤ −v_thresh                             → 仅后雷达检测
 *   原地旋转 |linear.x| < v_thresh 且 |angular.z| ≥ w_thresh   → 前后均检测
 *   停车     两轴均低于阈值（含镜像断流 > timeout，或模块单独调试）→ 前后均检测
 *
 * 门控语义：不参与检测的一侧整帧跳过点云处理（省算力），但照常发布安全告警
 * （999.0 / SAFE），使融合侧仍能判断该节点「在线但未检测」；方向切换后下一帧即恢复。
 */

#pragma once

#include <cmath>
#include <cstdio>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace pothole_detection
{

// 车辆运动方向四态（契约 §6.2）
enum class MotionState
{
  kForward,
  kBackward,
  kRotate,
  kStopped,
};

inline const char *toString(MotionState s)
{
  switch (s)
  {
    case MotionState::kForward:  return "前进";
    case MotionState::kBackward: return "后退";
    case MotionState::kRotate:   return "原地旋转";
    default:                     return "停车";
  }
}

// 雷达安装位置（决定定向检测归属）；kBoth = 单雷达调试，不做门控
enum class LidarSide
{
  kFront,
  kRear,
  kBoth,
};

inline LidarSide sideOf(const std::string &lidar_name)
{
  if (lidar_name == "front") return LidarSide::kFront;
  if (lidar_name == "rear") return LidarSide::kRear;
  return LidarSide::kBoth;
}

inline const char *toString(LidarSide s)
{
  switch (s)
  {
    case LidarSide::kFront: return "前雷达";
    case LidarSide::kRear:  return "后雷达";
    default:                return "单雷达";
  }
}

class MotionGate
{
public:
  /// lidar_name: "front" / "rear" / 其他（单雷达调试，不做门控）
  MotionGate(rclcpp::Node &node, const std::string &lidar_name)
  : node_(node), side_(sideOf(lidar_name))
  {
    node_.declare_parameter<std::string>("motion_cmd_topic", "/control/cmd_gate/cmd_vel");
    node_.declare_parameter<double>("motion_v_thresh", 0.05);  // 纵向速度阈值（m/s）
    node_.declare_parameter<double>("motion_w_thresh", 0.05);  // 角速度阈值（rad/s）
    node_.declare_parameter<double>("motion_timeout_s", 0.5);  // 镜像断流超时（s）→ 按停车

    node_.get_parameter("motion_cmd_topic", cmd_topic_);
    node_.get_parameter("motion_v_thresh", v_thresh_);
    node_.get_parameter("motion_w_thresh", w_thresh_);
    node_.get_parameter("motion_timeout_s", timeout_s_);

    sub_ = node_.create_subscription<geometry_msgs::msg::Twist>(
        cmd_topic_, rclcpp::QoS(10),  // 契约 §6.2 默认 QoS；只订阅，严禁发布 cmd_vel
        [this](geometry_msgs::msg::Twist::SharedPtr msg) {
          if (!seen_)
            RCLCPP_INFO(node_.get_logger(), "收到 gate 镜像 %s，运动方向门控生效",
                        cmd_topic_.c_str());
          v_ = msg->linear.x;
          w_ = msg->angular.z;
          stamp_ = node_.now();
          seen_ = true;
        });
  }

  /// 镜像是否新鲜（收到过且未超时）；不新鲜时按停车处理（契约 §6.2）
  bool mirrorFresh() const
  {
    return seen_ && (node_.now() - stamp_).seconds() <= timeout_s_;
  }

  /// 四态判定（未收到 / 断流超时 → 停车，契约 §6.2）
  MotionState state() const
  {
    if (!mirrorFresh()) return MotionState::kStopped;

    // 弧线转弯（v≠0 ∧ ω≠0）时车身仍沿 v 平移，线性主导，按 v 判
    if (v_ >= v_thresh_) return MotionState::kForward;
    if (v_ <= -v_thresh_) return MotionState::kBackward;
    if (std::fabs(w_) >= w_thresh_) return MotionState::kRotate;
    return MotionState::kStopped;
  }

  /// 本雷达在当前运动方向下是否参与检测（停车 / 原地旋转 → 前后均检测）
  bool active() const
  {
    switch (state())
    {
      case MotionState::kForward:  return side_ != LidarSide::kRear;
      case MotionState::kBackward: return side_ != LidarSide::kFront;
      default:                     return true;
    }
  }

  LidarSide side() const { return side_; }

  const std::string &cmdTopic() const { return cmd_topic_; }

  /// 调试用：一行状态标签（前进 / 后退 / 原地旋转 / 停车 / 无信号）
  /// 「无信号」= 未收到镜像或断流超时（此时行为上按停车处理：前后均检测）
  const char *shortName() const
  {
    return mirrorFresh() ? toString(state()) : "无信号";
  }

  /// 镜像链路状态：区分「从未收到」（单独调试/未接系统侧）与「收到过但断流」
  const char *mirrorName() const
  {
    if (!seen_) return "未收到（按停车处理，双向检测）";
    if (!mirrorFresh()) return "断流（按停车处理，双向检测）";
    return "正常";
  }

  /// 调试用：一行文字描述门控配置与当前状态（供启动日志 / 问题定位）
  std::string describe() const
  {
    char buf[224];
    std::snprintf(buf, sizeof(buf),
                  "%s, cmd=%s, v_thresh=%.3fm/s, w_thresh=%.3frad/s, timeout=%.2fs, 状态=%s, 镜像=%s",
                  toString(side_), cmd_topic_.c_str(), v_thresh_, w_thresh_, timeout_s_,
                  shortName(), mirrorName());
    return buf;
  }

private:
  rclcpp::Node &node_;
  LidarSide side_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;

  std::string cmd_topic_;
  double v_thresh_{0.05}, w_thresh_{0.05}, timeout_s_{0.5};
  double v_{0.0}, w_{0.0};  // 最近一帧镜像指令
  rclcpp::Time stamp_;      // 最近一帧镜像到达时刻（节点时钟）
  bool seen_{false};
};

}  // namespace pothole_detection
