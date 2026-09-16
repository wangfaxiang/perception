/**
 * @file warning_fusion_node.cpp
 * @brief 感知侧共用「报警融合节点」——契约三路话题的唯一发布者
 *        （AutoDTH 契约 §4 发布接口 / §5 字段语义 / §7 急停行为）
 *
 * 发布（契约 §4.1 冻结话题名与类型，默认 QoS KEEP_LAST(10)，≥5Hz 心跳，建议 10Hz）:
 *   /perception/edge_warning      dth_messages/msg/EdgeWarning     边坡预警（Web 面板显示/留档）
 *   /perception/obstacle_warning  dth_messages/msg/ObstacleWarning 障碍物预警（Web 面板显示/留档）
 *   /perception/stop_command      dth_messages/msg/StopCommand     急停指令（系统侧锁存）
 *
 * 订阅（两类检测节点各自发布的「单雷达原始告警」，均挂节点私有名空间，契约 §9）:
 *   边坡: /pothole_detection_{front,rear}/edge_warning_raw      EdgeWarning
 *   障碍: /cluster_{front,rear}/obstacle_warning_raw            ObstacleWarning
 *
 * 为什么两类检测共用本节点:
 *   ① 契约话题只能有一个发布者，三路输出必须出自同一处，才能保证「边坡 DANGER」与
 *      「障碍物 PERSON/VEHICLE/OTHER」在同一个周期内合成一条不矛盾的 stop 决策；
 *   ② 定向检测门控（perception_common::MotionGate）在两个检测节点内各自完成——前进只测前、
 *      后退只测后、停车/原地旋转双向均测——被门控的一侧原始告警恒为安全默认值，
 *      因此本节点只做「取哪一路」的聚合，不再需要方向判断（运动方向仅用于日志标注）。
 *
 * 融合规则:
 *   边坡（§5.2）: 取等级更高的一路，同级取 dist 更近者；warning = (level ≠ LEVEL_SAFE)。
 *   障碍（§3.1/§5.3）: 类别优先（人员 > 车辆 > 其他），同级取 dist_m 最近者；
 *                      无人威胁时输出安全默认值（detected=false / type=0 / dist=999.0）。
 *   急停（§3.3/§5.1）: 两类候选各自独立消抖后合并，reason 取数值最小者
 *                      （= 优先级最高：人员 1 > 车辆 2 > 其他 3 > 边坡 4），
 *                      confidence/description 与胜出候选同源。
 *
 * 安全默认值与心跳（§4.3）: 启动即进入安全态（stop=false / warning=false / detected=false /
 * 距离 999.0 / 置信度 0.0），周期发布；某路原始告警断流超时按安全默认值参与融合，
 * 全部断流即输出安全默认值；风险消失后立即恢复 stop=false（§7.3 操作员复位的前置条件）。
 */

#include <rclcpp/rclcpp.hpp>
#include <dth_messages/msg/edge_warning.hpp>
#include <dth_messages/msg/obstacle_warning.hpp>
#include <dth_messages/msg/stop_command.hpp>

#include <perception_common/motion_state.hpp>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>

class WarningFusion : public rclcpp::Node
{
public:
  using EdgeWarning = dth_messages::msg::EdgeWarning;
  using ObstacleWarning = dth_messages::msg::ObstacleWarning;
  using StopCommand = dth_messages::msg::StopCommand;

  WarningFusion() : Node("warning_fusion")
  {
    // ---- 契约话题（§4.1，冻结项：不得增删改名）----
    this->declare_parameter<std::string>("edge_warning_topic", "/perception/edge_warning");
    this->declare_parameter<std::string>("obstacle_warning_topic", "/perception/obstacle_warning");
    this->declare_parameter<std::string>("stop_command_topic", "/perception/stop_command");

    // ---- 两类检测实例的原始告警来源（检测节点私有名空间）----
    this->declare_parameter<std::string>("front_edge_topic",
                                         "/pothole_detection_front/edge_warning_raw");
    this->declare_parameter<std::string>("rear_edge_topic",
                                         "/pothole_detection_rear/edge_warning_raw");
    this->declare_parameter<std::string>("front_obstacle_topic",
                                         "/cluster_front/obstacle_warning_raw");
    this->declare_parameter<std::string>("rear_obstacle_topic",
                                         "/cluster_rear/obstacle_warning_raw");

    // 无目标帧（边坡为标量；无障碍物时坐标为 0）使用的 frame_id
    this->declare_parameter<std::string>("frame_id", "base_link");

    this->declare_parameter("heartbeat_hz", 10.0);      // 契约 §4.3 必须 ≥5Hz，建议 10Hz
    this->declare_parameter("source_timeout_s", 0.5);   // 单路原始告警断流超时（秒）

    // ---- 急停联动（§3.3/§5.1/§7）----
    this->declare_parameter("stop_on_danger", true);          // false = 只告警不联动急停
    this->declare_parameter("stop_debounce_frames", 2);       // 连续 N 帧原始告警才置 stop
    // 急停阈值（唯一真源 = config/params.yaml 的 /** 共享段，与检测实例同源）:
    this->declare_parameter("edge_danger_dist", rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("obstacle_danger_dist", rclcpp::ParameterType::PARAMETER_DOUBLE);

    this->get_parameter("edge_warning_topic", edge_topic_);
    this->get_parameter("obstacle_warning_topic", obstacle_topic_);
    this->get_parameter("stop_command_topic", stop_topic_);
    this->get_parameter("front_edge_topic", front_edge_topic_);
    this->get_parameter("rear_edge_topic", rear_edge_topic_);
    this->get_parameter("front_obstacle_topic", front_obstacle_topic_);
    this->get_parameter("rear_obstacle_topic", rear_obstacle_topic_);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("heartbeat_hz", heartbeat_hz_);
    this->get_parameter("source_timeout_s", source_timeout_s_);
    this->get_parameter("stop_on_danger", stop_on_danger_);
    this->get_parameter("stop_debounce_frames", stop_debounce_frames_);
    this->get_parameter("edge_danger_dist", edge_danger_dist_);
    this->get_parameter("obstacle_danger_dist", obstacle_danger_dist_);

    if (!(heartbeat_hz_ > 0.0)) heartbeat_hz_ = 10.0;
    if (stop_debounce_frames_ < 1) stop_debounce_frames_ = 1;  // 1 = 不消抖，收到即触发
    if (!(edge_danger_dist_ > 0.0) || !(obstacle_danger_dist_ > 0.0))
    {
      RCLCPP_FATAL(this->get_logger(),
                   "急停阈值非法（edge_danger_dist=%.1f, obstacle_danger_dist=%.1f）："
                   "须由 config/params.yaml 的 /** 共享段提供且 > 0",
                   edge_danger_dist_, obstacle_danger_dist_);
      throw std::runtime_error("invalid danger thresholds");
    }

    front_edge_.name = "前";
    rear_edge_.name = "后";
    front_obstacle_.name = "前";
    rear_obstacle_.name = "后";

    // 运动方向镜像：仅用于日志标注（前进/后退/原地旋转/停车/无信号）。
    // 门控已在检测实例内完成（lidar_name 传空 = 本节点不做门控）
    gate_ = std::make_unique<perception_common::MotionGate>(*this, "");

    pub_edge_ = this->create_publisher<EdgeWarning>(edge_topic_, 10);            // §4.2 默认 QoS
    pub_obstacle_ = this->create_publisher<ObstacleWarning>(obstacle_topic_, 10);
    pub_stop_ = this->create_publisher<StopCommand>(stop_topic_, 10);

    sub_front_edge_ = this->create_subscription<EdgeWarning>(
        front_edge_topic_, rclcpp::QoS(10),
        [this](EdgeWarning::SharedPtr msg) { onSource(msg, front_edge_); });
    sub_rear_edge_ = this->create_subscription<EdgeWarning>(
        rear_edge_topic_, rclcpp::QoS(10),
        [this](EdgeWarning::SharedPtr msg) { onSource(msg, rear_edge_); });
    sub_front_obstacle_ = this->create_subscription<ObstacleWarning>(
        front_obstacle_topic_, rclcpp::QoS(10),
        [this](ObstacleWarning::SharedPtr msg) { onSource(msg, front_obstacle_); });
    sub_rear_obstacle_ = this->create_subscription<ObstacleWarning>(
        rear_obstacle_topic_, rclcpp::QoS(10),
        [this](ObstacleWarning::SharedPtr msg) { onSource(msg, rear_obstacle_); });

    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / heartbeat_hz_),
        std::bind(&WarningFusion::publishCycle, this));

    RCLCPP_INFO(this->get_logger(),
                "告警融合启动: 边坡[%s + %s] 障碍[%s + %s] → %s / %s / %s，%.1fHz，"
                "source_timeout=%.2fs，motion_cmd=%s",
                front_edge_topic_.c_str(), rear_edge_topic_.c_str(),
                front_obstacle_topic_.c_str(), rear_obstacle_topic_.c_str(),
                edge_topic_.c_str(), obstacle_topic_.c_str(), stop_topic_.c_str(),
                heartbeat_hz_, source_timeout_s_, gate_->cmdTopic().c_str());
    RCLCPP_INFO(this->get_logger(),
                "急停联动: 边坡 DANGER(>%.1fm 内) / 障碍物 ≤%.1fm → %s "
                "(stop_on_danger=%s, debounce=%d帧, 最坏额外延迟≈%.0fms)",
                edge_danger_dist_, obstacle_danger_dist_, stop_topic_.c_str(),
                stop_on_danger_ ? "true" : "false", stop_debounce_frames_,
                1000.0 * stop_debounce_frames_ / heartbeat_hz_);
  }

private:
  /// 一路来源（前 / 后）的最新原始告警
  template <typename T>
  struct Source
  {
    const char *name{""};  // 日志用
    T msg;                 // 最近一帧
    rclcpp::Time stamp;    // 到达时刻（节点时钟）
    uint64_t seq{0};       // 收到的新帧序号（消抖按「新帧」计数，见 tick()）
    bool seen{false};      // 是否收到过
    bool stale_warned{false};
  };

  /// 单向消抖计数器：按「新帧」计数（本节点会复用最近一帧最长 source_timeout_s，
  /// 若按采样周期计数，检测侧的单帧毛刺会被当成 N 个周期，消抖形同虚设）
  struct Debounce
  {
    int frames{0};
    uint64_t last_key{0};  // 已计数的那一帧（侧别 + 帧序号，0 = 无）
  };

  template <typename T>
  void onSource(const std::shared_ptr<T> &msg, Source<T> &src)
  {
    src.msg = *msg;
    src.stamp = this->now();
    src.seen = true;
    ++src.seq;
  }

  /// 原始告警可用性：断流/未收到 → 安全默认值（断流与恢复各只提示一次）
  EdgeWarning usableEdge(Source<EdgeWarning> &src, bool &stale)
  {
    stale = isStale(src);
    if (!stale) return src.msg;

    EdgeWarning safe;
    safe.warning = false;
    safe.dist_to_edge_m = 999.0f;  // 无数据哨兵值（契约 §9）
    safe.level = EdgeWarning::LEVEL_SAFE;
    return safe;
  }

  ObstacleWarning usableObstacle(Source<ObstacleWarning> &src, bool &stale)
  {
    stale = isStale(src);
    if (!stale) return src.msg;
    return safeObstacle();
  }

  template <typename T>
  bool isStale(Source<T> &src)
  {
    const bool stale = !src.seen || (this->now() - src.stamp).seconds() > source_timeout_s_;
    if (stale && !src.stale_warned)
    {
      RCLCPP_WARN(this->get_logger(), "原始告警 %s 断流超时(>%.2fs)，按安全默认值参与融合",
                  src.name, source_timeout_s_);
      src.stale_warned = true;
    }
    else if (!stale && src.stale_warned)
    {
      RCLCPP_INFO(this->get_logger(), "原始告警 %s 恢复", src.name);
      src.stale_warned = false;
    }
    return stale;
  }

  /// 无障碍物时的安全默认值（契约 §4.3/§5.3：type=0、坐标 0.0、距离哨兵 999.0）
  static ObstacleWarning safeObstacle()
  {
    ObstacleWarning msg;
    msg.detected = false;
    msg.obstacle_type = 0;
    msg.obstacle_x = 0.0f;
    msg.obstacle_y = 0.0f;
    msg.dist_m = 999.0f;
    return msg;
  }

  /// 障碍物两路取优：类别优先（人员 > 车辆 > 其他），同级取 dist_m 更近者（契约 §3.1/§5.3）
  static bool betterObstacle(const ObstacleWarning &a, const ObstacleWarning &b)
  {
    const bool a_threat = isThreat(a);
    const bool b_threat = isThreat(b);
    if (a_threat != b_threat) return a_threat;
    if (!a_threat) return true;  // 两侧均无威胁，输出等价
    if (a.obstacle_type != b.obstacle_type) return a.obstacle_type < b.obstacle_type;
    return a.dist_m <= b.dist_m;
  }

  /// 合法威胁帧：detected=true 时必须带 type ∈ {1,2,3}（契约 §5.3 禁止 true+0 组合）
  static bool isThreat(const ObstacleWarning &msg)
  {
    return msg.detected && msg.obstacle_type >= ObstacleWarning::TYPE_PERSON &&
           msg.obstacle_type <= ObstacleWarning::TYPE_OTHER;
  }

  void tick(Debounce &db, bool active, uint64_t key)
  {
    if (!active)
    {
      db.frames = 0;
      db.last_key = 0;
      return;
    }
    if (key != db.last_key)
    {
      db.last_key = key;
      db.frames = std::min(db.frames + 1, stop_debounce_frames_);
    }
  }

  /// 周期融合发布（同时充当契约 §4.3 的心跳）
  void publishCycle()
  {
    const rclcpp::Time now = this->now();

    // ---------- ① 边坡：取等级更高的一路，同级取距离更近者 ----------
    bool front_edge_stale = false, rear_edge_stale = false;
    const EdgeWarning front_edge = usableEdge(front_edge_, front_edge_stale);
    const EdgeWarning rear_edge = usableEdge(rear_edge_, rear_edge_stale);
    const bool front_edge_wins =
        front_edge.level > rear_edge.level ||
        (front_edge.level == rear_edge.level &&
         front_edge.dist_to_edge_m <= rear_edge.dist_to_edge_m);

    EdgeWarning edge_out = front_edge_wins ? front_edge : rear_edge;
    edge_out.header.stamp = now;  // 契约 §4.3: 每帧填节点时钟 now()（尊重 use_sim_time）
    edge_out.header.frame_id = frame_id_;
    edge_out.warning = (edge_out.level != EdgeWarning::LEVEL_SAFE);  // §5.2 固定映射
    pub_edge_->publish(edge_out);

    // ---------- ② 障碍物：类别优先，同级取最近 ----------
    bool front_obstacle_stale = false, rear_obstacle_stale = false;
    const ObstacleWarning front_obstacle = usableObstacle(front_obstacle_, front_obstacle_stale);
    const ObstacleWarning rear_obstacle = usableObstacle(rear_obstacle_, rear_obstacle_stale);
    const bool front_obstacle_wins = betterObstacle(front_obstacle, rear_obstacle);

    ObstacleWarning obstacle_out = front_obstacle_wins ? front_obstacle : rear_obstacle;
    const bool obstacle_threat = isThreat(obstacle_out);
    if (!obstacle_threat)
    {
      obstacle_out = safeObstacle();  // 契约 §4.3: 无威胁 → 安全默认值
      obstacle_out.header.frame_id = frame_id_;
    }
    // 有威胁时保留胜出侧的 frame_id：位置即该雷达坐标系下的坐标（检测实例按雷达系输出），
    // 面板/留档如需车体系坐标可自行按 frame_id 做 TF 变换。
    obstacle_out.header.stamp = now;
    pub_obstacle_->publish(obstacle_out);

    // ---------- ③ 急停：两类候选各自消抖后合并 ----------
    const bool edge_danger = (edge_out.level == EdgeWarning::LEVEL_DANGER);
    const uint64_t edge_key =
        (front_edge_wins ? front_edge_.seq : rear_edge_.seq) * 2u + (front_edge_wins ? 0u : 1u);
    tick(debounce_edge_, edge_danger, edge_key);

    const bool obstacle_danger =
        obstacle_threat && obstacle_out.dist_m <= static_cast<float>(obstacle_danger_dist_);
    const uint64_t obstacle_key = (front_obstacle_wins ? front_obstacle_.seq
                                                       : rear_obstacle_.seq) *
                                      2u +
                                  (front_obstacle_wins ? 0u : 1u);
    tick(debounce_obstacle_, obstacle_danger, obstacle_key);

    const bool edge_ready = debounce_edge_.frames >= stop_debounce_frames_;
    const bool obstacle_ready = debounce_obstacle_.frames >= stop_debounce_frames_;

    // reason 取数值最小者 = 优先级最高（人员 1 > 车辆 2 > 其他 3 > 边坡 4）：
    // 契约未规定两类同时成立时的取舍，此处按 §5.1 枚举数值顺序（与 §3.1 类别优先一致）取最严者。
    uint8_t reason = StopCommand::REASON_NONE;
    if (edge_ready) reason = StopCommand::REASON_EDGE_DANGER;
    if (obstacle_ready && (reason == StopCommand::REASON_NONE || obstacle_out.obstacle_type < reason))
      reason = obstacle_out.obstacle_type;

    StopCommand stop_msg;
    stop_msg.header.stamp = now;  // 契约 §4.3: 每帧填节点时钟 now()
    stop_msg.header.frame_id = frame_id_;
    const bool stop = stop_on_danger_ && (reason != StopCommand::REASON_NONE);
    if (stop)
    {
      stop_msg.stop = true;
      stop_msg.reason = reason;
      if (reason == StopCommand::REASON_EDGE_DANGER)
      {
        stop_msg.confidence = confidenceOf(edge_out.dist_to_edge_m, edge_danger_dist_);
        stop_msg.description = std::string(front_edge_wins ? "前方 " : "后方 ") +
                               formatDist(edge_out.dist_to_edge_m) + " 边坡";
      }
      else
      {
        stop_msg.confidence = confidenceOf(obstacle_out.dist_m, obstacle_danger_dist_);
        stop_msg.description = obstacle_out.description;  // 检测实例已给「方位+距离+类别」
      }
    }
    else
    {
      stop_msg.stop = false;
      stop_msg.reason = StopCommand::REASON_NONE;  // §5.1: stop=false 时必须为 REASON_NONE
      stop_msg.confidence = 0.0f;                  // §5.1: stop=false 时必须填 0.0
    }
    pub_stop_->publish(stop_msg);

    if (stop != last_stop_)
    {
      if (stop)
      {
        RCLCPP_WARN(this->get_logger(),
                    "急停触发: stop=true reason=%s (%s, 置信度 %.2f) —— 系统侧锁存，需操作员复位",
                    reasonName(reason), stop_msg.description.c_str(), stop_msg.confidence);
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "急停解除: stop=false (边坡 %s / 障碍物 %s)",
                    levelName(edge_out.level), threatName(obstacle_out));
      }
      last_stop_ = stop;
    }

    char buf[320];
    std::snprintf(buf, sizeof(buf),
                  "融合: 运动=%s 边坡[前 %s %.1fm%s 后 %s %.1fm%s → %s %.1fm 取%s] "
                  "障碍[前 %s %.1fm%s 后 %s %.1fm%s → %s %.1fm 取%s] 急停=%s(%s)",
                  gate_->shortName(),
                  levelName(front_edge.level), front_edge.dist_to_edge_m,
                  front_edge_stale ? "/断流" : "",
                  levelName(rear_edge.level), rear_edge.dist_to_edge_m,
                  rear_edge_stale ? "/断流" : "",
                  levelName(edge_out.level), edge_out.dist_to_edge_m,
                  front_edge_wins ? "前" : "后",
                  threatName(front_obstacle), front_obstacle.dist_m,
                  front_obstacle_stale ? "/断流" : "",
                  threatName(rear_obstacle), rear_obstacle.dist_m,
                  rear_obstacle_stale ? "/断流" : "",
                  threatName(obstacle_out), obstacle_out.dist_m,
                  front_obstacle_wins ? "前" : "后",
                  last_stop_ ? "ON" : "off", reasonName(reason));
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "%s", buf);
  }

  /// 置信度 [0,1]（契约 §5.1）：由距离线性换算——危险阈值处 0.5，越近越高（0m → 1.0）。
  /// 档位本身已由检测实例按各自阈值给出，故这里不再另设置信度门槛。
  static float confidenceOf(float dist, double danger_dist)
  {
    const float ratio = 1.0f - dist / static_cast<float>(danger_dist);
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

  /// 障碍物一路的日志名（无威胁 / 类别 + 距离）
  static const char *typeName(uint8_t type)
  {
    switch (type)
    {
      case ObstacleWarning::TYPE_PERSON:  return "人员";
      case ObstacleWarning::TYPE_VEHICLE: return "车辆";
      case ObstacleWarning::TYPE_OTHER:   return "障碍物";
      default:                            return "无";
    }
  }

  static const char *threatName(const ObstacleWarning &msg)
  {
    return isThreat(msg) ? typeName(msg.obstacle_type) : "无";
  }

  static const char *reasonName(uint8_t reason)
  {
    switch (reason)
    {
      case StopCommand::REASON_OBSTACLE_PERSON:  return "人员";
      case StopCommand::REASON_OBSTACLE_VEHICLE: return "车辆";
      case StopCommand::REASON_OBSTACLE_OTHER:   return "其他障碍物";
      case StopCommand::REASON_EDGE_DANGER:      return "边坡 DANGER";
      default:                                   return "无";
    }
  }

  rclcpp::Publisher<EdgeWarning>::SharedPtr pub_edge_;
  rclcpp::Publisher<ObstacleWarning>::SharedPtr pub_obstacle_;
  rclcpp::Publisher<StopCommand>::SharedPtr pub_stop_;
  rclcpp::Subscription<EdgeWarning>::SharedPtr sub_front_edge_, sub_rear_edge_;
  rclcpp::Subscription<ObstacleWarning>::SharedPtr sub_front_obstacle_, sub_rear_obstacle_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<perception_common::MotionGate> gate_;  // 仅用于日志标注运动方向

  Source<EdgeWarning> front_edge_, rear_edge_;
  Source<ObstacleWarning> front_obstacle_, rear_obstacle_;
  Debounce debounce_edge_, debounce_obstacle_;

  std::string edge_topic_, obstacle_topic_, stop_topic_;
  std::string front_edge_topic_, rear_edge_topic_;
  std::string front_obstacle_topic_, rear_obstacle_topic_;
  std::string frame_id_;
  double heartbeat_hz_{10.0}, source_timeout_s_{0.5};
  bool stop_on_danger_{true};
  int stop_debounce_frames_{2};
  double edge_danger_dist_{0.0};      // 边坡 DANGER 阈值（米，仅供参考 confidence 换算）
  double obstacle_danger_dist_{0.0};  // 障碍物急停阈值（米）
  bool last_stop_{false};             // 上一周期 stop 电平（仅用于状态翻转日志）
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WarningFusion>());
  rclcpp::shutdown();
  return 0;
}
