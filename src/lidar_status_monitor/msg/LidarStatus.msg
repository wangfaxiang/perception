# 雷达数据流状态。
# 由 lidar_status_monitor 节点周期发布，用于跟踪雷达是否有数据 / 是否断流。
# （后续 CAN 故障上报节点可订阅本话题，按 state 的沿变化发故障/恢复消息）

std_msgs/Header header      # stamp = 本次采样时刻；frame_id 见节点参数 frame_id

string  lidar_name          # 雷达标识（front / rear）
bool    has_data            # 自节点启动以来是否收到过至少一帧点云

uint8   state               # 当前状态
uint8   STATE_UNKNOWN = 0   # 尚未收到过任何数据（无法判定是否故障）
uint8   STATE_OK      = 1   # 数据流正常
uint8   STATE_TIMEOUT = 2   # 断流：超过 timeout_s 秒未收到数据

float32 last_data_age_s     # 距上次收到数据的秒数；从未收到过数据时为 -1.0
