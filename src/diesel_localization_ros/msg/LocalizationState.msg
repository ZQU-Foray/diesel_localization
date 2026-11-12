# L3 定位健康契约：下游只依赖本消息判断位姿是否可用。
# 状态与 confidence 必须同时发布，禁止“位姿照发、失效不说”。

# 状态机取值（与 diesel::localization::HealthState 一一对应）
uint8 STATE_INITIALIZING=0
uint8 STATE_OK=1
uint8 STATE_DEGRADED=2
uint8 STATE_LOST=3

# 位姿来源（与 diesel::localization::PoseSource 对应）
uint8 SOURCE_UNKNOWN=0
uint8 SOURCE_SENTRY_LIDAR=1
uint8 SOURCE_EXTERNAL_ODOM=2

std_msgs/Header header

# map 坐标系下的机器人位姿（base_footprint）。
geometry_msgs/PoseWithCovariance pose

# 位姿可用性：只有本字段为 true 时才允许消费 pose（导航/决策的唯一放行条件）。
# INITIALIZING：还没完成热启动，pose 是未校正的里程计值；
# LOST：map->odom 停更且 TF 停发，pose 只是最后一次合成值。
# 两种情况下 pose 仅供诊断，禁止用于闭环控制（审计 B2：静默外推）。
bool pose_valid

# 0..1；LOST 时必须为 0。
float64 confidence

# 距最近一次有效校正的时间（秒）。
float64 age_sec

uint8 state
uint8 source

# 人类可读的原因，便于现场诊断与 rosbag 复盘。
string reason
