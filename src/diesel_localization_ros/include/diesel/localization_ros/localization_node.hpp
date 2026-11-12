// Copyright 2026 ZQU-Foray
//
// 定位主节点：地图配准、健康状态、TF map->odom、动态障碍与手动重定位。
// 不直接访问任何硬件；输入是标准 ROS 话题（decision.md D7）。

#ifndef DIESEL_LOCALIZATION_ROS_LOCALIZATION_NODE_HPP_
#define DIESEL_LOCALIZATION_ROS_LOCALIZATION_NODE_HPP_

#include "diesel/localization/health.hpp"
#include "diesel/localization/pose.hpp"
#include "diesel/localization/pose_ekf.hpp"
#include "diesel/localization_map/aligner.hpp"
#include "diesel/localization_map/aligner_factory.hpp"
#include "diesel/localization_map/observability.hpp"
#include "diesel/localization_map/prior_map.hpp"
#include "diesel/localization_map/residual_clusterer.hpp"
#include "diesel_localization_ros/msg/dynamic_obstacle_array.hpp"
#include "diesel_localization_ros/msg/localization_state.hpp"
#include "diesel_localization_ros/msg/map_meta.hpp"
#include "diesel_localization_ros/srv/relocalize.hpp"

#include <Eigen/Geometry>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

namespace diesel::localization_ros {

class LocalizationNode : public rclcpp::Node {
  public:
    /// `aligner` 非空时替代工厂构造的后端（decision.md D21 的可替换点）。该参数只服务于
    /// 离线回放与节点级测试：生产入口 `localization_node_main.cpp` 不传它，走编译期开关。
    explicit LocalizationNode(const rclcpp::NodeOptions& options,
                              std::unique_ptr<diesel::localization_map::Aligner> aligner = nullptr);

    /// 运行期计数快照（decision.md D29）。只服务于测试与现场排障，不参与任何控制逻辑。
    /// 读的时候持锁，所以拿到的是同一时刻的一致快照。
    struct Diagnostics {
        /// 里程计回调进入次数（含随后被校验丢弃的帧）。
        std::uint64_t odom_received{0};
        /// 完成状态更新的里程计帧数；与上一项之差即被丢弃的帧数。
        std::uint64_t odom_processed{0};
        /// 处理里程计帧时 `now - header.stamp` 的峰值（秒）。两者都是节点时钟，可直接相减。
        double max_odom_lag_sec{0.0};
        /// 实际发生的配准次数，无论后端是真配准还是注入的桩。
        std::uint64_t align_runs{0};
        /// 单次配准耗时峰值（毫秒，steady_clock）。
        double max_align_ms{0.0};
        /// 成功广播 `map -> odom` 的次数。
        std::uint64_t tf_published{0};
        /// 先验地图里被跳过的 ascii 行数（字段数不足或 x/y/z 非有限）。非 0 时装载期已告警，
        /// 这里再暴露一个可断言的出口：审计 L18 要证明「丢点」不再只是日志里的一句话。
        std::size_t skipped_pcd_lines{0};
    };
    Diagnostics diagnostics();

  private:
    void on_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);
    void on_odometry(const nav_msgs::msg::Odometry::ConstSharedPtr odometry);
    void on_external_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose);
    void on_state_timer();
    void on_tf_timer();
    void on_align_timer();
    void on_obstacle_timer();
    void
    on_relocalize(const std::shared_ptr<diesel_localization_ros::srv::Relocalize::Request> request,
                  std::shared_ptr<diesel_localization_ros::srv::Relocalize::Response> response);

    void load_map();
    void publish_state();
    void publish_tf();

    // 参数
    std::string map_manifest_;
    std::string map_frame_{"map"};
    std::string odom_frame_{"odom"};
    std::string base_frame_{"base_footprint"};
    double voxel_leaf_m_{0.1};
    double map_voxel_leaf_m_{0.25};
    double residual_threshold_m_{0.3};
    double cluster_tolerance_m_{0.3};
    bool verify_map_hashes_{true};
    std::string pose_source_param_{"auto"};
    /// 视场裁切（decision.md D23）。默认 360x180 = 不裁切，行为与改造前一致。
    double fov_horizontal_deg_{360.0};
    double fov_vertical_deg_{180.0};
    double fov_center_yaw_deg_{0.0};
    /// 归一化特征值阈值，兼作可观测性总开关：<= 0 表示不启用（默认）。
    double observability_ratio_threshold_{0.0};
    /// 配准对应距离上限，用于构建目标侧近邻索引与可观测性分析。
    double max_correspondence_distance_m_{0.5};

    /// 分段耗时累加器（decision.md D25）。profile_enabled 为 false 时只多一次判空。
    struct ProfileTotals {
        double to_pcl_ms{0.0};
        double voxel_ms{0.0};
        double align_ms{0.0};
        /// 点数漏斗（审计 L7 / decision.md D58）：原始 → 视场裁切后 → 体素后 → 送进配准的
        /// source，以及 target（先验图体素后的点数）。耗时不能解释「配准无有效对应」，
        /// 只有点数能：仿真实测 60000 点经 30° 裁切与 0.10 m 体素后只剩 64~70 点。
        double raw_points{0.0};
        double crop_points{0.0};
        double voxel_points{0.0};
        double align_source{0.0};
        double align_target{0.0};
        std::size_t samples{0};
        std::int64_t window_start_ns{0};
    };
    bool profile_enabled_{false};
    ProfileTotals profile_;

    void maybe_flush_profile(std::int64_t now_ns);
    /// 把「地图不可用」写成一条显式事实：记下原因并把健康状态压进 `LOST`。
    /// 只被 `load_map()` 的失败分支调用（decision.md D36）。
    void mark_map_unavailable(const std::string& reason);
    /// 查 `base_frame_ → child_frame` 的外参（decision.md D37）。点云与里程计共用这一套：
    /// 拿不到就返回 false，调用方负责丢帧——绝不返回单位阵把「不知道」装成「一致」。
    /// @param time 查询时刻；传 `rclcpp::Time(0)` 表示「取最新可用」。
    /// @param timeout 等待上限：点云用 100 ms 容忍 TF 晚到；里程计 200 Hz，用 0（不阻塞）。
    bool lookup_base_from(const std::string& child_frame, const rclcpp::Time& time,
                          std::chrono::milliseconds timeout, Eigen::Isometry3d* base_from_child);
    /// 雷达位姿是否还新鲜（`lidar_timeout_sec` 内收到过点云）。调用方须持有 `mutex_`。
    bool lidar_available_locked(std::int64_t now_ns) const;
    /// 外部位姿是否还新鲜（`odom_timeout_sec` 内收到过一帧）。调用方须持有 `mutex_`。
    bool external_available_locked(std::int64_t now_ns) const;

    rclcpp::Publisher<diesel_localization_ros::msg::LocalizationState>::SharedPtr state_pub_;
    rclcpp::Publisher<diesel_localization_ros::msg::MapMeta>::SharedPtr map_meta_pub_;
    rclcpp::Publisher<diesel_localization_ros::msg::DynamicObstacleArray>::SharedPtr obstacle_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        external_pose_sub_;
    rclcpp::Service<diesel_localization_ros::srv::Relocalize>::SharedPtr relocalize_srv_;

    /// 回调组按「谁阻塞谁」划分（decision.md D30）：配准与聚类的耗时可到百毫秒级，
    /// 它们必须和里程计、发布这类毫秒级、对时序敏感的回调分在不同组里，
    /// 否则 executor 再多线程也只能排队跑，里程计会被配准饿死。
    /// 组间并行只解决「互不阻塞」，共享状态仍由 `mutex_` 唯一仲裁。
    rclcpp::CallbackGroup::SharedPtr odom_group_;
    rclcpp::CallbackGroup::SharedPtr cloud_group_;
    rclcpp::CallbackGroup::SharedPtr align_group_;
    rclcpp::CallbackGroup::SharedPtr obstacle_group_;
    rclcpp::CallbackGroup::SharedPtr publish_group_;
    rclcpp::CallbackGroup::SharedPtr service_group_;

    rclcpp::TimerBase::SharedPtr state_timer_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
    rclcpp::TimerBase::SharedPtr align_timer_;
    rclcpp::TimerBase::SharedPtr obstacle_timer_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    diesel::localization_map::PriorMap map_;
    bool has_map_{false};
    /// 地图不可用的原因（空 = 地图正常）。非空时健康状态被锁在 `LOST`，
    /// `map → odom` 一帧都不发：没有地图就没有配准，任何位姿都是无源的（decision.md D36）。
    std::string map_error_;
    /// 最近一次 TF 查询失败的原因（空 = 最近一次查询成功）。
    /// `on_cloud()`/`on_odometry()` 拿不到外参就丢帧，于是一个样本都进不了健康状态机；
    /// 没有这条记录，`publish_state()` 就看不出「TF 缺失导致配准从未跑起来」，
    /// 状态会永久冻在 `INITIALIZING`（审计 L3）。查询发生在不持 `mutex_` 的回调里，故单独配锁。
    std::string tf_error_;
    std::mutex tf_error_mutex_;
    /// 目标侧近邻索引与法向缓存，随地图生命周期只建一次（decision.md D25）。
    /// 两者都持有 `map_.cloud` 的引用，必须在 `map_` 之后声明、在 relocad 时整体重建。
    std::shared_ptr<diesel::localization_map::PointCloudIndex> map_index_;
    std::shared_ptr<diesel::localization_map::NormalCache> map_normals_;
    /// 配准后端由工厂按编译期开关构造（decision.md D21），节点不关心具体实现。
    std::unique_ptr<diesel::localization_map::Aligner> aligner_;
    diesel::localization_map::ResidualClusterer clusterer_;
    diesel::localization::HealthMonitor health_;
    /// 融合层滤波（decision.md D24）。默认关闭，关闭时位姿更新与改造前完全一致。
    diesel::localization::PoseEkf pose_filter_;
    diesel::localization::HealthConfig health_config_;
    diesel::localization::PoseSourcePolicy policy_{diesel::localization::PoseSourcePolicy::kAuto};
    diesel::localization::PoseSource active_source_{diesel::localization::PoseSource::kUnknown};

    std::mutex mutex_;
    diesel::localization_map::PointCloud latest_cloud_;
    std::int64_t latest_cloud_stamp_ns_{0};
    /// 最近一帧点云的节点接收时刻（ns）。用于判定激光断流，避免依赖消息时间戳。
    std::int64_t last_cloud_rx_ns_{0};
    bool has_cloud_{false};
    /// 最近一次被配准消费的点云时间戳（ns）。TF `map → odom` 用它打戳（audit L6）：
    /// 位姿是配准在那一刻算出来的，标成 `now()` 等于把最多 200 ms 前的解当成当前解。
    std::int64_t last_align_stamp_ns_{0};
    Eigen::Isometry3d odom_to_base_{Eigen::Isometry3d::Identity()};
    std::int64_t last_odom_ns_{0};
    bool has_odom_{false};
    Eigen::Isometry3d map_to_base_{Eigen::Isometry3d::Identity()};
    Eigen::Isometry3d map_to_odom_{Eigen::Isometry3d::Identity()};
    std::int64_t last_valid_ns_{0};
    /// 最近一次被接受的外部位姿时间戳（decision.md D38）：位姿源仲裁要用它判新鲜度，
    /// 父提交没有任何「外部位姿还在不在」的状态，于是接管之后永远翻不回来。
    std::int64_t last_external_ns_{0};
    diesel::localization::HealthStatus health_status_;
    /// 最近一次配准的样本与逐方向可观性（decision.md D43）。状态消息要把「这个位姿有多准」
    /// 和位姿一起发出去，而这两样东西原本只是 `on_align_timer()` 的局部变量，配准结束即丢弃。
    diesel::localization::RegistrationSample last_sample_;
    std::array<bool, diesel::localization_map::kDofCount> observable_{true, true, true,
                                                                      true, true, true};
    /// `filter_unobservable_noise_scale` 的原值。不可观方向的方差倍数与融合层共用同一个旋钮。
    double unobservable_noise_scale_{1.0};
    std::atomic<bool> relocalize_pending_{false};
    Eigen::Isometry3d pending_map_to_base_{Eigen::Isometry3d::Identity()};

    /// Diagnostics 的计数成员。`odom_received_` 用原子量：它要在持有 `mutex_` 之前累加，
    /// 否则「进入次数」就等价于「处理次数」，区分不出排队与丢弃。其余计数都在 `mutex_` 内更新。
    std::atomic<std::uint64_t> odom_received_{0};
    std::uint64_t odom_processed_{0};
    double max_odom_lag_sec_{0.0};
    std::uint64_t align_runs_{0};
    double max_align_ms_{0.0};
    std::uint64_t tf_published_{0};
    /// 先验地图里被跳过的行数（只在 `load_map()` 写一次，`mutex_` 内）。
    std::size_t skipped_pcd_lines_{0};
};

} // namespace diesel::localization_ros

#endif // DIESEL_LOCALIZATION_ROS_LOCALIZATION_NODE_HPP_
