// Copyright 2026 ZQU-Foray
//
// 定位主节点：地图配准、健康状态、TF map->odom、动态障碍与手动重定位。
// 不直接访问任何硬件；输入是标准 ROS 话题（decision.md D7）。

#ifndef FORAY_LOCALIZATION_ROS_LOCALIZATION_NODE_HPP_
#define FORAY_LOCALIZATION_ROS_LOCALIZATION_NODE_HPP_

#include "foray/localization/health.hpp"
#include "foray/localization/pose.hpp"
#include "foray/localization/pose_ekf.hpp"
#include "foray/localization_map/aligner.hpp"
#include "foray/localization_map/aligner_factory.hpp"
#include "foray/localization_map/observability.hpp"
#include "foray/localization_map/prior_map.hpp"
#include "foray/localization_map/residual_clusterer.hpp"
#include "foray_localization_ros/msg/dynamic_obstacle_array.hpp"
#include "foray_localization_ros/msg/localization_state.hpp"
#include "foray_localization_ros/msg/map_meta.hpp"
#include "foray_localization_ros/srv/relocalize.hpp"

#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

namespace foray::localization_ros {

class LocalizationNode : public rclcpp::Node {
  public:
    /// `aligner` 非空时替代工厂构造的后端（decision.md D21 的可替换点）。该参数只服务于
    /// 离线回放与节点级测试：生产入口 `localization_node_main.cpp` 不传它，走编译期开关。
    explicit LocalizationNode(const rclcpp::NodeOptions& options,
                              std::unique_ptr<foray::localization_map::Aligner> aligner = nullptr);

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
    on_relocalize(const std::shared_ptr<foray_localization_ros::srv::Relocalize::Request> request,
                  std::shared_ptr<foray_localization_ros::srv::Relocalize::Response> response);

    void load_map();
    void publish_state();
    void update_health(const foray::localization_map::AlignResult& result, std::int64_t stamp_ns);
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
        std::size_t samples{0};
        std::int64_t window_start_ns{0};
    };
    bool profile_enabled_{false};
    ProfileTotals profile_;

    void maybe_flush_profile(std::int64_t now_ns);

    rclcpp::Publisher<foray_localization_ros::msg::LocalizationState>::SharedPtr state_pub_;
    rclcpp::Publisher<foray_localization_ros::msg::MapMeta>::SharedPtr map_meta_pub_;
    rclcpp::Publisher<foray_localization_ros::msg::DynamicObstacleArray>::SharedPtr obstacle_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        external_pose_sub_;
    rclcpp::Service<foray_localization_ros::srv::Relocalize>::SharedPtr relocalize_srv_;

    rclcpp::TimerBase::SharedPtr state_timer_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
    rclcpp::TimerBase::SharedPtr align_timer_;
    rclcpp::TimerBase::SharedPtr obstacle_timer_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    foray::localization_map::PriorMap map_;
    bool has_map_{false};
    /// 目标侧近邻索引与法向缓存，随地图生命周期只建一次（decision.md D25）。
    /// 两者都持有 `map_.cloud` 的引用，必须在 `map_` 之后声明、在 relocad 时整体重建。
    std::shared_ptr<foray::localization_map::PointCloudIndex> map_index_;
    std::shared_ptr<foray::localization_map::NormalCache> map_normals_;
    /// 配准后端由工厂按编译期开关构造（decision.md D21），节点不关心具体实现。
    std::unique_ptr<foray::localization_map::Aligner> aligner_;
    foray::localization_map::ResidualClusterer clusterer_;
    foray::localization::HealthMonitor health_;
    /// 融合层滤波（decision.md D24）。默认关闭，关闭时位姿更新与改造前完全一致。
    foray::localization::PoseEkf pose_filter_;
    foray::localization::HealthConfig health_config_;
    foray::localization::PoseSourcePolicy policy_{foray::localization::PoseSourcePolicy::kAuto};
    foray::localization::PoseSource active_source_{foray::localization::PoseSource::kUnknown};

    std::mutex mutex_;
    foray::localization_map::PointCloud latest_cloud_;
    std::int64_t latest_cloud_stamp_ns_{0};
    /// 最近一帧点云的节点接收时刻（ns）。用于判定激光断流，避免依赖消息时间戳。
    std::int64_t last_cloud_rx_ns_{0};
    bool has_cloud_{false};
    Eigen::Isometry3d odom_to_base_{Eigen::Isometry3d::Identity()};
    std::int64_t last_odom_ns_{0};
    bool has_odom_{false};
    Eigen::Isometry3d map_to_base_{Eigen::Isometry3d::Identity()};
    Eigen::Isometry3d map_to_odom_{Eigen::Isometry3d::Identity()};
    std::int64_t last_valid_ns_{0};
    foray::localization::HealthStatus health_status_;
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
};

} // namespace foray::localization_ros

#endif // FORAY_LOCALIZATION_ROS_LOCALIZATION_NODE_HPP_
