// Copyright 2026 ZQU-Foray
//
// ROS 消息与 core/map 值类型的转换。保持纯函数，便于单测与复用。

#ifndef DIESEL_LOCALIZATION_ROS_CONVERSION_HPP_
#define DIESEL_LOCALIZATION_ROS_CONVERSION_HPP_

#include "diesel/localization/health.hpp"
#include "diesel/localization/manifest.hpp"
#include "diesel/localization/pose.hpp"
#include "diesel/localization_map/residual_clusterer.hpp"

#include "diesel_localization_ros/msg/dynamic_obstacle_array.hpp"
#include "diesel_localization_ros/msg/localization_state.hpp"
#include "diesel_localization_ros/msg/map_meta.hpp"

#include <Eigen/Geometry>

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace diesel::localization_ros {

using LocalizationStateMsg = diesel_localization_ros::msg::LocalizationState;
using MapMetaMsg = diesel_localization_ros::msg::MapMeta;
using DynamicObstacleArrayMsg = diesel_localization_ros::msg::DynamicObstacleArray;

/// 时间戳（纳秒）与 ROS 时间互转。
builtin_interfaces::msg::Time to_msg_time(std::int64_t stamp_ns);
std::int64_t to_ns(const builtin_interfaces::msg::Time& stamp);

/// SE(3) 与 geometry_msgs::Pose 互转（四元数归一化）。
geometry_msgs::msg::Pose to_msg_pose(const Eigen::Isometry3d& transform);
Eigen::Isometry3d to_eigen_pose(const geometry_msgs::msg::Pose& pose);

/// 健康状态与位姿来源的枚举映射。
std::uint8_t to_msg_state(diesel::localization::HealthState state);
std::uint8_t to_msg_source(diesel::localization::PoseSource source);

/// 位姿消息里协方差的自由度个数与顺序：x, y, z, rotX, rotY, rotZ（行主序 6×6）。
inline constexpr std::size_t kPoseDofCount = 6;

/// 组装 `PoseWithCovariance` 所需的不确定性输入（decision.md D43）。
///
/// 父提交把协方差整块填 0（审计 L7）：在 `PoseWithCovariance` 的语义里 0 方差意味着
/// 「这个位姿绝对准确」，于是任何按协方差加权的下游都会把配准结果当成无限可信的观测，
/// 同时又把「其实不知道」这件事彻底藏起来。
struct PoseUncertainty {
    /// 本次位姿是否带可用的配准质量（残差 / 内点率 / 可观性）。
    /// 从未配准、状态尚未完成热启动、或位姿来自外部位姿源时为 false：此时只能如实
    /// 声明「不知道有多好」，不能假装质量很高（与 `RegistrationSample::has_registration_quality`
    /// 同一条口径，D38）。
    bool has_registration_quality{false};
    /// 最近一次配准的平均内点残差（米）。
    double mean_residual_m{0.0};
    /// 最近一次配准的内点占比，范围 [0, 1]。
    double inlier_ratio{0.0};
    /// 6 个自由度是否被本帧点云约束住。false 的方向由里程计承担，方差要放大。
    std::array<bool, kPoseDofCount> observable{true, true, true, true, true, true};
    /// 不可观方向的「标准差倍数」，取自 ROS 参数 `filter_unobservable_noise_scale`。
    /// 语义是标准差倍数，落到方差上是平方关系。
    double unobservable_scale{1.0};
};

/// 由不确定性输入算出 `PoseWithCovariance::covariance` 的 36 个值（行主序 6×6）。
///
/// 只填对角：配准残差没有给出方向间的相关性，编造非对角项等于凭空创造信息。
std::array<double, kPoseDofCount * kPoseDofCount> pose_covariance(const PoseUncertainty& input);

/// 组装 LocalizationState：header.stamp 取 now_ns，pose 取 map 下位姿。
/// `pose_valid` 由健康状态推导：OK/DEGRADED 为 true，INITIALIZING/LOST 为 false
/// —— 后两者下 pose 只是未经校正或已停更的合成值，禁止下游消费（审计 B2）。
/// `uncertainty` 必填：位姿与它的可信度必须一起发布（审计 L7）。
LocalizationStateMsg to_msg_state(const diesel::localization::HealthStatus& health,
                                  const diesel::localization::Pose3D& pose,
                                  diesel::localization::PoseSource source, std::int64_t now_ns,
                                  const PoseUncertainty& uncertainty);

MapMetaMsg to_msg_map_meta(const diesel::localization::MapManifest& manifest, std::int64_t now_ns);

DynamicObstacleArrayMsg
to_msg_obstacles(const std::vector<diesel::localization_map::ResidualCluster>& clusters,
                 const std::string& frame_id, std::int64_t stamp_ns);

} // namespace diesel::localization_ros

#endif // DIESEL_LOCALIZATION_ROS_CONVERSION_HPP_
