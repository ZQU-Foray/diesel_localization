// Copyright 2026 ZQU-Foray

#include "diesel/localization_ros/conversion.hpp"

#include <algorithm>
#include <cmath>

namespace diesel::localization_ros {

namespace {

/// 没有任何配准质量信息时的对角方差（m² / rad²）。语义是「不知道有多好」。
/// 不能填 0：`PoseWithCovariance` 里 0 方差等于「无限权重」，会把一个真实未知量
/// 伪装成绝对真值（审计 L7）。
constexpr double kNoInformationVariance = 1000.0;
/// 残差下限 1 cm。亚厘米残差不能当成「方差 ≈ 0」的证据，否则又回到无限权重。
constexpr double kMinSigmaM = 0.01;
/// 标准差上限 10 m / 10 rad。再大就等同于「无信息」，与 `kNoInformationVariance` 同量级。
constexpr double kMaxSigmaM = 10.0;
/// 内点率下限 0.05。内点率 0 时 `1 / sqrt(ratio)` 发散，按 5 % 截断（最多放大 4.5 倍）。
constexpr double kMinInlierRatio = 0.05;
/// 残差（米）折算成角度（弧度）的杠杆长度。MID360 在 12 m × 8 m 场地上，参与约束的
/// 点位到机器人的力臂量级为 1 m，故 1 m 残差 ≈ 1 rad 的角度不确定度（场地尺寸见 README）。
constexpr double kRotationLeverArmM = 1.0;
/// 不可观方向的放大倍数下限。参数被配成 < 1 时不允许「反向信任」不可观方向。
constexpr double kMinUnobservableScale = 1.0;

/// 把非有限输入折算成「最坏情况」，而不是让它悄悄变成 NaN 方差。
double finite_or(double value, double fallback) {
    return std::isfinite(value) ? value : fallback;
}

} // namespace

std::array<double, kPoseDofCount * kPoseDofCount> pose_covariance(const PoseUncertainty& input) {
    std::array<double, kPoseDofCount * kPoseDofCount> covariance{};
    if (!input.has_registration_quality) {
        for (std::size_t dof = 0; dof < kPoseDofCount; ++dof) {
            covariance[dof * kPoseDofCount + dof] = kNoInformationVariance;
        }
        return covariance;
    }

    // 残差是米制的直接观测量；内点率越低，同一个残差对均值的代表性越差，
    // 因此按 1 / sqrt(内点率) 放大标准差（等价于按 1 / 内点率 放大方差）。
    const double residual = finite_or(input.mean_residual_m, kMaxSigmaM);
    const double inlier_ratio =
        std::clamp(finite_or(input.inlier_ratio, 0.0), kMinInlierRatio, 1.0);
    const double sigma_position = std::clamp(
        std::max(residual, kMinSigmaM) / std::sqrt(inlier_ratio), kMinSigmaM, kMaxSigmaM);
    const double sigma_rotation =
        std::clamp(sigma_position / kRotationLeverArmM, kMinSigmaM, kMaxSigmaM);
    const double scale =
        std::max(finite_or(input.unobservable_scale, kMinUnobservableScale), kMinUnobservableScale);
    const double scale_squared = scale * scale;

    for (std::size_t dof = 0; dof < kPoseDofCount; ++dof) {
        const double sigma = dof < 3 ? sigma_position : sigma_rotation;
        double variance = sigma * sigma;
        // 不可观方向：本帧点云在这些自由度上没有约束，位姿全靠里程计承担，
        // 因此方差按 `filter_unobservable_noise_scale` 的平方放大（默认 10 倍 → 100 倍方差）。
        if (!input.observable[dof]) {
            variance *= scale_squared;
        }
        covariance[dof * kPoseDofCount + dof] = variance;
    }
    return covariance;
}

builtin_interfaces::msg::Time to_msg_time(std::int64_t stamp_ns) {
    builtin_interfaces::msg::Time stamp;
    constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;
    stamp.sec = static_cast<std::int32_t>(stamp_ns / kNanosecondsPerSecond);
    stamp.nanosec = static_cast<std::uint32_t>(stamp_ns % kNanosecondsPerSecond);
    return stamp;
}

std::int64_t to_ns(const builtin_interfaces::msg::Time& stamp) {
    constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;
    return static_cast<std::int64_t>(stamp.sec) * kNanosecondsPerSecond +
           static_cast<std::int64_t>(stamp.nanosec);
}

geometry_msgs::msg::Pose to_msg_pose(const Eigen::Isometry3d& transform) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = transform.translation().x();
    pose.position.y = transform.translation().y();
    pose.position.z = transform.translation().z();
    const Eigen::Quaterniond quaternion(transform.linear());
    const Eigen::Quaterniond normalized = quaternion.normalized();
    pose.orientation.x = normalized.x();
    pose.orientation.y = normalized.y();
    pose.orientation.z = normalized.z();
    pose.orientation.w = normalized.w();
    return pose;
}

Eigen::Isometry3d to_eigen_pose(const geometry_msgs::msg::Pose& pose) {
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    const Eigen::Quaterniond quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y,
                                        pose.orientation.z);
    if (quaternion.norm() > 1e-9) {
        transform.linear() = quaternion.normalized().toRotationMatrix();
    }
    transform.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    return transform;
}

std::uint8_t to_msg_state(diesel::localization::HealthState state) {
    switch (state) {
    case diesel::localization::HealthState::kInitializing:
        return LocalizationStateMsg::STATE_INITIALIZING;
    case diesel::localization::HealthState::kOk:
        return LocalizationStateMsg::STATE_OK;
    case diesel::localization::HealthState::kDegraded:
        return LocalizationStateMsg::STATE_DEGRADED;
    case diesel::localization::HealthState::kLost:
        return LocalizationStateMsg::STATE_LOST;
    }
    return LocalizationStateMsg::STATE_LOST;
}

std::uint8_t to_msg_source(diesel::localization::PoseSource source) {
    switch (source) {
    case diesel::localization::PoseSource::kSentryLidar:
        return LocalizationStateMsg::SOURCE_SENTRY_LIDAR;
    case diesel::localization::PoseSource::kExternalOdom:
        return LocalizationStateMsg::SOURCE_EXTERNAL_ODOM;
    case diesel::localization::PoseSource::kUnknown:
        return LocalizationStateMsg::SOURCE_UNKNOWN;
    }
    return LocalizationStateMsg::SOURCE_UNKNOWN;
}

LocalizationStateMsg to_msg_state(const diesel::localization::HealthStatus& health,
                                  const diesel::localization::Pose3D& pose,
                                  diesel::localization::PoseSource source, std::int64_t now_ns,
                                  const PoseUncertainty& uncertainty) {
    LocalizationStateMsg message;
    message.header.stamp = to_msg_time(now_ns);
    message.header.frame_id = pose.parent_frame;
    message.pose.pose = to_msg_pose(pose.transform);
    // 协方差按残差 / 内点率 / 可观性如实填充（decision.md D43）。父提交这里整块填 0，
    // 等于向所有按协方差加权的下游宣称「这个位姿绝对准确」（审计 L7）。
    message.pose.covariance = pose_covariance(uncertainty);
    // 只有 OK/DEGRADED 的 pose 可被下游消费；INITIALIZING 还没完成热启动、LOST 已停更
    // map->odom，两者的 pose 都只是诊断值（审计 B2：禁止静默外推）。
    message.pose_valid = health.state == diesel::localization::HealthState::kOk ||
                         health.state == diesel::localization::HealthState::kDegraded;
    message.confidence = health.confidence;
    message.age_sec = health.age_sec;
    message.state = to_msg_state(health.state);
    message.source = to_msg_source(source);
    message.reason = health.reason;
    return message;
}

MapMetaMsg to_msg_map_meta(const diesel::localization::MapManifest& manifest, std::int64_t now_ns) {
    MapMetaMsg message;
    message.header.stamp = to_msg_time(now_ns);
    message.header.frame_id = manifest.frame_id;
    message.version = manifest.version;
    message.origin.x = manifest.origin[0];
    message.origin.y = manifest.origin[1];
    message.origin.z = manifest.origin[2];
    message.resolution = manifest.resolution;
    message.width = static_cast<std::uint32_t>(manifest.width);
    message.height = static_cast<std::uint32_t>(manifest.height);
    message.pcd_sha256 = manifest.pcd_sha256;
    message.grid_sha256 = manifest.grid_sha256;
    message.source = manifest.source;
    return message;
}

DynamicObstacleArrayMsg
to_msg_obstacles(const std::vector<diesel::localization_map::ResidualCluster>& clusters,
                 const std::string& frame_id, std::int64_t stamp_ns) {
    DynamicObstacleArrayMsg message;
    message.header.stamp = to_msg_time(stamp_ns);
    message.header.frame_id = frame_id;
    message.obstacles.reserve(clusters.size());
    for (const auto& cluster : clusters) {
        diesel_localization_ros::msg::DynamicObstacle obstacle;
        obstacle.centroid.x = cluster.centroid.x();
        obstacle.centroid.y = cluster.centroid.y();
        obstacle.centroid.z = cluster.centroid.z();
        obstacle.size.x = cluster.max_corner.x() - cluster.min_corner.x();
        obstacle.size.y = cluster.max_corner.y() - cluster.min_corner.y();
        obstacle.size.z = cluster.max_corner.z() - cluster.min_corner.z();
        obstacle.point_count = static_cast<std::uint32_t>(cluster.point_count);
        obstacle.confidence = cluster.confidence;
        obstacle.mean_residual_m = cluster.mean_residual_m;
        message.obstacles.push_back(obstacle);
    }
    return message;
}

} // namespace diesel::localization_ros
