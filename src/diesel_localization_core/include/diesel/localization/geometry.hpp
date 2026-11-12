// Copyright 2026 ZQU-Foray
//
// 与 ROS 无关的 SE(3)/欧拉角工具。角度单位统一为弧度。

#ifndef DIESEL_LOCALIZATION_GEOMETRY_HPP_
#define DIESEL_LOCALIZATION_GEOMETRY_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>

namespace diesel::localization {

inline constexpr double kPi = 3.14159265358979323846;

/// 归一化到 (-pi, pi]。
double wrap_angle(double angle);

/// 由 `x, y, z`（米）与 `roll, pitch, yaw`（弧度）构造变换，旋转顺序 Rz * Ry * Rx。
Eigen::Isometry3d make_transform(double x, double y, double z, double roll, double pitch,
                                 double yaw);

/// 由平面位姿构造变换（z/roll/pitch 为 0）。
Eigen::Isometry3d pose_from_xy_yaw(double x, double y, double yaw);

/// 逆解为 `[x, y, z, roll, pitch, yaw]`。万向锁时 roll 置 0，yaw 取等价解。
std::array<double, 6> to_xyz_rpy(const Eigen::Isometry3d& transform);

/// 提取绕 +Z 的偏航角。
double yaw_of(const Eigen::Isometry3d& transform);

/// 两个偏航角的最短角距，范围 [0, pi]。
double yaw_distance(double lhs, double rhs);

/// 检查变换是否全为有限值（NaN/Inf 保护）。
bool is_finite(const Eigen::Isometry3d& transform);

} // namespace diesel::localization

#endif // DIESEL_LOCALIZATION_GEOMETRY_HPP_
