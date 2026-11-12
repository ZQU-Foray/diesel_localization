// Copyright 2026 ZQU-Foray

#include "diesel/localization/geometry.hpp"

#include <algorithm>
#include <cmath>

namespace diesel::localization {

double wrap_angle(double angle) {
    if (!std::isfinite(angle)) {
        return angle;
    }
    angle = std::fmod(angle, 2.0 * kPi);
    if (angle <= -kPi) {
        angle += 2.0 * kPi;
    } else if (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    return angle;
}

Eigen::Isometry3d make_transform(double x, double y, double z, double roll, double pitch,
                                 double yaw) {
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.linear() = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
                             .toRotationMatrix();
    transform.translation() = Eigen::Vector3d(x, y, z);
    return transform;
}

Eigen::Isometry3d pose_from_xy_yaw(double x, double y, double yaw) {
    return make_transform(x, y, 0.0, 0.0, 0.0, yaw);
}

std::array<double, 6> to_xyz_rpy(const Eigen::Isometry3d& transform) {
    const Eigen::Matrix3d rotation = transform.linear();
    // R = Rz(yaw) * Ry(pitch) * Rx(roll) 时 R(2,0) = -sin(pitch)。
    const double sin_pitch = std::clamp(-rotation(2, 0), -1.0, 1.0);
    const double pitch = std::asin(sin_pitch);

    double roll = 0.0;
    double yaw = 0.0;
    if (std::abs(sin_pitch) < 1.0 - 1e-9) {
        roll = std::atan2(rotation(2, 1), rotation(2, 2));
        yaw = std::atan2(rotation(1, 0), rotation(0, 0));
    } else {
        // 万向锁：roll/yaw 只有组合可观测，固定 roll = 0。
        roll = 0.0;
        yaw = std::atan2(-rotation(0, 1), rotation(1, 1));
    }

    const Eigen::Vector3d translation = transform.translation();
    return {translation.x(), translation.y(), translation.z(), roll, pitch, yaw};
}

double yaw_of(const Eigen::Isometry3d& transform) {
    const Eigen::Matrix3d rotation = transform.linear();
    return std::atan2(rotation(1, 0), rotation(0, 0));
}

double yaw_distance(double lhs, double rhs) {
    return std::abs(wrap_angle(lhs - rhs));
}

bool is_finite(const Eigen::Isometry3d& transform) {
    return transform.matrix().allFinite();
}

} // namespace diesel::localization
