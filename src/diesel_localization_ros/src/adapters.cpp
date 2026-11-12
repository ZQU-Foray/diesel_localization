// Copyright 2026 ZQU-Foray

#include "diesel/localization_ros/adapters.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cmath>
#include <string>

namespace diesel::localization_ros {
namespace {

bool has_float_field(const sensor_msgs::msg::PointCloud2& cloud, const std::string& name) {
    for (const auto& field : cloud.fields) {
        if (field.name == name) {
            return field.datatype == sensor_msgs::msg::PointField::FLOAT32;
        }
    }
    return false;
}

} // namespace

bool to_timestamp_ns(const builtin_interfaces::msg::Time& stamp, std::int64_t* stamp_ns) {
    if (stamp_ns == nullptr || stamp.sec < 0 || stamp.nanosec >= 1000000000U) {
        return false;
    }
    constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;
    *stamp_ns = static_cast<std::int64_t>(stamp.sec) * kNanosecondsPerSecond +
                static_cast<std::int64_t>(stamp.nanosec);
    return true;
}

bool validate_point_cloud(const sensor_msgs::msg::PointCloud2& cloud, std::string* error) {
    if (cloud.width == 0U || cloud.height == 0U) {
        *error = "点云宽或高为 0";
        return false;
    }
    if (!has_float_field(cloud, "x") || !has_float_field(cloud, "y") ||
        !has_float_field(cloud, "z")) {
        *error = "点云缺少 FLOAT32 的 x/y/z 字段";
        return false;
    }
    if (cloud.point_step == 0U || cloud.row_step < cloud.width * cloud.point_step) {
        *error = "点云 point_step/row_step 非法";
        return false;
    }
    if (cloud.data.size() < cloud.row_step * cloud.height) {
        *error = "点云 data 长度小于 row_step*height";
        return false;
    }
    return true;
}

diesel::localization_map::PointCloud to_point_cloud(const sensor_msgs::msg::PointCloud2& cloud,
                                                    std::string* error) {
    diesel::localization_map::PointCloud result;
    if (!validate_point_cloud(cloud, error)) {
        return result;
    }

    try {
        sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
        const std::size_t count = static_cast<std::size_t>(cloud.width) * cloud.height;
        result.points.reserve(count);
        for (std::size_t index = 0; index < count; ++index, ++x, ++y, ++z) {
            if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z)) {
                result.points.emplace_back(*x, *y, *z);
            }
        }
    } catch (const std::exception& exception) {
        result.points.clear();
        *error = std::string("点云迭代失败: ") + exception.what();
    }
    return result;
}

bool to_isometry(const geometry_msgs::msg::Pose& pose, Eigen::Isometry3d* transform) {
    if (transform == nullptr) {
        return false;
    }
    const Eigen::Quaterniond quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y,
                                        pose.orientation.z);
    if (quaternion.norm() < 1e-6) {
        return false;
    }
    if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
        !std::isfinite(pose.position.z) || !std::isfinite(quaternion.x()) ||
        !std::isfinite(quaternion.y()) || !std::isfinite(quaternion.z()) ||
        !std::isfinite(quaternion.w())) {
        return false;
    }
    transform->setIdentity();
    transform->linear() = quaternion.normalized().toRotationMatrix();
    transform->translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    return true;
}

bool validate_header(const std_msgs::msg::Header& header, const char* expected_frame,
                     std::string* error) {
    if (header.frame_id.empty()) {
        *error = "header.frame_id 为空";
        return false;
    }
    if (expected_frame != nullptr && expected_frame[0] != '\0' &&
        header.frame_id != expected_frame) {
        *error = "header.frame_id 不匹配: 期望 " + std::string(expected_frame) + " 实际 " +
                 header.frame_id;
        return false;
    }
    if (header.stamp.sec == 0 && header.stamp.nanosec == 0U) {
        *error = "header.stamp 为 0（未打时间戳）";
        return false;
    }
    return true;
}

bool validate_odometry(const nav_msgs::msg::Odometry& odometry, std::string* error) {
    if (!validate_header(odometry.header, nullptr, error)) {
        return false;
    }
    if (odometry.child_frame_id.empty()) {
        *error = "odometry.child_frame_id 为空";
        return false;
    }
    Eigen::Isometry3d transform;
    if (!to_isometry(odometry.pose.pose, &transform)) {
        *error = "odometry 位姿含非法数值或四元数为零";
        return false;
    }
    return true;
}

} // namespace diesel::localization_ros
