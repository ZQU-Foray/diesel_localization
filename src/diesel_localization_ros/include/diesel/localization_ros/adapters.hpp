// Copyright 2026 ZQU-Foray
//
// 传感器输入适配：把 ROS 消息校验/转换为内部类型。所有函数不抛异常，
// 失败通过返回值与 error 字符串上报，供节点降级而不是崩溃。

#ifndef DIESEL_LOCALIZATION_ROS_ADAPTERS_HPP_
#define DIESEL_LOCALIZATION_ROS_ADAPTERS_HPP_

#include "diesel/localization_map/point_cloud.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"

#include <Eigen/Geometry>

#include <cstdint>
#include <string>

namespace diesel::localization_ros {

bool to_timestamp_ns(const builtin_interfaces::msg::Time& stamp, std::int64_t* stamp_ns);

/// 校验 x/y/z 字段存在且为 32 位浮点，宽高乘积非零。
bool validate_point_cloud(const sensor_msgs::msg::PointCloud2& cloud, std::string* error);

/// 转换点云；非法点（NaN/Inf）被跳过。失败时 error 非空。
diesel::localization_map::PointCloud to_point_cloud(const sensor_msgs::msg::PointCloud2& cloud,
                                                   std::string* error);

/// 位姿必须全部有限，四元数模长不能接近 0。
bool to_isometry(const geometry_msgs::msg::Pose& pose, Eigen::Isometry3d* transform);

bool validate_odometry(const nav_msgs::msg::Odometry& odometry, std::string* error);

/// Header 的 frame_id 非空且 stamp 非零。
bool validate_header(const std_msgs::msg::Header& header, const char* expected_frame,
                     std::string* error);

} // namespace diesel::localization_ros

#endif // DIESEL_LOCALIZATION_ROS_ADAPTERS_HPP_
