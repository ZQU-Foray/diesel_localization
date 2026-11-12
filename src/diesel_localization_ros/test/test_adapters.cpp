// Copyright 2026 ZQU-Foray

#include "diesel/localization_ros/adapters.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

namespace {

using diesel::localization_ros::to_isometry;
using diesel::localization_ros::to_point_cloud;
using diesel::localization_ros::validate_header;
using diesel::localization_ros::validate_point_cloud;

sensor_msgs::msg::PointCloud2 make_cloud(const std::vector<float>& xyz, bool with_nan) {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = "front_mid360";
    cloud.height = 1;
    cloud.width = static_cast<std::uint32_t>(xyz.size() / 3U);
    cloud.is_dense = true;
    cloud.point_step = 12;
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(3);
    cloud.fields[0].name = "x";
    cloud.fields[0].offset = 0;
    cloud.fields[1].name = "y";
    cloud.fields[1].offset = 4;
    cloud.fields[2].name = "z";
    cloud.fields[2].offset = 8;
    for (auto& field : cloud.fields) {
        field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field.count = 1;
    }
    cloud.data.resize(cloud.row_step);
    std::memcpy(cloud.data.data(), xyz.data(), cloud.data.size());
    if (with_nan) {
        const float nan = std::numeric_limits<float>::quiet_NaN();
        std::memcpy(cloud.data.data(), &nan, sizeof(float));
    }
    return cloud;
}

TEST(AdaptersTest, ConvertsPointCloudAndSkipsNan) {
    const auto cloud = make_cloud({1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F}, true);
    std::string error;
    ASSERT_TRUE(validate_point_cloud(cloud, &error)) << error;

    const auto points = to_point_cloud(cloud, &error);
    ASSERT_EQ(points.size(), 1U);
    EXPECT_FLOAT_EQ(points.points[0].x(), 4.0F);
}

TEST(AdaptersTest, RejectsCloudWithoutFields) {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.width = 1;
    cloud.height = 1;
    cloud.point_step = 4;
    cloud.row_step = 4;
    cloud.data.resize(4);
    std::string error;
    EXPECT_FALSE(validate_point_cloud(cloud, &error));
    EXPECT_FALSE(error.empty());
}

TEST(AdaptersTest, ValidatesHeaderStampAndFrame) {
    std_msgs::msg::Header header;
    header.frame_id = "map";
    std::string error;
    EXPECT_FALSE(validate_header(header, "map", &error));

    header.stamp.sec = 1;
    EXPECT_TRUE(validate_header(header, "map", &error)) << error;
    EXPECT_FALSE(validate_header(header, "odom", &error));
}

TEST(AdaptersTest, ConvertsPoseAndRejectsZeroQuaternion) {
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = 1.0;
    Eigen::Isometry3d transform;
    ASSERT_TRUE(to_isometry(pose, &transform));
    EXPECT_DOUBLE_EQ(transform.translation().x(), 1.0);

    pose.orientation.w = 0.0;
    EXPECT_FALSE(to_isometry(pose, &transform));
}

} // namespace
