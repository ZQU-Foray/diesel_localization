// Copyright 2026 ZQU-Foray

#include "foray/localization_map/point_cloud.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

using foray::localization_map::crop_by_fov;
using foray::localization_map::PointCloud;
using foray::localization_map::transform_points;
using foray::localization_map::voxel_downsample;

TEST(PointCloudTest, EmptyCloudStaysEmpty) {
    const PointCloud cloud;
    EXPECT_TRUE(cloud.empty());
    EXPECT_EQ(voxel_downsample(cloud, 0.5).size(), 0U);
}

TEST(PointCloudTest, VoxelDownsampleCollapsesSingleVoxelToMean) {
    PointCloud cloud;
    cloud.points.emplace_back(0.1F, 0.1F, 0.1F);
    cloud.points.emplace_back(0.9F, 0.9F, 0.9F);
    cloud.points.emplace_back(0.5F, 0.5F, 0.5F);

    const PointCloud downsampled = voxel_downsample(cloud, 1.0);
    ASSERT_EQ(downsampled.size(), 1U);
    EXPECT_NEAR(downsampled.points[0].x(), 0.5F, 1e-5F);
    EXPECT_NEAR(downsampled.points[0].y(), 0.5F, 1e-5F);
    EXPECT_NEAR(downsampled.points[0].z(), 0.5F, 1e-5F);
}

TEST(PointCloudTest, VoxelDownsampleKeepsSeparateVoxels) {
    PointCloud cloud;
    cloud.points.emplace_back(0.1F, 0.1F, 0.1F);
    cloud.points.emplace_back(1.1F, 0.1F, 0.1F);

    EXPECT_EQ(voxel_downsample(cloud, 0.5).size(), 2U);
}

TEST(PointCloudTest, TransformAppliesRigidMotion) {
    PointCloud cloud;
    cloud.points.emplace_back(1.0F, 0.0F, 0.0F);

    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translate(Eigen::Vector3d(0.0, 2.0, 0.0));

    const PointCloud transformed = transform_points(cloud, transform);
    ASSERT_EQ(transformed.size(), 1U);
    EXPECT_NEAR(transformed.points[0].x(), 1.0F, 1e-5F);
    EXPECT_NEAR(transformed.points[0].y(), 2.0F, 1e-5F);
    EXPECT_NEAR(transformed.points[0].z(), 0.0F, 1e-5F);
}

TEST(PointCloudTest, CropByFovKeepsEverythingWhenFullFieldOfView) {
    PointCloud cloud;
    cloud.points.emplace_back(1.0F, 0.0F, 0.0F);
    cloud.points.emplace_back(-1.0F, 0.0F, 0.0F);
    cloud.points.emplace_back(0.0F, 1.0F, 0.0F);
    cloud.points.emplace_back(0.0F, 0.0F, 1.0F);

    // 360x180 是「不裁切」的同一个代码路径——默认配置必须与改造前逐字节等价。
    const PointCloud kept = crop_by_fov(cloud, 360.0, 180.0, 0.0);
    EXPECT_EQ(kept.size(), cloud.size());
}

TEST(PointCloudTest, CropByFovRemovesPointsOutsideHorizontalOpening) {
    PointCloud cloud;
    cloud.points.emplace_back(1.0F, 0.0F, 0.0F);   // 方位 0 度，正对中心
    cloud.points.emplace_back(0.0F, 1.0F, 0.0F);   // 方位 90 度
    cloud.points.emplace_back(-1.0F, 0.0F, 0.0F);  // 方位 180 度（正后方）

    // 120 度开口 = 偏离中心 ±60 度：只有方位 0 度那个点留下。
    const PointCloud kept = crop_by_fov(cloud, 120.0, 180.0, 0.0);
    ASSERT_EQ(kept.size(), 1U);
    EXPECT_NEAR(kept.points[0].x(), 1.0F, 1e-5F);
}

TEST(PointCloudTest, CropByFovRotatesWithCenterYaw) {
    PointCloud cloud;
    cloud.points.emplace_back(1.0F, 0.0F, 0.0F);   // 方位 0 度
    cloud.points.emplace_back(0.0F, 1.0F, 0.0F);   // 方位 90 度

    // 中心转到 90 度、只看 60 度开口：只剩正 y 方向那个点。
    const PointCloud kept = crop_by_fov(cloud, 60.0, 180.0, 90.0);
    ASSERT_EQ(kept.size(), 1U);
    EXPECT_NEAR(kept.points[0].y(), 1.0F, 1e-5F);
}

TEST(PointCloudTest, CropByFovAppliesVerticalOpening) {
    PointCloud cloud;
    cloud.points.emplace_back(1.0F, 0.0F, 0.0F);
    cloud.points.emplace_back(0.0F, 0.0F, 1.0F);  // 正上方，俯仰 90 度

    const PointCloud kept = crop_by_fov(cloud, 360.0, 40.0, 0.0);
    ASSERT_EQ(kept.size(), 1U);
    EXPECT_NEAR(kept.points[0].x(), 1.0F, 1e-5F);
}

} // namespace
