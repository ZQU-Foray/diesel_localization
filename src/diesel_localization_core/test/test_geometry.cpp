// Copyright 2026 ZQU-Foray

#include "diesel/localization/geometry.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace diesel::localization {
namespace {

TEST(WrapAngle, NormalizesToHalfOpenRange) {
    EXPECT_DOUBLE_EQ(wrap_angle(0.0), 0.0);
    EXPECT_NEAR(wrap_angle(kPi), kPi, 1e-12);
    EXPECT_NEAR(wrap_angle(-kPi), kPi, 1e-12);
    EXPECT_NEAR(wrap_angle(3.0 * kPi), kPi, 1e-12);
    EXPECT_NEAR(wrap_angle(kPi + 0.1), -kPi + 0.1, 1e-12);
    EXPECT_NEAR(wrap_angle(-kPi - 0.1), kPi - 0.1, 1e-12);
}

TEST(Geometry, RoundTripsThroughXyzRpy) {
    const Eigen::Isometry3d original = make_transform(1.0, -2.0, 0.5, 0.2, -0.3, 1.1);
    const std::array<double, 6> values = to_xyz_rpy(original);
    const Eigen::Isometry3d rebuilt =
        make_transform(values[0], values[1], values[2], values[3], values[4], values[5]);

    EXPECT_LT((original.matrix() - rebuilt.matrix()).norm(), 1e-9);
    EXPECT_NEAR(yaw_of(original), 1.1, 1e-9);
}

TEST(Geometry, HandlesGimbalLock) {
    const Eigen::Isometry3d original = make_transform(0.0, 0.0, 0.0, 0.4, kPi / 2.0, 0.7);
    const std::array<double, 6> values = to_xyz_rpy(original);
    const Eigen::Isometry3d rebuilt =
        make_transform(values[0], values[1], values[2], values[3], values[4], values[5]);

    EXPECT_LT((original.matrix() - rebuilt.matrix()).norm(), 1e-9);
    EXPECT_DOUBLE_EQ(values[3], 0.0);
}

TEST(Geometry, YawDistanceUsesShortestArc) {
    EXPECT_NEAR(yaw_distance(kPi - 0.1, -kPi + 0.1), 0.2, 1e-12);
    EXPECT_NEAR(yaw_distance(0.0, 0.0), 0.0, 1e-12);
    EXPECT_NEAR(yaw_distance(0.0, 3.0 * kPi), kPi, 1e-12);
}

TEST(Geometry, RejectsNonFiniteTransforms) {
    EXPECT_TRUE(is_finite(pose_from_xy_yaw(1.0, 2.0, 0.5)));

    Eigen::Isometry3d broken = Eigen::Isometry3d::Identity();
    broken.translation().x() = std::nan("");
    EXPECT_FALSE(is_finite(broken));
}

} // namespace
} // namespace diesel::localization
