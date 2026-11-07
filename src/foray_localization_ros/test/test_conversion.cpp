// Copyright 2026 ZQU-Foray

#include "foray/localization_ros/conversion.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace {

using foray::localization::HealthState;
using foray::localization::HealthStatus;
using foray::localization::MapManifest;
using foray::localization::Pose3D;
using foray::localization::PoseSource;
using foray::localization_map::ResidualCluster;
using foray::localization_ros::kPoseDofCount;
using foray::localization_ros::pose_covariance;
using foray::localization_ros::PoseUncertainty;
using foray::localization_ros::to_msg_map_meta;
using foray::localization_ros::to_msg_obstacles;
using foray::localization_ros::to_msg_pose;
using foray::localization_ros::to_msg_state;
using foray::localization_ros::to_msg_time;

/// 行主序 6×6 里第 `dof` 个对角项的偏移。
constexpr std::size_t diag_offset(std::size_t dof) {
    return dof * kPoseDofCount + dof;
}

/// 一帧「好」的配准：残差 5 cm、内点率 90 %，6 个方向都被约束。
PoseUncertainty good_frame() {
    PoseUncertainty input;
    input.has_registration_quality = true;
    input.mean_residual_m = 0.05;
    input.inlier_ratio = 0.9;
    input.unobservable_scale = 10.0;
    return input;
}

TEST(ConversionTest, StateCarriesHealthAndPose) {
    HealthStatus health;
    health.state = HealthState::kDegraded;
    health.confidence = 0.42;
    health.age_sec = 0.25;
    health.reason = "残差偏大";

    Pose3D pose;
    pose.parent_frame = "map";
    pose.child_frame = "base_footprint";
    pose.transform = Eigen::Isometry3d::Identity();
    pose.transform.translation() = Eigen::Vector3d(1.0, 2.0, 0.0);

    const auto message =
        to_msg_state(health, pose, PoseSource::kSentryLidar, 1500000000LL, good_frame());
    EXPECT_EQ(message.state, message.STATE_DEGRADED);
    EXPECT_EQ(message.source, message.SOURCE_SENTRY_LIDAR);
    EXPECT_TRUE(message.pose_valid);
    EXPECT_DOUBLE_EQ(message.confidence, 0.42);
    EXPECT_EQ(message.header.frame_id, "map");
    EXPECT_EQ(message.header.stamp.sec, 1);
    EXPECT_EQ(message.header.stamp.nanosec, 500000000U);
    EXPECT_DOUBLE_EQ(message.pose.pose.position.x, 1.0);
    EXPECT_EQ(message.reason, "残差偏大");
}

TEST(ConversionTest, PoseValidFollowsHealthState) {
    // 审计 B2：只有完成校正的状态才允许下游消费 pose。
    struct Case {
        HealthState state;
        bool expected_valid;
    };
    const Case cases[] = {
        {HealthState::kInitializing, false},
        {HealthState::kOk, true},
        {HealthState::kDegraded, true},
        {HealthState::kLost, false},
    };

    Pose3D pose;
    pose.parent_frame = "map";
    pose.child_frame = "base_footprint";
    pose.transform = Eigen::Isometry3d::Identity();

    for (const auto& item : cases) {
        HealthStatus health;
        health.state = item.state;
        health.reason = foray::localization::to_string(item.state);

        const auto message =
            to_msg_state(health, pose, PoseSource::kSentryLidar, 1000000000LL, PoseUncertainty{});
        EXPECT_EQ(message.pose_valid, item.expected_valid) << "state=" << message.state;
    }
}

TEST(ConversionTest, PoseRoundTrip) {
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.linear() = Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    transform.translation() = Eigen::Vector3d(-1.5, 0.25, 0.0);

    const auto message = to_msg_pose(transform);
    const Eigen::Isometry3d restored = foray::localization_ros::to_eigen_pose(message);
    EXPECT_LT((restored.translation() - transform.translation()).norm(), 1e-9);
    EXPECT_LT((restored.linear() - transform.linear()).norm(), 1e-9);
}

TEST(ConversionTest, MapMetaAndObstacles) {
    MapManifest manifest;
    manifest.version = "2027_3v3";
    manifest.frame_id = "map";
    manifest.origin = {1.0, -2.0, 0.0};
    manifest.resolution = 0.05;
    manifest.width = 240;
    manifest.height = 160;
    manifest.pcd_sha256 = std::string(64, 'a');
    manifest.grid_sha256 = std::string(64, 'b');
    manifest.source = "sim";

    const auto meta = to_msg_map_meta(manifest, 1000LL);
    EXPECT_EQ(meta.version, "2027_3v3");
    EXPECT_EQ(meta.header.frame_id, "map");
    EXPECT_EQ(meta.width, 240U);
    EXPECT_DOUBLE_EQ(meta.origin.x, 1.0);

    ResidualCluster cluster;
    cluster.centroid = Eigen::Vector3d(1.0, 2.0, 0.3);
    cluster.min_corner = Eigen::Vector3f(0.5F, 1.5F, 0.0F);
    cluster.max_corner = Eigen::Vector3f(1.5F, 2.5F, 0.6F);
    cluster.point_count = 12;
    cluster.confidence = 0.9;
    cluster.mean_residual_m = 0.4;

    const auto obstacles = to_msg_obstacles({cluster}, "map", 2000LL);
    ASSERT_EQ(obstacles.obstacles.size(), 1U);
    EXPECT_EQ(obstacles.header.frame_id, "map");
    EXPECT_EQ(obstacles.obstacles[0].point_count, 12U);
    EXPECT_NEAR(obstacles.obstacles[0].size.x, 1.0, 1e-5);
}

TEST(ConversionTest, TimestampRoundTrip) {
    const auto stamp = to_msg_time(1234567890123LL);
    EXPECT_EQ(foray::localization_ros::to_ns(stamp), 1234567890123LL);
}

TEST(ConversionTest, CovarianceNeverClaimsZeroUncertaintyWithoutQuality) {
    // 审计 L7：父提交整块填 0，等于宣称「位姿绝对准确」。没有配准质量时（从未配准、
    // 外部位姿源、尚未热启动）必须如实写「不知道有多好」。
    const auto covariance = pose_covariance(PoseUncertainty{});
    for (std::size_t dof = 0; dof < kPoseDofCount; ++dof) {
        EXPECT_DOUBLE_EQ(covariance[diag_offset(dof)], 1000.0) << "dof=" << dof;
    }
    EXPECT_EQ(covariance[0 * kPoseDofCount + 1], 0.0);
    EXPECT_EQ(covariance[3 * kPoseDofCount + 5], 0.0);
}

TEST(ConversionTest, CovarianceFollowsResidualAndInlierRatio) {
    const auto covariance = pose_covariance(good_frame());
    // sigma = max(残差, 1 cm) / sqrt(内点率)；这一帧 σ = 0.05 / sqrt(0.9)。
    const double sigma = 0.05 / std::sqrt(0.9);
    for (std::size_t dof = 0; dof < kPoseDofCount; ++dof) {
        EXPECT_NEAR(covariance[diag_offset(dof)], sigma * sigma, 1e-12) << "dof=" << dof;
    }
    // 残差没有告诉任何方向间的相关性，非对角项一律为 0（不凭空创造信息）。
    for (std::size_t row = 0; row < kPoseDofCount; ++row) {
        for (std::size_t col = 0; col < kPoseDofCount; ++col) {
            if (row != col) {
                EXPECT_DOUBLE_EQ(covariance[row * kPoseDofCount + col], 0.0);
            }
        }
    }
}

TEST(ConversionTest, CovarianceGrowsWhenRegistrationDegrades) {
    PoseUncertainty poor = good_frame();
    poor.mean_residual_m = 0.29; // README 健康契约的 DEGRADED 残差上限
    poor.inlier_ratio = 0.36;    // 低于 ok_inlier_ratio 0.55

    const auto good = pose_covariance(good_frame());
    const auto bad = pose_covariance(poor);
    const double sigma = 0.29 / std::sqrt(0.36);
    EXPECT_NEAR(bad[diag_offset(0)], sigma * sigma, 1e-12);
    // 差帧的方差必须显著更大，但仍小于「无信息」的量级——它毕竟还是一帧有效配准。
    EXPECT_GT(bad[diag_offset(0)], 50.0 * good[diag_offset(0)]);
    EXPECT_LT(bad[diag_offset(0)], 1000.0);
}

TEST(ConversionTest, UnobservableDirectionVarianceIsInflatedByScaleSquared) {
    PoseUncertainty input = good_frame();
    input.observable[2] = false; // 竖直方向不可观（小视场下的典型退化方向）
    const auto covariance = pose_covariance(input);
    const double sigma = 0.05 / std::sqrt(0.9);
    EXPECT_NEAR(covariance[diag_offset(2)], sigma * sigma * 100.0,
                1e-9); // 10 倍标准差 → 100 倍方差
    EXPECT_NEAR(covariance[diag_offset(0)], sigma * sigma, 1e-12);
    EXPECT_NEAR(covariance[diag_offset(5)], sigma * sigma, 1e-12);
}

TEST(ConversionTest, CovarianceTreatsNonFiniteResidualAsWorstCase) {
    // 配准后端返回 NaN 残差时，不能把 NaN 漏进协方差（下游会拿它做加权）。
    PoseUncertainty input = good_frame();
    input.mean_residual_m = std::numeric_limits<double>::quiet_NaN();
    const auto covariance = pose_covariance(input);
    for (std::size_t dof = 0; dof < kPoseDofCount; ++dof) {
        EXPECT_TRUE(std::isfinite(covariance[diag_offset(dof)])) << "dof=" << dof;
        EXPECT_NEAR(covariance[diag_offset(dof)], 100.0, 1e-9) << "dof=" << dof; // 10 m 上限
    }
}

TEST(ConversionTest, StateMessageCarriesTheCovariance) {
    // 端到端：消息里的协方差必须就是模型算出来的那一份，不是 0。
    HealthStatus health;
    health.state = HealthState::kOk;
    Pose3D pose;
    pose.parent_frame = "map";
    pose.child_frame = "base_footprint";

    const auto message =
        to_msg_state(health, pose, PoseSource::kSentryLidar, 1000000000LL, good_frame());
    const double sigma = 0.05 / std::sqrt(0.9);
    EXPECT_NEAR(message.pose.covariance[0], sigma * sigma, 1e-12);
    EXPECT_NEAR(message.pose.covariance[diag_offset(5)], sigma * sigma, 1e-12);
}

} // namespace
