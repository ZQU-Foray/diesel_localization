// Copyright 2026 ZQU-Foray
//
// 融合层 EKF 的行为契约（decision.md D24）。重点守三件事：
// 关掉时必须与改造前逐字节等价、开的时必须真的压住抖动、不可观方向必须少听量测。

#include "foray/localization/pose_ekf.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace foray::localization {
namespace {

Eigen::Isometry3d make_pose(double x, double y, double yaw) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(x, y, 0.0);
    pose.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return pose;
}

PoseEkfConfig enabled_config() {
    PoseEkfConfig config;
    config.enabled = true;
    return config;
}

TEST(PoseEkf, DisabledPassesMeasurementThrough) {
    PoseEkf ekf; // 默认 enabled = false
    ASSERT_FALSE(ekf.enabled());
    const Eigen::Isometry3d measurement = make_pose(1.0, 2.0, 0.3);
    const auto before = ekf.covariance();
    const Eigen::Isometry3d out = ekf.update(measurement);
    // 直通路径不能有任何平滑：改造前的节点就是直接用配准结果。
    EXPECT_TRUE(out.matrix().isApprox(measurement.matrix(), 0.0));
    EXPECT_TRUE(ekf.covariance().isApprox(before, 0.0));
}

TEST(PoseEkf, ResetInstallsEstimateAndCovariance) {
    PoseEkf ekf(enabled_config());
    const Eigen::Isometry3d initial = make_pose(3.0, -1.0, 1.2);
    ekf.reset(initial);
    EXPECT_TRUE(ekf.estimate().matrix().isApprox(initial.matrix(), 1e-12));
    EXPECT_GT(ekf.covariance().trace(), 0.0);
    EXPECT_LT(ekf.covariance().trace(), 1.0); // 起步不确定度应在量测噪声量级，不是 1 m 量级
}

TEST(PoseEkf, SmoothsAlternatingMeasurementNoise) {
    PoseEkf ekf(enabled_config());
    const Eigen::Isometry3d truth = make_pose(0.0, 0.0, 0.0);
    ekf.reset(truth);
    double raw_sum = 0.0;
    double filtered_sum = 0.0;
    constexpr int kSamples = 40;
    for (int i = 0; i < kSamples; ++i) {
        const double jitter = (i % 2 == 0) ? 0.1 : -0.1;
        const Eigen::Isometry3d measurement = make_pose(jitter, 0.0, 0.0);
        ekf.predict(0.2);
        const Eigen::Isometry3d out = ekf.update(measurement);
        raw_sum += std::abs(jitter);
        filtered_sum += std::abs(out.translation().x());
    }
    // 交替噪声被压住一半以上，才算真的起了滤波作用。
    EXPECT_LT(filtered_sum, 0.5 * raw_sum);
}

TEST(PoseEkf, UnobservableDirectionTrustedLess) {
    std::array<bool, kPoseDofCount> observable{};
    observable.fill(true);

    PoseEkf trusted(enabled_config());
    trusted.reset(Eigen::Isometry3d::Identity());
    const Eigen::Isometry3d step = make_pose(0.5, 0.0, 0.0);
    trusted.update(step, &observable);

    // x 方向标成不可观：同一帧量测应该被大幅打折，位置几乎不动。
    observable[0] = false;
    PoseEkf untrusted(enabled_config());
    untrusted.reset(Eigen::Isometry3d::Identity());
    untrusted.update(step, &observable);

    EXPECT_GT(trusted.estimate().translation().x(), 0.2);
    EXPECT_LT(untrusted.estimate().translation().x(), 0.1);
    EXPECT_LT(untrusted.estimate().translation().x(), trusted.estimate().translation().x());
}

TEST(PoseEkf, RotationResidualTakesShortWayAround) {
    PoseEkf ekf(enabled_config());
    // 起点偏航 170°，量测 -170°：真实残差是 +20°（跨 ±180° 的近路），不是 -340°。
    ekf.reset(make_pose(0.0, 0.0, 3.0 * M_PI / 4.0));
    const Eigen::Isometry3d measurement = make_pose(0.0, 0.0, -3.0 * M_PI / 4.0);
    const Eigen::Isometry3d out = ekf.update(measurement);
    const double yaw = std::atan2(out.linear()(1, 0), out.linear()(0, 0));
    // 估计应当朝量测方向走小半步，而不是反着绕 340°。
    EXPECT_GT(std::abs(yaw), 3.0 * M_PI / 4.0);
    EXPECT_LE(std::abs(yaw), M_PI);
}

} // namespace
} // namespace foray::localization
