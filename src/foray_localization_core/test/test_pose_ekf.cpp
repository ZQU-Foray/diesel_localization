// Copyright 2026 ZQU-Foray
//
// 融合层 EKF 的行为契约（decision.md D24）。重点守三件事：
// 关掉时必须与改造前逐字节等价、开的时必须真的压住抖动、不可观方向必须少听量测。

#include "foray/localization/pose_ekf.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>

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

/// L12 用例一：不可观方向的权重必须始终是「约 1 %」，不能随外推时间爬回去。
///
/// 噪声倍数只决定增益有多小（P_ii / (P_ii + R·scale²)），决定不了它的上限：
/// 中间没有配准、协方差按 0.05 m/s 一直长，分子追上分母后增益自己就大了。
TEST(PoseEkf, UnobservableDirectionWeightStaysCapped) {
    std::array<bool, kPoseDofCount> observable{};
    observable.fill(true);
    observable[0] = false;

    PoseEkf ekf(enabled_config());
    ekf.reset(Eigen::Isometry3d::Identity());
    // 100 s 只外推不更新：协方差长到约 0.25 m²。
    for (int index = 0; index < 500; ++index) {
        ekf.predict(0.2);
    }
    const double covariance_x = ekf.covariance()(0, 0);
    ekf.update(make_pose(1.0, 0.0, 0.0), &observable);
    const double moved = ekf.estimate().translation().x();
    std::printf("L12 回执(增益漂移): 外推 100s 后 cov[x]=%.4f 量测 1.0 m → 估计移动=%.4f "
                "m（隐含增益 %.3f）\n",
                covariance_x, moved, moved);
    EXPECT_LT(moved, 0.02) << "不可观方向的增益被协方差增长推高了：1 m 的量测让病态方向动了 "
                           << moved << " m";
}

/// L12 用例二：协方差不能无限增长——否则迟早溢出，且增益会贴着 1。
TEST(PoseEkf, CovarianceStopsGrowing) {
    PoseEkf ekf(enabled_config());
    ekf.reset(Eigen::Isometry3d::Identity());
    // 1e5 s（约 28 h）外推：无上限时平移方差会长到 250 m²。
    for (int index = 0; index < 100000; ++index) {
        ekf.predict(1.0);
    }
    const double translation = ekf.covariance()(0, 0);
    const double rotation = ekf.covariance()(3, 3);
    // 上限值写死在断言里而不是引用常量：本用例要能在父提交上编译并失败。
    std::printf("L12 回执(方差上限): 外推 1e5 s 后 cov[x]=%.4f cov[rx]=%.4f（上限 1.00 / 0.25）\n",
                translation, rotation);
    EXPECT_LE(translation, 1.0);
    EXPECT_LE(rotation, 0.25);
    EXPECT_TRUE(std::isfinite(translation));
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
