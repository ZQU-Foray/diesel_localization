// Copyright 2026 ZQU-Foray
//
// 融合层 EKF 的行为契约（decision.md D24）。重点守三件事：
// 关掉时必须与改造前逐字节等价、开的时必须真的压住抖动、不可观方向必须少听量测。

#include "foray/localization/pose_ekf.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <limits>

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

/// L13 用例一：非有限量测必须被丢掉，而不是把 NaN 写进状态。
TEST(PoseEkf, NonFiniteMeasurementIsDropped) {
    PoseEkf ekf(enabled_config());
    ekf.reset(make_pose(0.3, 0.0, 0.0));
    const Eigen::Matrix<double, kPoseDofCount, kPoseDofCount> before = ekf.covariance();

    Eigen::Isometry3d broken = make_pose(1.0, 1.0, 0.5);
    broken.translation().x() = std::numeric_limits<double>::quiet_NaN();
    ekf.update(broken);

    const Eigen::Isometry3d& estimate = ekf.estimate();
    std::printf("L13 回执(非有限量测): 估计 x=%.4f 有限=%d 协方差有限=%d 协方差未变=%d\n",
                estimate.translation().x(),
                estimate.matrix().allFinite() && std::isfinite(estimate.translation().x()) ? 1 : 0,
                ekf.covariance().allFinite() ? 1 : 0,
                (ekf.covariance() - before).cwiseAbs().maxCoeff() < 1e-12 ? 1 : 0);
    EXPECT_TRUE(estimate.matrix().allFinite());
    EXPECT_NEAR(estimate.translation().x(), 0.3, 1e-12);
    EXPECT_TRUE(ekf.covariance().allFinite());
    EXPECT_LT((ekf.covariance() - before).cwiseAbs().maxCoeff(), 1e-12);
}

/// L13 用例二：`dt_sec <= 0` 挡不住 NaN，预测必须自己判有限性。
TEST(PoseEkf, NonFinitePredictionIsDropped) {
    PoseEkf ekf(enabled_config());
    ekf.reset(Eigen::Isometry3d::Identity());
    const Eigen::Matrix<double, kPoseDofCount, kPoseDofCount> before = ekf.covariance();

    ekf.predict(std::numeric_limits<double>::quiet_NaN());

    std::printf("L13 回执(NaN dt): 协方差有限=%d 协方差未变=%d\n",
                ekf.covariance().allFinite() ? 1 : 0,
                (ekf.covariance() - before).cwiseAbs().maxCoeff() < 1e-12 ? 1 : 0);
    EXPECT_TRUE(ekf.covariance().allFinite());
    EXPECT_LT((ekf.covariance() - before).cwiseAbs().maxCoeff(), 1e-12);
}

/// L13 用例三：非有限初值不能进状态（重定位注入坏值同样要挡住）。
TEST(PoseEkf, NonFiniteResetIsIgnored) {
    PoseEkf ekf(enabled_config());
    ekf.reset(make_pose(0.25, 0.0, 0.0));

    Eigen::Isometry3d broken = make_pose(9.0, 9.0, 0.0);
    broken.translation().y() = std::numeric_limits<double>::infinity();
    ekf.reset(broken);

    std::printf("L13 回执(非有限 reset): 估计 x=%.4f y=%.4f\n", ekf.estimate().translation().x(),
                ekf.estimate().translation().y());
    EXPECT_NEAR(ekf.estimate().translation().x(), 0.25, 1e-12);
    EXPECT_NEAR(ekf.estimate().translation().y(), 0.0, 1e-12);
}

/// L13 用例四：多轮预测/更新后协方差保持对称、对角为正（Joseph + 对称化的回归守卫）。
TEST(PoseEkf, CovarianceStaysSymmetricAndPositive) {
    PoseEkf ekf(enabled_config());
    ekf.reset(Eigen::Isometry3d::Identity());
    std::array<bool, kPoseDofCount> observable{};
    observable.fill(true);
    observable[5] = false;

    for (int index = 0; index < 500; ++index) {
        ekf.predict(0.2);
        const double jitter = (index % 2 == 0) ? 0.02 : -0.02;
        ekf.update(make_pose(0.1 + jitter, 0.05, 0.01 * jitter), &observable);
    }
    const Eigen::Matrix<double, kPoseDofCount, kPoseDofCount> covariance = ekf.covariance();
    const double asymmetry = (covariance - covariance.transpose()).cwiseAbs().maxCoeff();
    double min_diagonal = covariance(0, 0);
    for (int i = 1; i < kPoseDofCount; ++i) {
        min_diagonal = std::min(min_diagonal, covariance(i, i));
    }
    std::printf("L13 回执(对称性): 最大不对称=%.3e 最小对角=%.3e\n", asymmetry, min_diagonal);
    EXPECT_LT(asymmetry, 1e-12);
    EXPECT_GT(min_diagonal, 0.0);
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
