// Copyright 2026 ZQU-Foray

#include "foray/localization_map/icp_aligner.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <string>

namespace {

using foray::localization_map::AlignOptions;
using foray::localization_map::IcpAligner;
using foray::localization_map::PointCloud;

PointCloud make_grid(int side, double spacing) {
    PointCloud cloud;
    for (int iy = 0; iy < side; ++iy) {
        for (int ix = 0; ix < side; ++ix) {
            cloud.points.emplace_back(static_cast<float>(ix * spacing),
                                      static_cast<float>(iy * spacing), 0.0F);
        }
    }
    return cloud;
}

/// 非周期点云：规则网格在 ICP 里有别名问题（会收敛到错位的局部最优），
/// 判定「收敛到哪」的用例必须用无周期性约束的点集。
PointCloud make_random_cloud(int count, double extent) {
    PointCloud cloud;
    cloud.points.reserve(static_cast<std::size_t>(count));
    unsigned seed = 20261031U;
    auto next = [&seed, extent]() {
        seed = seed * 1103515245U + 12345U;
        return (static_cast<double>((seed >> 8U) % 100000U) / 100000.0 - 0.5) * extent;
    };
    for (int index = 0; index < count; ++index) {
        cloud.points.emplace_back(static_cast<float>(next()), static_cast<float>(next()),
                                  static_cast<float>(next() * 0.2));
    }
    return cloud;
}

TEST(IcpAlignerTest, RecoversKnownTransform) {
    const PointCloud target = make_grid(20, 0.1);

    Eigen::Isometry3d truth = Eigen::Isometry3d::Identity();
    truth.linear() = Eigen::AngleAxisd(0.02, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    truth.translation() = Eigen::Vector3d(0.05, -0.03, 0.0);
    Eigen::Isometry3d inverse = truth.inverse();

    PointCloud source;
    for (const auto& point : target.points) {
        source.points.push_back((inverse * point.cast<double>()).cast<float>());
    }

    AlignOptions options;
    options.max_iterations = 40;
    options.max_correspondence_distance_m = 0.3;
    IcpAligner aligner(options);
    const auto result = aligner.align(source, target, Eigen::Isometry3d::Identity());

    ASSERT_TRUE(result.converged) << result.error;
    EXPECT_LT((result.transform.translation() - truth.translation()).norm(), 0.02);
    const double angle =
        Eigen::AngleAxisd(result.transform.linear() * truth.linear().transpose()).angle();
    EXPECT_LT(std::abs(angle), 0.02);
    EXPECT_GT(result.inlier_ratio, 0.95);
}

TEST(IcpAlignerTest, FailsOnDisjointClouds) {
    PointCloud source = make_grid(5, 0.1);
    PointCloud target = make_grid(5, 0.1);
    for (auto& point : target.points) {
        point.x() += 100.0F;
    }

    AlignOptions options;
    options.max_correspondence_distance_m = 0.2;
    IcpAligner aligner(options);
    const auto result = aligner.align(source, target, Eigen::Isometry3d::Identity());
    EXPECT_FALSE(result.converged);
    EXPECT_FALSE(result.error.empty());
}

TEST(IcpAlignerTest, RejectsEmptyInput) {
    const PointCloud empty;
    IcpAligner aligner;
    const auto result = aligner.align(empty, empty, Eigen::Isometry3d::Identity());
    EXPECT_FALSE(result.converged);
    EXPECT_EQ(result.iterations, 0);
}

/// L11 用例一：米制阈值不许兼职当姿态阈值。
///
/// 调用方把平移容忍放宽到 1 m（例如只在粗配准时才关心平移）时，姿态精度仍必须由弧度
/// 阈值单独约束。父提交把同一个字段同时当米和弧度用，于是 `transformation_epsilon_m = 1.0`
/// 等于同时允许「每轮转 1 rad（57°）」，第一次迭代就宣布收敛，交出一个还差约 0.18 rad 的位姿。
TEST(IcpAlignerTest, MeterThresholdDoesNotSetRotationTolerance) {
    const PointCloud target = make_random_cloud(600, 1.0);
    const double misalignment_rad = 0.2;
    Eigen::Isometry3d truth = Eigen::Isometry3d::Identity();
    truth.linear() =
        Eigen::AngleAxisd(misalignment_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    PointCloud source;
    for (const auto& point : target.points) {
        source.points.push_back((truth.inverse() * point.cast<double>()).cast<float>());
    }

    AlignOptions options;
    options.max_iterations = 100;
    options.max_correspondence_distance_m = 0.3;
    options.transformation_epsilon_m = 1.0;
    IcpAligner aligner(options);
    const auto result = aligner.align(source, target, Eigen::Isometry3d::Identity());

    const double residual_rad =
        Eigen::AngleAxisd(result.transform.linear() * truth.linear().transpose()).angle();
    std::printf("L11 回执(米制阈值): eps_m=1.0 converged=%d iterations=%d 残余角度=%.4f rad\n",
                result.converged ? 1 : 0, result.iterations, residual_rad);
    ASSERT_TRUE(result.converged) << result.error;
    EXPECT_LT(residual_rad, 0.02) << "平移阈值被当成了弧度阈值：姿态还差 " << residual_rad
                                  << " rad 就宣布收敛了";
}

/// L11 用例二：跑满预算没收敛时必须说清停在哪里，而不是只给一个空 error。
TEST(IcpAlignerTest, ExhaustedBudgetExplainsItself) {
    const PointCloud target = make_grid(20, 0.1);
    Eigen::Isometry3d truth = Eigen::Isometry3d::Identity();
    truth.linear() = Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    PointCloud source;
    for (const auto& point : target.points) {
        source.points.push_back((truth.inverse() * point.cast<double>()).cast<float>());
    }

    AlignOptions options;
    options.max_iterations = 1;
    options.max_correspondence_distance_m = 0.3;
    IcpAligner aligner(options);
    const auto result = aligner.align(source, target, Eigen::Isometry3d::Identity());

    std::printf("L11 回执(预算耗尽): converged=%d iterations=%d error='%s'\n",
                result.converged ? 1 : 0, result.iterations, result.error.c_str());
    ASSERT_FALSE(result.converged);
    EXPECT_NE(result.error.find("未收敛"), std::string::npos)
        << "跑满 max_iterations 却没有任何说明：调用方分不清是数据坏还是阈值严";
    EXPECT_NE(result.error.find("rad"), std::string::npos);
}

} // namespace
