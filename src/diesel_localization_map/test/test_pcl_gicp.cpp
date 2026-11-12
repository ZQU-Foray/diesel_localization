// Copyright 2026 ZQU-Foray
//
// PCL GICP 后端的契约测试。仅在 DIESEL_WITH_PCL 时构建（CMakeLists 里条件注册）。
//
// 注意点云形状：这里用三维格点，不用 test_icp 里的平面网格。GICP 按邻域协方差
// 加权，纯共面点云在法向上协方差退化，本身就是病态输入，用它测 GICP 只会测出噪声。

#include "diesel/localization_map/pcl_gicp_aligner.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdio>

namespace {

using diesel::localization_map::AlignOptions;
using diesel::localization_map::PclGicpAligner;
using diesel::localization_map::PointCloud;

PointCloud make_cube(int side, double spacing) {
    PointCloud cloud;
    cloud.points.reserve(static_cast<std::size_t>(side * side * side));
    for (int iz = 0; iz < side; ++iz) {
        for (int iy = 0; iy < side; ++iy) {
            for (int ix = 0; ix < side; ++ix) {
                cloud.points.emplace_back(static_cast<float>(ix * spacing),
                                          static_cast<float>(iy * spacing),
                                          static_cast<float>(iz * spacing));
            }
        }
    }
    return cloud;
}

AlignOptions make_options() {
    AlignOptions options;
    options.max_iterations = 50;
    options.max_correspondence_distance_m = 0.5;
    return options;
}

TEST(PclGicpAlignerTest, RecoversKnownTransform) {
    // spacing 0.3 让点间距远大于初始错位（< 0.06 m），最近邻对应关系无歧义。
    const PointCloud target = make_cube(5, 0.3);

    Eigen::Isometry3d truth = Eigen::Isometry3d::Identity();
    truth.linear() = Eigen::AngleAxisd(0.02, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    truth.translation() = Eigen::Vector3d(0.05, -0.03, 0.01);
    const Eigen::Isometry3d inverse = truth.inverse();

    PointCloud source;
    source.points.reserve(target.points.size());
    for (const auto& point : target.points) {
        source.points.push_back((inverse * point.cast<double>()).cast<float>());
    }

    PclGicpAligner aligner(make_options());
    const auto result = aligner.align(source, target, Eigen::Isometry3d::Identity());

    ASSERT_TRUE(result.converged) << result.error;
    EXPECT_LT((result.transform.translation() - truth.translation()).norm(), 0.02);
    const double angle =
        Eigen::AngleAxisd(result.transform.linear() * truth.linear().transpose()).angle();
    EXPECT_LT(std::abs(angle), 0.02);
    EXPECT_GT(result.inlier_ratio, 0.9);
    EXPECT_GT(result.iterations, 0);
}

/// 非周期点云：格点在 3 m 平移下与自身重合（0.3 m 间距正好 10 格），
/// 用它测「靶标换了」会得到「旧靶标也匹配」的假阴性。
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
                                  static_cast<float>(next()));
    }
    return cloud;
}

/// L14 用例一：target 缓存不能只看地址。
///
/// 同一个 `PointCloud` 对象被整体换内容（`map = load_pcd(...)` 不会换地址）时，
/// 缓存命中会让 GICP 对着旧靶标优化、质量评估却用新靶标——读数就是
/// 「converged=1 但 inlier=0」，位姿却是照旧地图算出来的。
TEST(PclGicpAlignerTest, TargetCacheInvalidatesWhenContentChanges) {
    PointCloud target = make_random_cloud(400, 1.0);
    Eigen::Isometry3d truth = Eigen::Isometry3d::Identity();
    truth.translation() = Eigen::Vector3d(0.05, -0.03, 0.01);

    auto make_source = [](const PointCloud& cloud, const Eigen::Isometry3d& offset) {
        PointCloud source;
        source.points.reserve(cloud.points.size());
        const Eigen::Isometry3d inverse = offset.inverse();
        for (const auto& point : cloud.points) {
            source.points.push_back((inverse * point.cast<double>()).cast<float>());
        }
        return source;
    };

    PclGicpAligner aligner(make_options());
    const auto first =
        aligner.align(make_source(target, truth), target, Eigen::Isometry3d::Identity());
    ASSERT_TRUE(first.converged) << first.error;
    ASSERT_GT(first.inlier_ratio, 0.9);

    // 同一个对象、同一个地址，内容整体平移 3 m —— 相当于地图被重新加载。
    constexpr double kShiftM = 3.0;
    PointCloud replaced;
    replaced.points.reserve(target.points.size());
    for (const auto& point : target.points) {
        replaced.points.push_back(point + Eigen::Vector3f(static_cast<float>(kShiftM), 0.0F, 0.0F));
    }
    target = replaced;

    // 初值就是真值：这一帧唯一可能失败的原因只能是「靶标没换」。
    const auto second = aligner.align(make_source(target, truth), target, truth);
    std::printf(
        "L14 回执(target 内容变更): converged=%d iterations=%d inlier=%.3f x=%.3f error='%s'\n",
        second.converged ? 1 : 0, second.iterations, second.inlier_ratio,
        second.transform.translation().x(), second.error.c_str());
    ASSERT_TRUE(second.converged) << second.error;
    EXPECT_GT(second.inlier_ratio, 0.9) << "配准对着旧靶标做了：质量评估却说没有内点";
    // source 与 target 用的是同一份新内容，所以解就是它们之间的真值偏移（不是 3 m 本身）。
    EXPECT_LT((second.transform.translation() - truth.translation()).norm(), 0.02);
}

/// L14 用例二：`set_options()` 之后必须在下一帧生效，而不是被「只配置一次」吞掉。
TEST(PclGicpAlignerTest, OptionsTakeEffectAfterSetOptions) {
    const PointCloud target = make_cube(5, 0.3);
    Eigen::Isometry3d truth = Eigen::Isometry3d::Identity();
    truth.linear() = Eigen::AngleAxisd(0.02, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    PointCloud source;
    for (const auto& point : target.points) {
        source.points.push_back((truth.inverse() * point.cast<double>()).cast<float>());
    }

    AlignOptions stingy = make_options();
    stingy.max_iterations = 1;
    PclGicpAligner aligner(stingy);
    const auto limited = aligner.align(source, target, Eigen::Isometry3d::Identity());
    ASSERT_LE(limited.iterations, 1);

    aligner.set_options(make_options());
    const auto restored = aligner.align(source, target, Eigen::Isometry3d::Identity());
    std::printf("L14 回执(参数下发): 改前 iterations=%d → set_options 后 iterations=%d\n",
                limited.iterations, restored.iterations);
    EXPECT_GT(restored.iterations, limited.iterations)
        << "set_options() 没有在下一帧生效：迭代预算仍是旧值";
    ASSERT_TRUE(restored.converged) << restored.error;
    EXPECT_GT(restored.inlier_ratio, 0.9);
}

TEST(PclGicpAlignerTest, RejectsEmptyInput) {
    const PointCloud empty;
    PclGicpAligner aligner;
    const auto result = aligner.align(empty, empty, Eigen::Isometry3d::Identity());
    EXPECT_FALSE(result.converged);
    EXPECT_EQ(result.iterations, 0);
    EXPECT_FALSE(result.error.empty());
    EXPECT_DOUBLE_EQ(result.inlier_ratio, 0.0);
}

TEST(PclGicpAlignerTest, FailsOnDisjointClouds) {
    const PointCloud source = make_cube(4, 0.2);
    PointCloud target = make_cube(4, 0.2);
    for (auto& point : target.points) {
        point.x() += 100.0F;
    }

    AlignOptions options;
    options.max_correspondence_distance_m = 0.2;
    PclGicpAligner aligner(options);
    const auto result = aligner.align(source, target, Eigen::Isometry3d::Identity());

    EXPECT_DOUBLE_EQ(result.inlier_ratio, 0.0);
    EXPECT_FALSE(result.error.empty());
}

TEST(PclGicpAlignerTest, MetricsAreConsistentWithIcpSemantics) {
    // 守住共享评估口径的语义契约（decision.md D21）：fitness / inlier_ratio 都是
    // 0..1 且越大越好。PCL 自带的 getFitnessScore() 是均方误差（越小越好），若有人
    // 图省事直接把它填进 AlignResult，这两条断言会立刻失败。
    const PointCloud target = make_cube(5, 0.3);
    Eigen::Isometry3d truth = Eigen::Isometry3d::Identity();
    truth.linear() = Eigen::AngleAxisd(-0.03, Eigen::Vector3d::UnitY()).toRotationMatrix();
    truth.translation() = Eigen::Vector3d(-0.04, 0.02, 0.03);
    const Eigen::Isometry3d inverse = truth.inverse();

    PointCloud source;
    source.points.reserve(target.points.size());
    for (const auto& point : target.points) {
        source.points.push_back((inverse * point.cast<double>()).cast<float>());
    }

    PclGicpAligner aligner(make_options());
    const auto result = aligner.align(source, target, Eigen::Isometry3d::Identity());

    EXPECT_GT(result.inlier_ratio, 0.0);
    EXPECT_LE(result.inlier_ratio, 1.0);
    EXPECT_GT(result.fitness, 0.0);
    EXPECT_LE(result.fitness, 1.0);
    EXPECT_GE(result.mean_residual_m, 0.0);
    EXPECT_LE(result.mean_residual_m, aligner.options().max_correspondence_distance_m);
}

} // namespace
