// Copyright 2026 ZQU-Foray

#include "diesel/localization_map/residual_clusterer.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>

namespace {

using diesel::localization_map::kUnmatchedResidualMultiple;
using diesel::localization_map::PointCloud;
using diesel::localization_map::ResidualClusterer;
using diesel::localization_map::ResidualClustererConfig;

PointCloud make_plane(int side, double spacing) {
    PointCloud cloud;
    for (int iy = 0; iy < side; ++iy) {
        for (int ix = 0; ix < side; ++ix) {
            cloud.points.emplace_back(static_cast<float>(ix * spacing),
                                      static_cast<float>(iy * spacing), 0.0F);
        }
    }
    return cloud;
}

TEST(ResidualClustererTest, FindsDynamicClusterAboveMap) {
    const PointCloud target = make_plane(21, 0.1);
    PointCloud source = target;
    for (int i = 0; i < 20; ++i) {
        source.points.emplace_back(0.5F, 0.5F, 0.5F + 0.01F * static_cast<float>(i));
    }

    const ResidualClusterer clusterer;
    const auto clusters = clusterer.cluster(source, target);
    ASSERT_EQ(clusters.size(), 1U);
    EXPECT_EQ(clusters[0].point_count, 20);
    EXPECT_NEAR(clusters[0].centroid.z(), 0.6, 0.05);
    EXPECT_GT(clusters[0].confidence, 0.5);
}

TEST(ResidualClustererTest, DropsSmallClusters) {
    const PointCloud target = make_plane(21, 0.1);
    PointCloud source = target;
    source.points.emplace_back(5.0F, 5.0F, 5.0F);
    source.points.emplace_back(5.0F, 5.0F, 5.05F);

    const ResidualClusterer clusterer;
    EXPECT_TRUE(clusterer.cluster(source, target).empty());
}

TEST(ResidualClustererTest, UnmatchedPointsReportTheDocumentedResidual) {
    // 审计 L17：`4.0 * residual_threshold_m` 曾是裸字面量。现在它是公开常量
    // `kUnmatchedResidualMultiple`，这条用例把「隐式约定」钉成显式契约：
    // 「地图上一个邻格都没有」的点报 4 倍阈值，而「找到了但超出阈值」的点报真实距离。
    const PointCloud target = make_plane(21, 0.1);
    PointCloud source;
    // 远处一簇：27 个邻格里没有任何地图点 → 走 `!found` 分支。
    for (int i = 0; i < 10; ++i) {
        source.points.emplace_back(50.0F, 50.0F, 50.0F + 0.01F * static_cast<float>(i));
    }
    // 地图上方一簇：最近邻确实存在、距离 0.5 m > 阈值 0.3 m → 报真实距离。
    for (int i = 0; i < 10; ++i) {
        source.points.emplace_back(0.5F, 0.5F, 0.5F + 0.01F * static_cast<float>(i));
    }

    const ResidualClustererConfig config;
    const ResidualClusterer clusterer(config);
    const auto clusters = clusterer.cluster(source, target);
    ASSERT_EQ(clusters.size(), 2U);

    // 簇按点数降序，两个簇一样大 → 按输出顺序逐条比对残差量级。
    double unmatched_residual = 0.0;
    double matched_residual = 0.0;
    for (const auto& cluster : clusters) {
        if (cluster.centroid.z() > 10.0) {
            unmatched_residual = cluster.mean_residual_m;
        } else {
            matched_residual = cluster.mean_residual_m;
        }
    }
    printf("L17 回执: 无邻格簇残差=%.3f m（阈值 %.2f × %.1f）有邻格簇残差=%.3f m\n",
           unmatched_residual, config.residual_threshold_m, kUnmatchedResidualMultiple,
           matched_residual);

    EXPECT_DOUBLE_EQ(unmatched_residual, kUnmatchedResidualMultiple * config.residual_threshold_m);
    EXPECT_GT(unmatched_residual, config.residual_threshold_m)
        << "占位值必须恒大于阈值，否则会被当成内点";
    // 这簇点在 z=0.50…0.59 之间，最近邻都是正下方的地图点，所以残差落在 (0.5, 0.6)：
    // 关键是与「占位值」区分得开，而不是钉在某一位小数上。
    EXPECT_GT(matched_residual, 0.5);
    EXPECT_LT(matched_residual, 0.6) << "有邻格的残差点必须报真实距离，不能被占位值覆盖";
    EXPECT_LT(matched_residual, unmatched_residual);
}

TEST(ResidualClustererTest, SplitsSeparatedClusters) {
    const PointCloud target = make_plane(21, 0.1);
    PointCloud source = target;
    for (int i = 0; i < 10; ++i) {
        source.points.emplace_back(0.5F, 0.5F, 0.5F + 0.01F * static_cast<float>(i));
        source.points.emplace_back(3.0F, 3.0F, 0.5F + 0.01F * static_cast<float>(i));
    }

    const ResidualClusterer clusterer;
    const auto clusters = clusterer.cluster(source, target);
    ASSERT_EQ(clusters.size(), 2U);
}

TEST(ResidualClustererTest, RespectsMaxClusterSize) {
    const PointCloud target = make_plane(21, 0.1);
    PointCloud source = target;
    for (int i = 0; i < 60; ++i) {
        source.points.emplace_back(0.5F, 0.5F, 0.5F + 0.005F * static_cast<float>(i));
    }

    ResidualClustererConfig config;
    config.max_cluster_size = 50;
    const ResidualClusterer clusterer(config);
    EXPECT_TRUE(clusterer.cluster(source, target).empty());
}

} // namespace
