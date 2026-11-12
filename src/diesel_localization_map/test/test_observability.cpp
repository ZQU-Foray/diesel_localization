// Copyright 2026 ZQU-Foray
//
// 可观测性分析的判据测试：点面残差信息矩阵必须把「哪个自由度没被约束」如实分开。
// 判据用的是投影语义（给一个 6 维修正量，看它是否被清除），而不是去猜特征向量
// 恰好落在哪根坐标轴上——特征方向本来就是正交基，不保证与 x/y/z 对齐。

#include "diesel/localization_map/observability.hpp"

#include <gtest/gtest.h>

namespace diesel::localization_map {
namespace {

/// 归一化特征值阈值：特征值掉到最大值的 5% 以下即视为该方向不可观。
/// 与节点侧默认参数同量级；测试固定住它，避免判据随参数漂移。
constexpr double kThreshold = 0.05;

/// 盖一张 n×n 的平面点阵：从 `origin` 起，沿 `u`、`v` 两个方向铺开。
PointCloud make_plane(const Eigen::Vector3f& origin, const Eigen::Vector3f& u,
                      const Eigen::Vector3f& v, int n, float step) {
    PointCloud cloud;
    cloud.points.reserve(static_cast<std::size_t>(n) * static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            cloud.points.push_back(origin + u * (step * static_cast<float>(i)) +
                                   v * (step * static_cast<float>(j)));
        }
    }
    return cloud;
}

void merge(PointCloud& destination, const PointCloud& source) {
    destination.points.insert(destination.points.end(), source.points.begin(), source.points.end());
}

/// 单面墙：x = 0 平面，张在 y-z 上。法向只有 ±x。
PointCloud make_wall() {
    return make_plane(Eigen::Vector3f(0.0F, -2.0F, -2.0F), Eigen::Vector3f(0.0F, 1.0F, 0.0F),
                      Eigen::Vector3f(0.0F, 0.0F, 1.0F), 41, 0.1F);
}

/// 只有地面：z = 0 平面。
PointCloud make_ground() {
    return make_plane(Eigen::Vector3f(-3.0F, -3.0F, 0.0F), Eigen::Vector3f(1.0F, 0.0F, 0.0F),
                      Eigen::Vector3f(0.0F, 1.0F, 0.0F), 61, 0.1F);
}

/// 六个面围成的盒子：每个平移方向与每个旋转方向都被对面约束住。
PointCloud make_box() {
    PointCloud box;
    const float extent = 2.0F;
    const float step = 0.2F;
    const int count = 21;
    const Eigen::Vector3f x(1.0F, 0.0F, 0.0F);
    const Eigen::Vector3f y(0.0F, 1.0F, 0.0F);
    const Eigen::Vector3f z(0.0F, 0.0F, 1.0F);
    merge(box, make_plane(Eigen::Vector3f(-extent, -extent, -extent), x, y, count, step));
    merge(box, make_plane(Eigen::Vector3f(-extent, -extent, extent), x, y, count, step));
    merge(box, make_plane(Eigen::Vector3f(-extent, -extent, -extent), x, z, count, step));
    merge(box, make_plane(Eigen::Vector3f(-extent, extent, -extent), x, z, count, step));
    merge(box, make_plane(Eigen::Vector3f(-extent, -extent, -extent), y, z, count, step));
    merge(box, make_plane(Eigen::Vector3f(extent, -extent, -extent), y, z, count, step));
    return box;
}

Eigen::Matrix<double, kDofCount, 1> translation_delta(double x, double y, double z) {
    Eigen::Matrix<double, kDofCount, 1> delta = Eigen::Matrix<double, kDofCount, 1>::Zero();
    delta(0) = x;
    delta(1) = y;
    delta(2) = z;
    return delta;
}

} // namespace

TEST(Observability, SurroundingGeometryConstrainsAllDirections) {
    const PointCloud box = make_box();
    const NormalCache normals(box, 0.4, 6);
    const PointCloudIndex index(box, 0.5);

    const ObservabilityReport report =
        analyze_observability(box, Eigen::Isometry3d::Identity(), index, normals, 0.5, kThreshold);

    // 没有任何一对点面约束时 inlier_count 会是 0，这里先确认真的用上了点云。
    EXPECT_GT(report.inlier_count, 1000U);
    EXPECT_EQ(report.unobservable_count, 0);
    EXPECT_FALSE(report.degenerate());
    // 全向包围下条件数应该很小（各方向都被约束得差不多）。
    EXPECT_LT(report.condition_number, 100.0);
}

TEST(Observability, SingleWallRejectsTangentialTranslation) {
    const PointCloud wall = make_wall();
    const NormalCache normals(wall, 0.3, 6);
    const PointCloudIndex index(wall, 0.5);
    ASSERT_GT(normals.coverage(), 0.9);

    const ObservabilityReport report =
        analyze_observability(wall, Eigen::Isometry3d::Identity(), index, normals, 0.5, kThreshold);

    // 单面墙只约束法向：沿墙切向的平移必须被清除，否则走廊场景会把整帧配准当可信。
    const auto tangential = project_delta_to_observable(translation_delta(0.0, 0.5, 0.0), report);
    EXPECT_LT(tangential.norm(), 0.01);
    const auto vertical = project_delta_to_observable(translation_delta(0.0, 0.0, 0.5), report);
    EXPECT_LT(vertical.norm(), 0.01);

    // 法向平移必须保留——这是这面墙唯一能提供的约束。
    const auto normal_direction =
        project_delta_to_observable(translation_delta(0.5, 0.0, 0.0), report);
    EXPECT_NEAR(normal_direction(0), 0.5, 0.05);

    EXPECT_GT(report.unobservable_count, 0);
    EXPECT_TRUE(report.degenerate());
}

TEST(Observability, GroundOnlyRejectsPlanarTranslation) {
    const PointCloud ground = make_ground();
    const NormalCache normals(ground, 0.3, 6);
    const PointCloudIndex index(ground, 0.5);
    ASSERT_GT(normals.coverage(), 0.9);

    const ObservabilityReport report = analyze_observability(ground, Eigen::Isometry3d::Identity(),
                                                             index, normals, 0.5, kThreshold);

    // 只有地面时，平面内的两个平移与绕法向的偏航都拿不到约束。
    const auto planar = project_delta_to_observable(translation_delta(0.5, 0.5, 0.0), report);
    EXPECT_LT(planar.norm(), 0.01);

    // 高度方向必须保留。
    const auto height = project_delta_to_observable(translation_delta(0.0, 0.0, 0.5), report);
    EXPECT_NEAR(height(2), 0.5, 0.05);

    EXPECT_GT(report.unobservable_count, 0);
}

TEST(Observability, RotationLeverFollowsTheCorrectedFrame) {
    // 审计 L15：EKF 修正的量是 `map → odom`，它的旋转自由度是「绕里程计原点转」，
    // 而配准给的位姿是 `map → base`。两者相差 |odom → base|——机器人沿场地跑出 40 m
    // 后，同一个旋转在残差上等价于 40 m 的平移。父提交把力臂固定成本体位置，于是
    // 40 m 的杠杆把旋转信息放大到 10^6 量级，归一化后连「墙的法向平移」都掉到阈值下，
    // 掩码判它不可观，下游就把这个方向的修正清零——明明是最可信的方向。
    PointCloud scene = make_wall();
    merge(scene, make_ground());
    const NormalCache normals(scene, 0.3, 6);
    const PointCloudIndex index(scene, 0.5);

    Eigen::Isometry3d map_to_base = Eigen::Isometry3d::Identity();
    map_to_base.translation().x() = 40.0;
    PointCloud source;
    source.points.reserve(scene.points.size());
    for (const auto& point : scene.points) {
        source.points.push_back((map_to_base.inverse() * point.cast<double>()).cast<float>());
    }

    // 里程计原点落在地图原点上：力臂相差 40 m。
    const Eigen::Vector3d odom_origin = Eigen::Vector3d::Zero();
    const ObservabilityReport corrected =
        analyze_observability(source, map_to_base, index, normals, 0.5, kThreshold, &odom_origin);
    const ObservabilityReport body_relative =
        analyze_observability(source, map_to_base, index, normals, 0.5, kThreshold);

    int flipped = 0;
    for (std::size_t dof = 0; dof < kDofCount; ++dof) {
        if (corrected.observable[dof] != body_relative.observable[dof]) {
            ++flipped;
        }
    }
    printf("L15 回执(力臂语义): odom-力臂 unobs=%d cond=%.3g；body-力臂 unobs=%d cond=%.3g；"
           "判断翻转 %d/6\n",
           corrected.unobservable_count, corrected.condition_number,
           body_relative.unobservable_count, body_relative.condition_number, flipped);

    // 40 m 杠杆把旋转信息放大到 10^6 量级，归一化后 4 个方向跌到阈值以下。
    EXPECT_EQ(corrected.unobservable_count, 1);
    EXPECT_EQ(body_relative.unobservable_count, 5);
    EXPECT_EQ(flipped, 4);
    EXPECT_LT(corrected.condition_number, 1e3) << "绕里程计原点算力臂时条件数应与场景本身相当";
    EXPECT_GT(body_relative.condition_number, 1e6) << "绕本体算力臂时 40 m 杠杆主导了条件数";
}

TEST(Observability, RotationLeverDefaultsToTheBodyPose) {
    // 不传 lever_origin 时行为必须与父提交一致：力臂取 `transform.translation()`。
    // 里程计原点与本体重合（启动那一刻就是如此）时两种算法逐位相同，这条守住默认值。
    const PointCloud scene = make_box();
    const NormalCache normals(scene, 0.4, 6);
    const PointCloudIndex index(scene, 0.5);

    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation() = Eigen::Vector3d(1.0, -2.0, 0.5);
    const Eigen::Vector3d body_origin = transform.translation();

    const ObservabilityReport defaulted =
        analyze_observability(scene, transform, index, normals, 0.5, kThreshold);
    const ObservabilityReport explicit_body =
        analyze_observability(scene, transform, index, normals, 0.5, kThreshold, &body_origin);

    EXPECT_EQ(defaulted.information, explicit_body.information);
    EXPECT_EQ(defaulted.observable, explicit_body.observable);
}

TEST(Observability, DisjointCloudsReportNothingObservable) {
    const PointCloud wall = make_wall();
    const NormalCache normals(wall, 0.3, 6);
    const PointCloudIndex index(wall, 0.5);

    // 把源点云整体挪到 100 m 之外：一对有效对应都不该有。
    Eigen::Isometry3d far_away = Eigen::Isometry3d::Identity();
    far_away.translation().x() = 100.0;
    const ObservabilityReport report =
        analyze_observability(wall, far_away, index, normals, 0.5, kThreshold);

    EXPECT_EQ(report.inlier_count, 0U);
    EXPECT_EQ(report.unobservable_count, static_cast<int>(kDofCount));
    EXPECT_TRUE(report.degenerate());
    // 没有任何约束时，修正量一项都不许通过。
    const auto projected = project_delta_to_observable(translation_delta(1.0, 1.0, 1.0), report);
    EXPECT_LT(projected.norm(), 1e-12);
}

} // namespace diesel::localization_map
