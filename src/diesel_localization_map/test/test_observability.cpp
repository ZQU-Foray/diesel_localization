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

/// 走廊：两面 y 向墙（y=0 / y=3，x∈[0,4]、z∈[0,4]）加地面。
/// 沿走廊方向 x 的平移没有任何点面约束——这正是复核 P0-0 的现场几何。
PointCloud make_corridor() {
    PointCloud scene;
    const Eigen::Vector3f along_x(1.0F, 0.0F, 0.0F);
    const Eigen::Vector3f along_z(0.0F, 0.0F, 1.0F);
    merge(scene, make_plane(Eigen::Vector3f(0.0F, 0.0F, 0.0F), along_x, along_z, 41, 0.1F));
    merge(scene, make_plane(Eigen::Vector3f(0.0F, 3.0F, 0.0F), along_x, along_z, 41, 0.1F));
    merge(scene, make_plane(Eigen::Vector3f(-1.0F, -1.0F, 0.0F), along_x,
                            Eigen::Vector3f(0.0F, 1.0F, 0.0F), 61, 0.1F));
    return scene;
}

/// 把地图点云搬到 `pose` 的本体系里，当作这一帧的观测（配准完美时逐点重合）。
PointCloud observed_from(const PointCloud& scene, const Eigen::Isometry3d& pose) {
    const Eigen::Isometry3d inverse = pose.inverse();
    PointCloud source;
    source.points.reserve(scene.points.size());
    for (const auto& point : scene.points) {
        source.points.push_back((inverse * point.cast<double>()).cast<float>());
    }
    return source;
}

/// 左乘增量 → 6 维量，口径与节点侧一致（平移在前，angle-axis 在后）。
Eigen::Matrix<double, kDofCount, 1> twist_of(const Eigen::Isometry3d& delta) {
    Eigen::Matrix<double, kDofCount, 1> twist;
    twist.head<3>() = delta.translation();
    const Eigen::AngleAxisd rotation(delta.linear());
    twist.tail<3>() = rotation.angle() * rotation.axis();
    return twist;
}

/// 父提交（fix/projection-scope-1117 之前）的投影实现，逐行照抄节点里被替换掉的那段。
/// 保留它是为了让下面两条判据**自证能拦下旧实现**：同一个场景、同一串输入，
/// 旧公式必须失败、新函数必须通过。两处错都在这一段里：
///   1. 被投影的是 `registered * previous⁻¹`（含整段里程计传播），不是相对 guess 的修正量；
///   2. 平移分量按「绕世界原点转」送出，却用「绕 lever 转」的雅可比去投影。
Eigen::Isometry3d legacy_projection(const Eigen::Isometry3d& previous,
                                    const Eigen::Isometry3d& registered,
                                    const ObservabilityReport& report) {
    const Eigen::Isometry3d delta = registered * previous.inverse();
    const auto projected = project_delta_to_observable(twist_of(delta), report);
    Eigen::Isometry3d correction = Eigen::Isometry3d::Identity();
    correction.translation() = projected.head<3>();
    const Eigen::Vector3d omega = projected.tail<3>();
    const double angle = omega.norm();
    if (angle > 1e-12) {
        correction.linear() = Eigen::AngleAxisd(angle, omega / angle).toRotationMatrix();
    }
    return correction * previous;
}

/// 一个左乘增量在「不可观方向」上还剩多少分量（按信息矩阵的 lever 口径衡量）。
/// 投影正确时它必须为 0：投影后的量再投一次不该再变。
double unobservable_residue(const Eigen::Isometry3d& delta, const ObservabilityReport& report,
                            const Eigen::Vector3d& lever) {
    Eigen::Matrix<double, kDofCount, 1> twist = twist_of(delta);
    twist.head<3>() += twist.tail<3>().cross(lever);
    return (twist - project_delta_to_observable(twist, report)).norm();
}

/// 沿走廊直行 `frames` 帧、每帧 `step` 米，配准每帧都完美（结果 = 真值）。
/// 返回走完后的 `map → base`；`last_report` 带回最后一帧的可观性判据。
/// 里程计真值恒为单位阵（起点处 odom 与 map 重合），所以力臂从 0 起、纯平移无旋转——
/// P0-1 在这个场景里静默，失败只能归因于投影范围（P0-0）。
Eigen::Isometry3d run_corridor_frames(const PointCloud& scene, const PointCloudIndex& index,
                                      const NormalCache& normals, int frames, double step,
                                      bool legacy, ObservabilityReport& last_report) {
    Eigen::Isometry3d map_to_odom = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d odom_to_base = Eigen::Isometry3d::Identity();
    odom_to_base.translation() = Eigen::Vector3d(1.0, 1.5, 0.2);
    Eigen::Isometry3d map_to_base = map_to_odom * odom_to_base;
    for (int frame = 0; frame < frames; ++frame) {
        odom_to_base.translation().x() += step;
        const Eigen::Isometry3d truth = odom_to_base;
        const Eigen::Isometry3d guess = map_to_odom * odom_to_base;
        const Eigen::Vector3d lever = map_to_odom.translation();
        const PointCloud source = observed_from(scene, truth);
        last_report = analyze_observability(source, truth, index, normals, 0.5, kThreshold, &lever);
        map_to_base = legacy ? legacy_projection(map_to_base, truth, last_report)
                             : project_registration_correction(guess, truth, last_report, lever);
        map_to_odom = map_to_base * odom_to_base.inverse();
    }
    return map_to_base;
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

TEST(Observability, ProjectionKeepsOdometryAdvanceAlongUnobservableDirection) {
    // 复核 P0-0：投影的对象必须是「配准相对初值 guess 的修正量」。guess 里已经含两次配准
    // 之间的里程计外推，而父提交投影的是 `registered * previous⁻¹`——previous 是上一轮的
    // 位姿，两者之差含整段里程计传播 D = O_prev⁻¹·O_cur。沿走廊方向 D 不可观，于是外推被
    // 一并删掉，位姿冻结在原地，与 README「不可观方向由里程计外推承担」的契约正好相反。
    // 实链回执（rmul_2027，5.76 m 直行）：投影开着只前进 0.59 m（advance 0.10），关掉走满。
    const PointCloud scene = make_corridor();
    const NormalCache normals(scene, 0.3, 6);
    const PointCloudIndex index(scene, 0.5);

    constexpr int kFrames = 10;
    constexpr double kStep = 0.25;
    constexpr double kTravel = kFrames * kStep; // 2.5 m，x 从 1.0 走到 3.5，始终在走廊里
    ObservabilityReport fixed_report;
    ObservabilityReport legacy_report;
    const Eigen::Isometry3d fixed =
        run_corridor_frames(scene, index, normals, kFrames, kStep, false, fixed_report);
    const Eigen::Isometry3d legacy =
        run_corridor_frames(scene, index, normals, kFrames, kStep, true, legacy_report);

    // 先确认场景真的沿 x 退化，否则这条判据什么都没测。
    ASSERT_GT(fixed_report.unobservable_count, 0);
    ASSERT_GT(fixed_report.inlier_count, 1000U);
    const double fixed_advance = fixed.translation().x() - 1.0;
    const double legacy_advance = legacy.translation().x() - 1.0;
    printf("P0-0 回执(投影范围): 真实前进 %.3f m；新实现 %.4f m (advance %.4f)；"
           "父提交 %.4f m (advance %.4f)；不可观 %d/6 cond=%.3g\n",
           kTravel, fixed_advance, fixed_advance / kTravel, legacy_advance,
           legacy_advance / kTravel, fixed_report.unobservable_count,
           fixed_report.condition_number);

    // 配准完美时修正量恒为零，投影不该动任何东西：走多少就是多少。
    EXPECT_NEAR(fixed_advance, kTravel, 1e-6);
    EXPECT_NEAR(fixed.translation().y(), 1.5, 1e-6);
    EXPECT_NEAR(fixed.translation().z(), 0.2, 1e-6);
    // 旧实现必须被这条判据拦下：它几乎原地不动。
    EXPECT_LT(legacy_advance, 0.2 * kTravel) << "父提交的投影把里程计外推一起删掉了";
}

TEST(Observability, ProjectionUsesTheSameLeverAsTheInformationMatrix) {
    // 复核 P0-1：信息矩阵的旋转雅可比是 (p − lever) × n，对应的 6 维量是「绕 lever 转」；
    // 左乘增量的平移分量却是「绕世界原点转」的表述，两者相差 ω × lever。父提交不做换算，
    // 于是「绕 lever 纯旋转」被当成「平移 + 旋转」送进投影，在不可观方向上凭空注入
    // |ω|·|lever| 米的位移。lever 取 map→odom 的原点，机器人跑离起点后它就不再是本体位置。
    const PointCloud scene = make_corridor();
    const NormalCache normals(scene, 0.3, 6);
    const PointCloudIndex index(scene, 0.5);

    // 里程计原点在 (1.0, 1.5, 0)（|lever|=1.80 m，非零才会暴露缺项），本体已跑到 (2.0, 1.5, 0.2)。
    // 实链里 map→odom 的原点就是机器人起点在地图里的位置，量级同此。
    Eigen::Isometry3d guess = Eigen::Isometry3d::Identity();
    guess.translation() = Eigen::Vector3d(2.0, 1.5, 0.2);
    const Eigen::Vector3d lever = Eigen::Vector3d(1.0, 1.5, 0.0);
    // 绕 z 转 0.1 rad（5 Hz 下 0.5 rad/s 的一帧），旋转中心就是 lever。
    constexpr double kOmega = 0.1;
    const Eigen::Isometry3d spin =
        Eigen::Isometry3d(Eigen::AngleAxisd(kOmega, Eigen::Vector3d::UnitZ()));
    Eigen::Isometry3d about_lever = Eigen::Isometry3d::Identity();
    about_lever.linear() = spin.linear();
    about_lever.translation() = lever - spin * lever;
    const Eigen::Isometry3d registered = about_lever * guess;

    const PointCloud source = observed_from(scene, registered);
    const ObservabilityReport report =
        analyze_observability(source, registered, index, normals, 0.5, kThreshold, &lever);
    ASSERT_GT(report.unobservable_count, 0);

    const Eigen::Isometry3d fixed =
        project_registration_correction(guess, registered, report, lever);
    const Eigen::Isometry3d legacy = legacy_projection(guess, registered, report);
    // 判据一：投影后的修正量在不可观方向上不许剩分量（再投一次不该再变）。
    const double fixed_residue = unobservable_residue(fixed * guess.inverse(), report, lever);
    const double legacy_residue = unobservable_residue(legacy * guess.inverse(), report, lever);
    // 判据二：意图是「绕 lever 纯旋转」，投影后不该离配准结果太远。
    const double fixed_pose_err = (fixed * registered.inverse()).translation().norm();
    const double legacy_pose_err = (legacy * registered.inverse()).translation().norm();
    printf("P0-1 回执(力臂口径): 绕 lever 转 %.3f rad，|lever|=%.2f m，不可观 %d/6；"
           "不可观残留 新 %.6f / 父提交 %.6f；离配准结果 新 %.6f m / 父提交 %.6f m\n",
           kOmega, lever.norm(), report.unobservable_count, fixed_residue, legacy_residue,
           fixed_pose_err, legacy_pose_err);

    // 这条是力臂口径的判据本身：投影后的量再投一次必须一位不变。
    EXPECT_LT(fixed_residue, 1e-9);
    EXPECT_GT(legacy_residue, 0.05)
        << "父提交把 ω×lever 当成了平移修正量，投影后仍在不可观方向上留分量";
    // 阈值由缺项的物理量级 |ω|·|lever| 推出，不取拍脑袋的常数：父提交的位姿误差应当
    // 就是这个量级（实测 0.189 m vs 理论 0.180 m）；新实现只剩小角度线性化 O(ω²·|lever|)
    // 与特征基裁剪的零头，必须比它小一个量级。
    const double missing_term = kOmega * lever.norm();
    EXPECT_GT(legacy_pose_err, 0.8 * missing_term);
    EXPECT_LT(fixed_pose_err, 0.25 * missing_term);
}

TEST(Observability, FullyObservableProjectionIsIdentity) {
    // 全 6 方向可观时投影必须是恒等变换：否则「投影」本身就在偷偷改配准结果。
    const PointCloud box = make_box();
    const NormalCache normals(box, 0.4, 6);
    const PointCloudIndex index(box, 0.5);

    Eigen::Isometry3d guess = Eigen::Isometry3d::Identity();
    guess.translation() = Eigen::Vector3d(0.3, -0.2, 0.1);
    Eigen::Isometry3d registered = guess;
    registered.translation() += Eigen::Vector3d(0.05, -0.04, 0.02);
    registered.linear() =
        (Eigen::AngleAxisd(0.03, Eigen::Vector3d(0.4, 0.5, 0.7).normalized()) * guess.linear());

    const PointCloud source = observed_from(box, registered);
    const Eigen::Vector3d lever = Eigen::Vector3d(0.1, 0.2, 0.0);
    const ObservabilityReport report =
        analyze_observability(source, registered, index, normals, 0.5, kThreshold, &lever);
    ASSERT_EQ(report.unobservable_count, 0);

    const Eigen::Isometry3d projected =
        project_registration_correction(guess, registered, report, lever);
    EXPECT_LT((projected * registered.inverse()).translation().norm(), 1e-9);
    EXPECT_LT(Eigen::AngleAxisd((projected * registered.inverse()).linear()).angle(), 1e-9);
}

} // namespace diesel::localization_map
