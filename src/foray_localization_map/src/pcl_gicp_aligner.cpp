// Copyright 2026 ZQU-Foray

#include "foray/localization_map/pcl_gicp_aligner.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>

#include <array>
#include <cstdint>
#include <string>

#include "foray/localization_map/alignment_metrics.hpp"

namespace foray::localization_map {
namespace {

using PclCloud = pcl::PointCloud<pcl::PointXYZ>;

PclCloud::Ptr to_pcl(const PointCloud& cloud) {
    PclCloud::Ptr out(new PclCloud());
    out->reserve(cloud.points.size());
    for (const Eigen::Vector3f& point : cloud.points) {
        out->push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
    }
    out->width = static_cast<std::uint32_t>(out->size());
    out->height = 1;
    return out;
}

/// target 的「内容指纹」：点数 + 16 个等距采样点的坐标和。
///
/// 只比地址是不够的（审计 L14）：同一个 `PointCloud` 对象完全可能被整体替换内容
/// （`map = load_pcd(...)` 这种赋值不会换地址），此时地址相等而数据全变，缓存会
/// 让 GICP 继续对着旧地图优化，而质量评估用的是新地图——结果是「配准明明收敛了，
/// 内点率却是 0」这种自相矛盾的读数。指纹加点数把这种情况挡住。
std::array<float, 4> target_fingerprint(const PointCloud& cloud) {
    std::array<float, 4> fingerprint{0.0F, 0.0F, 0.0F, 0.0F};
    constexpr std::size_t kSamples = 16;
    if (cloud.points.empty()) {
        return fingerprint;
    }
    for (std::size_t index = 0; index < kSamples; ++index) {
        const std::size_t position =
            cloud.points.size() == 1 ? 0 : index * (cloud.points.size() - 1) / (kSamples - 1);
        const Eigen::Vector3f& point = cloud.points[position];
        fingerprint[0] += point.x();
        fingerprint[1] += point.y();
        fingerprint[2] += point.z();
        fingerprint[3] += point.squaredNorm();
    }
    return fingerprint;
}

/// PCL 1.12 把 `nr_iterations_` 放在 `GeneralizedIterativeClosestPoint` 的 public
/// using 区（`/usr/include/pcl-1.12/pcl/registration/gicp.h:59-68`），但那是实现细节。
/// 版本耦合点收在这一个派生类里：升级 PCL 时只有这里会编译报错，而不是散落在 align() 里。
struct GicpWithIterationCount
    : pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> {
    int iterations() const { return static_cast<int>(this->nr_iterations_); }
};

} // namespace

/// PCL 对象与 target 缓存都藏在这里，头文件因此不必认识 pcl:: 的任何类型。
struct PclGicpAligner::Impl {
    GicpWithIterationCount gicp;
    /// 上一次 setInputTarget 的目标点云：地址 + 点数 + 内容指纹三重识别。
    const PointCloud* cached_target{nullptr};
    std::size_t cached_target_size{0};
    std::array<float, 4> cached_target_fingerprint{0.0F, 0.0F, 0.0F, 0.0F};
    PclCloud::Ptr cached_target_pcl;
};

PclGicpAligner::PclGicpAligner(AlignOptions options)
    : options_(options), impl_(std::make_unique<Impl>()) {}

PclGicpAligner::~PclGicpAligner() = default;

void PclGicpAligner::set_options(const AlignOptions& options) {
    options_ = options;
}

AlignResult PclGicpAligner::align(const PointCloud& source, const PointCloud& target,
                                  const Eigen::Isometry3d& initial_guess) {
    AlignResult result;
    result.transform = initial_guess;

    if (source.empty() || target.empty()) {
        result.error = "配准输入点云为空";
        return result;
    }

    const PclCloud::Ptr source_pcl = to_pcl(source);

    // 复用同一个 GICP 对象，只在 target 换掉时重新 setInputTarget
    // （decision.md D27）：PCL 1.12 的 setInputTarget() 会 target_covariances_.reset()
    // 并在下次 initCompute() 重算整个 target 协方差 + KdTree，这是配准里最贵的一段。
    // 而 setInputSource() 会清 input_covariances_，所以逐帧换 source 是安全的——
    // 离线基准里 12 帧不同区域的位姿结果与「每帧新建」逐位相同，速度快 2.02 倍。
    auto& gicp = impl_->gicp;
    // 参数每次配准都下发（审计 L14）：三个 setter 只是给成员赋值，代价不可测量，换来的是
    // 「改参立即生效」——不再依赖「构造时配置一次」这种在 set_options() 面前会失效的隐式假设。
    gicp.setMaximumIterations(options_.max_iterations);
    gicp.setMaxCorrespondenceDistance(options_.max_correspondence_distance_m);
    gicp.setTransformationEpsilon(options_.transformation_epsilon_m);
    // correspondence_randomness 保留 PCL 默认 20：没有实测数据支撑调参，
    // 先不暴露成 ROS 参数，避免凭空多出一个没人验证过的旋钮（D21）。

    // 缓存命中要求「地址 + 点数 + 内容指纹」三者全等：只比地址会把「同一个对象被整体
    // 换内容」当成没换图，于是对着旧靶标优化、拿新靶标算质量（审计 L14）。
    const std::array<float, 4> fingerprint = target_fingerprint(target);
    const bool target_unchanged = impl_->cached_target_pcl != nullptr &&
                                  impl_->cached_target == &target &&
                                  impl_->cached_target_size == target.points.size() &&
                                  impl_->cached_target_fingerprint == fingerprint;
    if (!target_unchanged) {
        impl_->cached_target_pcl = to_pcl(target);
        gicp.setInputTarget(impl_->cached_target_pcl);
        impl_->cached_target = &target;
        impl_->cached_target_size = target.points.size();
        impl_->cached_target_fingerprint = fingerprint;
    }
    gicp.setInputSource(source_pcl);

    PclCloud aligned;
    gicp.align(aligned, initial_guess.matrix().cast<float>());

    result.transform = gicp.getFinalTransformation().cast<double>();
    result.converged = gicp.hasConverged();
    // PCL 没有迭代次数的 getter；版本耦合点收在 GicpWithIterationCount 一处。
    result.iterations = gicp.iterations();

    // 质量一律走公共口径：PCL 的 getFitnessScore() 是均方误差（越小越好），
    // 与本项目 fitness（0..1，越大越好）语义相反，绝不能直接填进 AlignResult。
    const AlignmentQuality quality = evaluate_alignment(source, target, result.transform,
                                                        options_.max_correspondence_distance_m);
    result.inlier_ratio = quality.inlier_ratio;
    result.mean_residual_m = quality.mean_residual_m;
    result.fitness = quality.fitness;

    // 失败原因必须写清楚：节点侧只看到「registration rejected」时，
    // 无从区分是没收敛还是压根没对上（decision.md D21）。
    if (result.inlier_ratio <= 0.0) {
        result.error = "GICP 无有效对应（source=" + std::to_string(source.size()) +
                       " target=" + std::to_string(target.size()) +
                       " iterations=" + std::to_string(result.iterations) + "）";
    } else if (!result.converged) {
        result.error = "GICP 未收敛（iterations=" + std::to_string(result.iterations) + "）";
    }
    return result;
}

} // namespace foray::localization_map
