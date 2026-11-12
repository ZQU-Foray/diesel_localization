// Copyright 2026 ZQU-Foray
//
// 配准后端接口：定位层只依赖这个抽象，PCL GICP / 内置 ICP / 未来的
// small_gicp 后端都可替换，而健康状态机与 ROS 节点不受影响。

#ifndef DIESEL_LOCALIZATION_MAP_ALIGNER_HPP_
#define DIESEL_LOCALIZATION_MAP_ALIGNER_HPP_

#include "diesel/localization_map/point_cloud.hpp"

#include <Eigen/Geometry>

#include <string>

namespace diesel::localization_map {

struct AlignOptions {
    /// 最大迭代次数；5Hz 配准下 10 次是精度与实时性的折中（decision.md D2）。
    int max_iterations{10};
    /// 对应点最大距离（米）。超过即视为外点，不参与位姿估计。
    double max_correspondence_distance_m{0.5};
    /// 相邻两轮**平移**变化小于该值（米）即认为平移收敛。
    double transformation_epsilon_m{1e-4};
    /// 相邻两轮**姿态**变化小于该值（弧度）即认为姿态收敛。
    ///
    /// 与 `transformation_epsilon_m` 分开：米和弧度不是同一个量纲。此前两者共用 1e-4，
    /// 等于要求每轮姿态变化小于 0.0057°，比点云 float32 所能表达的精度还细，实测里会把
    /// 迭代预算烧在噪声上、最后报「未收敛」（审计 L11）。1e-3 rad（0.057°）与 5 Hz
    /// 配准、厘米级残差的精度要求相称。PCL 的 GICP 后端用自己的收敛判据，不读本字段。
    double transformation_epsilon_rad{1e-3};
};

struct AlignResult {
    bool converged{false};
    Eigen::Isometry3d transform{Eigen::Isometry3d::Identity()};
    /// 0..1：内点比例越高、残差越小，fitness 越接近 1。
    double fitness{0.0};
    double mean_residual_m{0.0};
    double inlier_ratio{0.0};
    int iterations{0};
    std::string error;
};

/// 把 source 点云配准到 target 点云，返回 source->target 的变换。
class Aligner {
  public:
    virtual ~Aligner() = default;

    virtual AlignResult align(const PointCloud& source, const PointCloud& target,
                              const Eigen::Isometry3d& initial_guess) = 0;
};

} // namespace diesel::localization_map

#endif // DIESEL_LOCALIZATION_MAP_ALIGNER_HPP_
