// Copyright 2026 ZQU-Foray
//
// 配准残差聚类：把“落在地图之外的观测点”聚成无标号簇，作为动态物体
// 候选（decision.md D6）。标签由下游 vision/decision 给出，本层只给几何与置信度。

#ifndef FORAY_LOCALIZATION_MAP_RESIDUAL_CLUSTERER_HPP_
#define FORAY_LOCALIZATION_MAP_RESIDUAL_CLUSTERER_HPP_

#include "foray/localization_map/point_cloud.hpp"

#include <Eigen/Core>

#include <memory>
#include <vector>

namespace foray::localization_map {

struct ResidualClustererConfig {
    /// 点到地图最近点的欧氏距离超过该值即为残差点（米）。
    double residual_threshold_m{0.3};
    /// 同一簇内相邻点的最大间距（米）。
    double cluster_tolerance_m{0.3};
    /// 小于该点数的簇视为噪声丢弃。
    int min_cluster_size{5};
    /// 大于该点数的簇不输出，避免把整片地图过期误报成动态物体。
    int max_cluster_size{500};
};

struct ResidualCluster {
    Eigen::Vector3d centroid{Eigen::Vector3d::Zero()};
    int point_count{0};
    Eigen::Vector3f min_corner{Eigen::Vector3f::Zero()};
    Eigen::Vector3f max_corner{Eigen::Vector3f::Zero()};
    double mean_residual_m{0.0};
    /// 0..1，仅由簇的点数相对 min_cluster_size 决定，不含语义置信度。
    double confidence{0.0};
};

class ResidualClusterer {
  public:
    explicit ResidualClusterer(ResidualClustererConfig config = ResidualClustererConfig{});

    /// 预建 target 的地图哈希格。整图建格是这条 10 Hz 路径上最贵的固定开销，
    /// 而它只依赖地图、不依赖当前帧，没有理由每帧重建（decision.md D35）。
    /// 换地图后必须重新调用：这里不做「目标变了就自动重建」的猜测，
    /// 因为静默地用旧地图判残差会把整片新地图报成动态物体。
    void set_target(const PointCloud& target);

    /// 是否已有可用的 target 索引。
    bool has_target() const { return target_grid_ != nullptr; }

    /// 用 set_target 建好的索引做聚类；未设置 target 时返回空。
    std::vector<ResidualCluster> cluster(const PointCloud& source) const;

    /// 在 target 地图上为 source 的每个点找最近邻，残差点做欧氏聚类。
    /// 每次调用都会重建 target 索引，只在测试与一次性调用里使用。
    std::vector<ResidualCluster> cluster(const PointCloud& source, const PointCloud& target) const;

  private:
    struct TargetGrid;

    std::vector<ResidualCluster> cluster_against(const PointCloud& source,
                                                 const TargetGrid& grid) const;

    ResidualClustererConfig config_;
    std::shared_ptr<const TargetGrid> target_grid_;
};

} // namespace foray::localization_map

#endif // FORAY_LOCALIZATION_MAP_RESIDUAL_CLUSTERER_HPP_
