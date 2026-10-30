// Copyright 2026 ZQU-Foray
//
// 配准质量评估：把「内点比例 / 平均残差 / fitness」的口径集中到一处。
//
// 为什么必须共用：PCL GICP 自带的 getFitnessScore() 是均方误差（越小越好），
// 与本项目 fitness（0..1，越大越好）语义相反。若各后端各填一套，
// 健康状态机的阈值会随后端漂移，换后端就等于换了一套判定标准（decision.md D21）。

#ifndef FORAY_LOCALIZATION_MAP_ALIGNMENT_METRICS_HPP_
#define FORAY_LOCALIZATION_MAP_ALIGNMENT_METRICS_HPP_

#include "foray/localization_map/point_cloud.hpp"

#include <Eigen/Geometry>

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace foray::localization_map {

/// 配准质量。三个字段共同决定 RegistrationSample.valid 与健康状态，故口径必须与后端无关。
struct AlignmentQuality {
    /// 0..1：内点比例越高、残差越小，fitness 越接近 1。
    double fitness{0.0};
    /// 内点的平均最近邻距离（米）；无内点时为 0。
    double mean_residual_m{0.0};
    /// 0..1：在 max_correspondence_distance 内找到对应点的 source 点占比。
    double inlier_ratio{0.0};
};

/// 体素哈希近邻索引。按体素分桶，查询只扫 3x3x3 邻域，
/// 避免为每帧配准重建 KD 树（单帧 1e4 量级点的实测依据见 decision.md D2）。
class PointCloudIndex {
  public:
    /// 索引只持有 `cloud` 的引用、不复制点云：调用方须保证 `cloud` 活过本对象。
    PointCloudIndex(const PointCloud& cloud, double max_correspondence_distance_m);

    /// 查询最近点。距离超过 `max_distance_m` 视为无匹配，返回 false 时不动输出参数。
    bool nearest(const Eigen::Vector3f& query, double max_distance_m, std::uint32_t* index_out,
                 double* distance_out) const;

    /// 被索引的点云。可观测性分析需要按下标取回目标点坐标来算点面残差（decision.md D23）。
    const PointCloud& cloud() const { return cloud_; }

  private:
    struct CellKey {
        std::int32_t x{0};
        std::int32_t y{0};
        std::int32_t z{0};

        bool operator==(const CellKey& other) const {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    struct CellHash {
        std::size_t operator()(const CellKey& key) const;
    };

    static CellKey cell_of(const Eigen::Vector3f& point, double size);

    const PointCloud& cloud_;
    double cell_size_{1.0};
    std::unordered_map<CellKey, std::vector<std::uint32_t>, CellHash> cells_;
};

/// 评估 `transform` 把 source 变换到 target 之后的配准质量。
AlignmentQuality evaluate_alignment(const PointCloud& source, const PointCloud& target,
                                    const Eigen::Isometry3d& transform,
                                    double max_correspondence_distance_m);

/// 复用已建索引的版本：同一帧里已为目标点云建过索引时用它，省一次 O(n) 分桶。
AlignmentQuality evaluate_alignment(const PointCloud& source, const PointCloudIndex& index,
                                    const Eigen::Isometry3d& transform,
                                    double max_correspondence_distance_m);

} // namespace foray::localization_map

#endif // FORAY_LOCALIZATION_MAP_ALIGNMENT_METRICS_HPP_
