// Copyright 2026 ZQU-Foray
//
// 可观测性分析：把「这一帧配准到底约束住了哪几个自由度」变成可计算的量。
//
// 为什么必须有这一层：视野被裁（传感器视场受限、遮挡、主动 ROI）之后，内点变少、
// 几何约束变弱，但配准仍然可能「收敛」到一个毫无意义的解——例如沿走廊方向平移时，
// 点对点残差依旧很小。此时如果只有「整体可信 / 整体否决」两档，就会把「只丢一维」
// 退化成「完全不可用」（decision.md D23）。
//
// 因此这里用**点面残差**构造 6x6 信息矩阵再做特征分解：点对点在墙面切向上会重匹配到
// 相邻点、残差假性变小，而点面残差的切向分量恒为 0，能把「这一维没被约束」如实暴露出来。

#ifndef FORAY_LOCALIZATION_MAP_OBSERVABILITY_HPP_
#define FORAY_LOCALIZATION_MAP_OBSERVABILITY_HPP_

#include "foray/localization_map/alignment_metrics.hpp"
#include "foray/localization_map/point_cloud.hpp"

#include <Eigen/Geometry>

#include <array>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace foray::localization_map {

/// 6 个自由度：[0..2] 平移 x/y/z，[3..5] 旋转 rx/ry/rz（小角度）。
inline constexpr std::size_t kDofCount = 6;

/// 目标点云法向缓存（邻域 PCA）。在地图加载时构建一次，之后按点下标查询。
///
/// 用体素分桶而不是 KD 树：本仓只需要「每个点取局部邻域求协方差」这一种查询，
/// 而分桶是纯标准库实现，不引入 PCL/KdTree 依赖（`FORAY_WITH_PCL=OFF` 时也要能编）。
class NormalCache {
  public:
    /// `cell_size_m` 决定邻域半径的量级，取地图体素的 2 倍即可覆盖一个局部面片。
    /// `min_neighbors` 以下的邻域不产生法向（退化为无效法向）。
    NormalCache(const PointCloud& cloud, double cell_size_m, std::size_t min_neighbors);

    /// 该点是否有有效法向（邻域点数够、且不是各向同性的相关点团）。
    bool valid(std::size_t index) const;

    /// 该点的单位法向；`valid(index)` 为 false 时返回零向量。
    Eigen::Vector3f normal(std::size_t index) const;

    /// 有效法向的点占比，用于日志与配置自检。
    double coverage() const;

    std::size_t size() const { return normals_.size(); }

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

    void compute_normals(const PointCloud& cloud, double cell_size_m, std::size_t min_neighbors);

    std::vector<Eigen::Vector3f> normals_;
    std::vector<std::uint8_t> valid_;
    double cell_size_{1.0};
};

/// 可观测性报告：信息矩阵本身 + 归一化特征值 + 特征方向。
///
/// 特征向量按 Eigen 的升序排列，因此 `normalized_eigenvalues[0]` 是最不可观的方向。
struct ObservabilityReport {
    /// 信息矩阵 H = Σ Jᵢᵀ Jᵢ / σᵢ²，Jᵢ = [nᵢᵀ, (pᵢ × nᵢ)ᵀ]。
    Eigen::Matrix<double, kDofCount, kDofCount> information{
        Eigen::Matrix<double, kDofCount, kDofCount>::Zero()};
    /// 归一化特征值：各值除以最大特征值，最大者恒为 1。
    Eigen::Matrix<double, kDofCount, 1> normalized_eigenvalues{
        Eigen::Matrix<double, kDofCount, 1>::Zero()};
    /// 归一化前的原始特征值（用于条件数）。
    Eigen::Matrix<double, kDofCount, 1> eigenvalues{Eigen::Matrix<double, kDofCount, 1>::Zero()};
    /// 特征向量，第 i 列对应 `eigenvalues[i]`。
    Eigen::Matrix<double, kDofCount, kDofCount> eigenvectors{
        Eigen::Matrix<double, kDofCount, kDofCount>::Identity()};
    /// 最大 / 最小特征值；最小为 0 时取 kInfiniteConditionNumber。
    double condition_number{0.0};
    /// 参与统计的内点数。
    std::size_t inlier_count{0};
    /// 每个特征方向是否可观（阈值由调用方给定）。
    std::array<bool, kDofCount> observable{};
    /// 不可观方向个数，等于 `observable` 中 false 的个数。
    int unobservable_count{static_cast<int>(kDofCount)};

    /// 是否退化：只要有一个方向不可观就算退化（是否因此降级由健康状态机决定）。
    bool degenerate() const { return unobservable_count > 0; }
};

/// 条件数上限的哨兵值：最小特征值为 0（即某个方向完全没有约束）时用它代替 inf，
/// 免得下游拿 inf 做比较得到 NaN。
inline constexpr double kInfiniteConditionNumber = 1e12;

/// 在 `transform` 处计算可观测性。
///
/// `transform` 是 source→target 的当前估计；`index` 与 `normals` 必须对应同一个目标点云。
/// `eigenvalue_ratio_threshold` 是归一化特征值的下限：低于它视为该方向不可观。
/// 阈值应随内点数自适应（点数少时特征值噪声大），调用方按 `inlier_count` 调整后传入。
///
/// `lever_origin` 是旋转自由度所绕的点（世界坐标，单位 m）。旋转雅可比
/// `(p - lever_origin) × n` 里的力臂必须与「谁被修正」一致：把旋转解释成「绕本体
/// 转」和「绕里程计原点转」，等价的平移量可以差几十米，掩码也会随之翻转。传
/// `nullptr` 表示绕 `transform.translation()`（本体）转；下游要修正的是 `map→odom`
/// 这种外部位姿时，必须显式传它的原点（审计 L15）。
ObservabilityReport analyze_observability(const PointCloud& source,
                                          const Eigen::Isometry3d& transform,
                                          const PointCloudIndex& index, const NormalCache& normals,
                                          double max_correspondence_distance_m,
                                          double eigenvalue_ratio_threshold,
                                          const Eigen::Vector3d* lever_origin = nullptr);

/// 把 6 维修正量投影到可观子空间，不可观方向上的分量被清零。
///
/// `delta` 的顺序与信息矩阵一致：[dx, dy, dz, d_rx, d_ry, d_rz]（旋转按小角度处理）。
/// 投影后的修正量保证不会把里程计外推出来的位姿在不可观方向上拉走（decision.md D23）。
Eigen::Matrix<double, kDofCount, 1>
project_delta_to_observable(const Eigen::Matrix<double, kDofCount, 1>& delta,
                            const ObservabilityReport& report);

} // namespace foray::localization_map

#endif // FORAY_LOCALIZATION_MAP_OBSERVABILITY_HPP_
