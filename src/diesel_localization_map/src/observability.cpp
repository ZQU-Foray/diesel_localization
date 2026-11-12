// Copyright 2026 ZQU-Foray

#include "diesel/localization_map/observability.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>

namespace diesel::localization_map {

namespace {

/// 邻域点数上限：法向估计只需要一个局部面片，取 30 个点已足够稳定，
/// 再多的点数对 PCA 几乎没有增益，却会让法向构建从 O(n) 变成 O(n·k) 的常数翻倍。
constexpr std::size_t kMaxNeighbors = 30;

/// 平面判据：邻域协方差的最小特征值必须小于最大特征值的这个比例，才认为点落在
/// 一个面片上并给出法向。取值来源——球状或线状邻域（多线束交叉、墙角、树叶）的
/// 最小特征值与最大特征值同量级，其「法向」对扰动极不稳定，宁可不给。
constexpr double kPlanarityRatio = 0.1;

/// 残差方差的下限。残差整体为 0（源点云与目标点云完全重合）时 σ² 会退化为 0，
/// 信息矩阵随之变成 inf；这里用 1e-6 m²（即 1 mm 残差）兜底。
constexpr double kMinResidualVariance = 1e-6;

} // namespace

NormalCache::NormalCache(const PointCloud& cloud, double cell_size_m, std::size_t min_neighbors)
    : cell_size_(std::max(1e-3, cell_size_m)) {
    compute_normals(cloud, cell_size_, min_neighbors);
}

NormalCache::CellKey NormalCache::cell_of(const Eigen::Vector3f& point, double size) {
    return CellKey{static_cast<std::int32_t>(std::floor(point.x() / size)),
                   static_cast<std::int32_t>(std::floor(point.y() / size)),
                   static_cast<std::int32_t>(std::floor(point.z() / size))};
}

std::size_t NormalCache::CellHash::operator()(const CellKey& key) const {
    const std::uint64_t packed = (static_cast<std::uint64_t>(key.x) << 42U) ^
                                 (static_cast<std::uint64_t>(key.y) << 21U) ^
                                 static_cast<std::uint64_t>(key.z);
    // 与 PointCloudIndex 用同一个 64 位黄金比例常数做 Fibonacci hashing，
    // 两个体素索引的哈希行为保持一致，便于排查时的直觉复用。
    std::uint64_t h = packed * 0x9E3779B97F4A7C15ULL;
    h ^= h >> 29U;
    return static_cast<std::size_t>(h);
}

void NormalCache::compute_normals(const PointCloud& cloud, double cell_size_m,
                                  std::size_t min_neighbors) {
    normals_.assign(cloud.points.size(), Eigen::Vector3f::Zero());
    valid_.assign(cloud.points.size(), 0U);
    if (cloud.empty()) {
        return;
    }

    std::unordered_map<CellKey, std::vector<std::uint32_t>, CellHash> cells;
    cells.reserve(cloud.points.size());
    for (std::size_t i = 0; i < cloud.points.size(); ++i) {
        cells[cell_of(cloud.points[i], cell_size_m)].push_back(static_cast<std::uint32_t>(i));
    }

    const std::size_t wanted = std::max<std::size_t>(min_neighbors, 3U);
    std::vector<Eigen::Vector3d> neighbors;
    neighbors.reserve(kMaxNeighbors);

    for (std::size_t i = 0; i < cloud.points.size(); ++i) {
        const Eigen::Vector3f& point = cloud.points[i];
        const CellKey center = cell_of(point, cell_size_m);
        neighbors.clear();
        for (int dx = -1; dx <= 1 && neighbors.size() < kMaxNeighbors; ++dx) {
            for (int dy = -1; dy <= 1 && neighbors.size() < kMaxNeighbors; ++dy) {
                for (int dz = -1; dz <= 1 && neighbors.size() < kMaxNeighbors; ++dz) {
                    const auto it =
                        cells.find(CellKey{center.x + dx, center.y + dy, center.z + dz});
                    if (it == cells.end()) {
                        continue;
                    }
                    for (const std::uint32_t candidate : it->second) {
                        if (neighbors.size() >= kMaxNeighbors) {
                            break;
                        }
                        neighbors.push_back(cloud.points[candidate].cast<double>());
                    }
                }
            }
        }
        if (neighbors.size() < wanted) {
            continue;
        }

        Eigen::Vector3d mean = Eigen::Vector3d::Zero();
        for (const Eigen::Vector3d& neighbor : neighbors) {
            mean += neighbor;
        }
        mean /= static_cast<double>(neighbors.size());

        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
        for (const Eigen::Vector3d& neighbor : neighbors) {
            const Eigen::Vector3d centered = neighbor - mean;
            covariance += centered * centered.transpose();
        }
        covariance /= static_cast<double>(neighbors.size());

        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
        if (solver.info() != Eigen::Success) {
            continue;
        }
        // Eigen 按升序返回特征值：第 0 个是最小特征值，其方向即邻域的最薄方向＝法向。
        const double smallest = solver.eigenvalues()(0);
        const double largest = solver.eigenvalues()(2);
        if (largest <= 0.0 || smallest > kPlanarityRatio * largest) {
            continue;
        }
        const Eigen::Vector3d normal = solver.eigenvectors().col(0).normalized();
        normals_[i] = normal.cast<float>();
        valid_[i] = 1U;
    }
}

bool NormalCache::valid(std::size_t index) const {
    return index < valid_.size() && valid_[index] != 0U;
}

Eigen::Vector3f NormalCache::normal(std::size_t index) const {
    return valid(index) ? normals_[index] : Eigen::Vector3f::Zero();
}

double NormalCache::coverage() const {
    if (valid_.empty()) {
        return 0.0;
    }
    std::size_t count = 0;
    for (const std::uint8_t flag : valid_) {
        if (flag != 0U) {
            ++count;
        }
    }
    return static_cast<double>(count) / static_cast<double>(valid_.size());
}

ObservabilityReport analyze_observability(const PointCloud& source,
                                          const Eigen::Isometry3d& transform,
                                          const PointCloudIndex& index, const NormalCache& normals,
                                          double max_correspondence_distance_m,
                                          double eigenvalue_ratio_threshold,
                                          const Eigen::Vector3d* lever_origin) {
    ObservabilityReport report;
    if (source.empty()) {
        return report;
    }

    // 第一遍收集雅可比与点面残差：σ² 要由这一帧的残差统计得出，所以必须分两遍。
    // 力臂原点：默认绕本体转；调用方要修正外部位姿时必须传它的原点（审计 L15）。
    const Eigen::Vector3d lever = lever_origin != nullptr ? *lever_origin : transform.translation();
    std::vector<Eigen::Matrix<double, kDofCount, 1>> jacobians;
    std::vector<double> residuals;
    jacobians.reserve(source.points.size());
    residuals.reserve(source.points.size());

    for (const Eigen::Vector3f& point : source.points) {
        const Eigen::Vector3d moved = transform * point.cast<double>();
        std::uint32_t match = 0;
        double distance = 0.0;
        if (!index.nearest(moved.cast<float>(), max_correspondence_distance_m, &match, &distance)) {
            continue;
        }
        if (!normals.valid(match)) {
            continue;
        }
        const Eigen::Vector3d normal = normals.normal(match).cast<double>();
        const Eigen::Vector3d target_point = index.cloud().points[match].cast<double>();
        // 点面残差：沿法向的投影。墙面切向上的位移在这里恒为 0，
        // 正是它把「点对点重匹配造成的假性可观」暴露出来的原因（decision.md D23）。
        const double residual = normal.dot(moved - target_point);

        Eigen::Matrix<double, kDofCount, 1> jacobian;
        jacobian.head<3>() = normal;
        // 力臂相对 `lever` 的平移，使旋转自由度代表「绕 lever 的旋转」——
        // 与下游被修正量的语义一致。若用 map 原点做力臂，机器人离原点越远，
        // 同一个旋转在投影里就会被放大成巨大的平移。
        jacobian.tail<3>() = (moved - lever).cross(normal);
        jacobians.push_back(jacobian);
        residuals.push_back(residual);
    }

    if (jacobians.empty()) {
        // 没有一对有效点面约束：6 个方向全不可观（unobservable_count 保持默认值 6）。
        return report;
    }

    double squared_sum = 0.0;
    for (const double residual : residuals) {
        squared_sum += residual * residual;
    }
    const double variance =
        std::max(squared_sum / static_cast<double>(residuals.size()), kMinResidualVariance);

    for (const auto& jacobian : jacobians) {
        report.information += (jacobian * jacobian.transpose()) / variance;
    }
    report.inlier_count = jacobians.size();

    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, kDofCount, kDofCount>> solver(
        report.information);
    if (solver.info() != Eigen::Success) {
        return report;
    }
    report.eigenvalues = solver.eigenvalues();
    report.eigenvectors = solver.eigenvectors();

    const double largest = report.eigenvalues.maxCoeff();
    if (!(largest > 0.0)) {
        return report;
    }
    report.normalized_eigenvalues = report.eigenvalues / largest;

    const double smallest = std::max(report.eigenvalues.minCoeff(), 0.0);
    report.condition_number = smallest > 0.0 ? largest / smallest : kInfiniteConditionNumber;

    report.unobservable_count = 0;
    for (std::size_t i = 0; i < kDofCount; ++i) {
        const bool observable = report.normalized_eigenvalues(i) >= eigenvalue_ratio_threshold;
        report.observable[i] = observable;
        if (!observable) {
            ++report.unobservable_count;
        }
    }
    return report;
}

Eigen::Matrix<double, kDofCount, 1>
project_delta_to_observable(const Eigen::Matrix<double, kDofCount, 1>& delta,
                            const ObservabilityReport& report) {
    Eigen::Matrix<double, kDofCount, 1> projected = Eigen::Matrix<double, kDofCount, 1>::Zero();
    for (std::size_t i = 0; i < kDofCount; ++i) {
        if (!report.observable[i]) {
            continue;
        }
        const Eigen::Matrix<double, kDofCount, 1> direction = report.eigenvectors.col(i);
        projected += direction * direction.dot(delta);
    }
    return projected;
}

} // namespace diesel::localization_map
