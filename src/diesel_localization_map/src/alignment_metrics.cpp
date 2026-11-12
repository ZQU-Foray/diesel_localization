// Copyright 2026 ZQU-Foray

#include "diesel/localization_map/alignment_metrics.hpp"

#include <algorithm>
#include <cmath>

namespace diesel::localization_map {

PointCloudIndex::PointCloudIndex(const PointCloud& cloud, double max_correspondence_distance_m)
    : cloud_(cloud), cell_size_(std::max(1e-3, max_correspondence_distance_m)) {
    cells_.reserve(cloud.points.size());
    for (std::uint32_t i = 0; i < cloud.points.size(); ++i) {
        cells_[cell_of(cloud.points[i], cell_size_)].push_back(i);
    }
}

PointCloudIndex::CellKey PointCloudIndex::cell_of(const Eigen::Vector3f& point, double size) {
    return CellKey{static_cast<std::int32_t>(std::floor(point.x() / size)),
                   static_cast<std::int32_t>(std::floor(point.y() / size)),
                   static_cast<std::int32_t>(std::floor(point.z() / size))};
}

std::size_t PointCloudIndex::CellHash::operator()(const CellKey& key) const {
    const std::uint64_t packed = (static_cast<std::uint64_t>(key.x) << 42U) ^
                                 (static_cast<std::uint64_t>(key.y) << 21U) ^
                                 static_cast<std::uint64_t>(key.z);
    // 0x9E3779B97F4A7C15 是 64 位黄金比例常数（2^64/φ），Fibonacci hashing 的标准混合乘数；
    // 用它把体素坐标打散，减少 unordered_map 的桶碰撞。
    std::uint64_t h = packed * 0x9E3779B97F4A7C15ULL;
    h ^= h >> 29U;
    return static_cast<std::size_t>(h);
}

bool PointCloudIndex::nearest(const Eigen::Vector3f& query, double max_distance_m,
                              std::uint32_t* index_out, double* distance_out) const {
    const CellKey center = cell_of(query, cell_size_);
    double best = max_distance_m * max_distance_m;
    bool found = false;
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                const auto it = cells_.find(CellKey{center.x + dx, center.y + dy, center.z + dz});
                if (it == cells_.end()) {
                    continue;
                }
                for (const std::uint32_t candidate : it->second) {
                    const double distance =
                        (cloud_.points[candidate] - query).cast<double>().squaredNorm();
                    if (distance < best) {
                        best = distance;
                        *index_out = candidate;
                        found = true;
                    }
                }
            }
        }
    }
    if (found) {
        *distance_out = std::sqrt(best);
    }
    return found;
}

AlignmentQuality evaluate_alignment(const PointCloud& source, const PointCloud& target,
                                    const Eigen::Isometry3d& transform,
                                    double max_correspondence_distance_m) {
    if (source.empty() || target.empty()) {
        return AlignmentQuality{};
    }
    const PointCloudIndex index(target, max_correspondence_distance_m);
    return evaluate_alignment(source, index, transform, max_correspondence_distance_m);
}

AlignmentQuality evaluate_alignment(const PointCloud& source, const PointCloudIndex& index,
                                    const Eigen::Isometry3d& transform,
                                    double max_correspondence_distance_m) {
    AlignmentQuality quality;
    if (source.empty()) {
        return quality;
    }

    std::size_t inliers = 0;
    double residual_sum = 0.0;
    for (const Eigen::Vector3f& point : source.points) {
        const Eigen::Vector3f moved = (transform * point.cast<double>()).cast<float>();
        std::uint32_t match = 0;
        double distance = 0.0;
        if (index.nearest(moved, max_correspondence_distance_m, &match, &distance)) {
            ++inliers;
            residual_sum += distance;
        }
    }

    quality.inlier_ratio = static_cast<double>(inliers) / static_cast<double>(source.points.size());
    quality.mean_residual_m = inliers > 0U ? residual_sum / static_cast<double>(inliers) : 0.0;
    quality.fitness =
        quality.inlier_ratio / (1.0 + quality.mean_residual_m / max_correspondence_distance_m);
    return quality;
}

} // namespace diesel::localization_map
