// Copyright 2026 ZQU-Foray

#include "foray/localization_map/residual_clusterer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <unordered_map>
#include <vector>

namespace foray::localization_map {
namespace {

struct CellKey {
    std::int32_t x{0};
    std::int32_t y{0};
    std::int32_t z{0};

    bool operator==(const CellKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct CellHash {
    std::size_t operator()(const CellKey& key) const {
        const std::uint64_t packed = (static_cast<std::uint64_t>(key.x) << 42U) ^
                                     (static_cast<std::uint64_t>(key.y) << 21U) ^
                                     static_cast<std::uint64_t>(key.z);
        std::uint64_t h = packed * 0x9E3779B97F4A7C15ULL;
        h ^= h >> 29U;
        return static_cast<std::size_t>(h);
    }
};

CellKey cell_of(const Eigen::Vector3f& point, double size) {
    return CellKey{static_cast<std::int32_t>(std::floor(point.x() / size)),
                   static_cast<std::int32_t>(std::floor(point.y() / size)),
                   static_cast<std::int32_t>(std::floor(point.z() / size))};
}

} // namespace

/// target 的点与哈希格绑在一起：只存下标、靠外部再传一遍点云，
/// 会让「用 A 地图的格子查 B 地图的点」变成一个能编译的静默错误。
struct ResidualClusterer::TargetGrid {
    double cell{1.0};
    PointCloud points;
    std::unordered_map<CellKey, std::vector<std::uint32_t>, CellHash> cells;
};

ResidualClusterer::ResidualClusterer(ResidualClustererConfig config) : config_(config) {}

void ResidualClusterer::set_target(const PointCloud& target) {
    auto grid = std::make_shared<TargetGrid>();
    grid->cell = std::max(1e-3, config_.residual_threshold_m);
    grid->points = target;
    grid->cells.reserve(target.points.size());
    for (std::uint32_t i = 0; i < target.points.size(); ++i) {
        grid->cells[cell_of(target.points[i], grid->cell)].push_back(i);
    }
    target_grid_ = std::move(grid);
}

std::vector<ResidualCluster> ResidualClusterer::cluster(const PointCloud& source) const {
    if (target_grid_ == nullptr) {
        return {};
    }
    return cluster_against(source, *target_grid_);
}

std::vector<ResidualCluster> ResidualClusterer::cluster(const PointCloud& source,
                                                        const PointCloud& target) const {
    TargetGrid grid;
    grid.cell = std::max(1e-3, config_.residual_threshold_m);
    grid.points = target;
    grid.cells.reserve(target.points.size());
    for (std::uint32_t i = 0; i < target.points.size(); ++i) {
        grid.cells[cell_of(target.points[i], grid.cell)].push_back(i);
    }
    return cluster_against(source, grid);
}

std::vector<ResidualCluster> ResidualClusterer::cluster_against(const PointCloud& source,
                                                                const TargetGrid& grid) const {
    std::vector<ResidualCluster> clusters;
    if (source.empty() || grid.points.empty() || config_.cluster_tolerance_m <= 0.0) {
        return clusters;
    }

    const double query_cell = grid.cell;
    const auto& target_index = grid.cells;
    const auto& target = grid.points;

    std::vector<Eigen::Vector3f> residual_points;
    std::vector<double> residual_distances;
    for (const Eigen::Vector3f& point : source.points) {
        const CellKey center = cell_of(point, query_cell);
        double best = std::numeric_limits<double>::infinity();
        bool found = false;
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    const auto it =
                        target_index.find(CellKey{center.x + dx, center.y + dy, center.z + dz});
                    if (it == target_index.end()) {
                        continue;
                    }
                    for (const std::uint32_t candidate : it->second) {
                        const double distance =
                            (target.points[candidate] - point).cast<double>().squaredNorm();
                        if (distance < best) {
                            best = distance;
                            found = true;
                        }
                    }
                }
            }
        }
        if (!found || std::sqrt(best) > config_.residual_threshold_m) {
            residual_points.push_back(point);
            residual_distances.push_back(std::isfinite(best) ? std::sqrt(best)
                                                             : 4.0 * config_.residual_threshold_m);
        }
    }

    if (residual_points.empty()) {
        return clusters;
    }

    // 对残差点再做一次网格，用 26 邻域 BFS 得到欧氏簇。
    const double cluster_cell = std::max(1e-3, config_.cluster_tolerance_m);
    std::unordered_map<CellKey, std::vector<std::uint32_t>, CellHash> residual_index;
    residual_index.reserve(residual_points.size());
    for (std::uint32_t i = 0; i < residual_points.size(); ++i) {
        residual_index[cell_of(residual_points[i], cluster_cell)].push_back(i);
    }

    std::vector<bool> visited(residual_points.size(), false);
    for (std::uint32_t seed = 0; seed < residual_points.size(); ++seed) {
        if (visited[seed]) {
            continue;
        }
        visited[seed] = true;
        std::deque<std::uint32_t> queue{seed};
        std::vector<std::uint32_t> component;
        while (!queue.empty()) {
            const std::uint32_t current = queue.front();
            queue.pop_front();
            component.push_back(current);
            const CellKey center = cell_of(residual_points[current], cluster_cell);
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dz = -1; dz <= 1; ++dz) {
                        const auto it = residual_index.find(
                            CellKey{center.x + dx, center.y + dy, center.z + dz});
                        if (it == residual_index.end()) {
                            continue;
                        }
                        for (const std::uint32_t candidate : it->second) {
                            if (visited[candidate]) {
                                continue;
                            }
                            if ((residual_points[candidate] - residual_points[current])
                                    .cast<double>()
                                    .norm() <= config_.cluster_tolerance_m) {
                                visited[candidate] = true;
                                queue.push_back(candidate);
                            }
                        }
                    }
                }
            }
        }

        if (static_cast<int>(component.size()) < config_.min_cluster_size ||
            static_cast<int>(component.size()) > config_.max_cluster_size) {
            continue;
        }

        ResidualCluster cluster;
        cluster.point_count = static_cast<int>(component.size());
        cluster.min_corner = residual_points[component.front()];
        cluster.max_corner = residual_points[component.front()];
        Eigen::Vector3d sum = Eigen::Vector3d::Zero();
        double residual_sum = 0.0;
        for (const std::uint32_t index : component) {
            sum += residual_points[index].cast<double>();
            residual_sum += residual_distances[index];
            cluster.min_corner = cluster.min_corner.cwiseMin(residual_points[index]);
            cluster.max_corner = cluster.max_corner.cwiseMax(residual_points[index]);
        }
        cluster.centroid = sum / static_cast<double>(component.size());
        cluster.mean_residual_m = residual_sum / static_cast<double>(component.size());
        cluster.confidence =
            std::min(1.0, static_cast<double>(cluster.point_count) /
                              (2.0 * static_cast<double>(config_.min_cluster_size)));
        clusters.push_back(cluster);
    }

    std::sort(clusters.begin(), clusters.end(),
              [](const ResidualCluster& a, const ResidualCluster& b) {
                  return a.point_count > b.point_count;
              });
    return clusters;
}

} // namespace foray::localization_map
