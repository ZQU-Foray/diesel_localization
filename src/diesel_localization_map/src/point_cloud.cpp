// Copyright 2026 ZQU-Foray

#include "diesel/localization_map/point_cloud.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>

namespace diesel::localization_map {
namespace {

struct VoxelKey {
    std::int32_t x{0};
    std::int32_t y{0};
    std::int32_t z{0};

    bool operator==(const VoxelKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey& key) const {
        // 三个 int32 打包进 uint64 后做一次 64 位混合，避免简单的异或碰撞。
        const std::uint64_t packed = (static_cast<std::uint64_t>(key.x) << 42U) ^
                                     (static_cast<std::uint64_t>(key.y) << 21U) ^
                                     static_cast<std::uint64_t>(key.z);
        std::uint64_t h = packed * 0x9E3779B97F4A7C15ULL;
        h ^= h >> 29U;
        return static_cast<std::size_t>(h);
    }
};

struct VoxelAccum {
    Eigen::Vector3d sum{Eigen::Vector3d::Zero()};
    std::size_t count{0};
};

std::int32_t voxel_index(float value, double leaf) {
    return static_cast<std::int32_t>(std::floor(static_cast<double>(value) / leaf));
}

} // namespace

PointCloud voxel_downsample(const PointCloud& cloud, double leaf_size_m) {
    if (leaf_size_m <= 0.0 || cloud.points.empty()) {
        return cloud;
    }

    std::unordered_map<VoxelKey, VoxelAccum, VoxelKeyHash> voxels;
    voxels.reserve(cloud.points.size() / 2U + 1U);

    for (const Eigen::Vector3f& point : cloud.points) {
        if (!point.allFinite()) {
            continue;
        }
        const VoxelKey key{voxel_index(point.x(), leaf_size_m), voxel_index(point.y(), leaf_size_m),
                           voxel_index(point.z(), leaf_size_m)};
        VoxelAccum& accum = voxels[key];
        accum.sum += point.cast<double>();
        ++accum.count;
    }

    PointCloud downsampled;
    downsampled.points.reserve(voxels.size());
    for (const auto& entry : voxels) {
        const Eigen::Vector3d mean = entry.second.sum / static_cast<double>(entry.second.count);
        downsampled.points.push_back(mean.cast<float>());
    }
    return downsampled;
}

PointCloud transform_points(const PointCloud& cloud, const Eigen::Isometry3d& transform) {
    PointCloud transformed;
    transformed.points.reserve(cloud.points.size());
    for (const Eigen::Vector3f& point : cloud.points) {
        transformed.points.push_back((transform * point.cast<double>()).cast<float>());
    }
    return transformed;
}

PointCloud crop_by_fov(const PointCloud& cloud, double horizontal_deg, double vertical_deg,
                       double center_yaw_deg) {
    const bool full_horizontal = horizontal_deg >= 360.0;
    const bool full_vertical = vertical_deg >= 180.0;
    if ((full_horizontal && full_vertical) || cloud.points.empty()) {
        return cloud;
    }

    constexpr double kPi = 3.14159265358979323846;
    constexpr double kDegToRad = kPi / 180.0;
    // 张角的一半：把「总开口角」换算成「偏离中心的最大角差」。
    const double half_horizontal = 0.5 * std::clamp(horizontal_deg, 0.0, 360.0) * kDegToRad;
    const double half_vertical = 0.5 * std::clamp(vertical_deg, 0.0, 180.0) * kDegToRad;
    // 中心方位归一化到 (-pi, pi]，与 atan2 的值域对齐，免得跨 ±180 度时判反。
    double center = std::fmod(center_yaw_deg * kDegToRad, 2.0 * kPi);
    if (center > kPi) {
        center -= 2.0 * kPi;
    } else if (center <= -kPi) {
        center += 2.0 * kPi;
    }

    PointCloud cropped;
    cropped.points.reserve(cloud.points.size());
    for (const Eigen::Vector3f& point : cloud.points) {
        if (!point.allFinite()) {
            continue;
        }
        const double planar_range = std::sqrt(static_cast<double>(point.x()) * point.x() +
                                              static_cast<double>(point.y()) * point.y());
        // 竖直方向（±z 轴附近）的点水平方位无定义，只按垂直张角判定。
        if (planar_range > 1e-6) {
            double yaw = std::atan2(static_cast<double>(point.y()), static_cast<double>(point.x()));
            double delta = yaw - center;
            if (delta > kPi) {
                delta -= 2.0 * kPi;
            } else if (delta <= -kPi) {
                delta += 2.0 * kPi;
            }
            if (std::abs(delta) > half_horizontal) {
                continue;
            }
        }
        const double pitch = std::atan2(static_cast<double>(point.z()), planar_range);
        if (std::abs(pitch) > half_vertical) {
            continue;
        }
        cropped.points.push_back(point);
    }
    return cropped;
}

} // namespace diesel::localization_map
