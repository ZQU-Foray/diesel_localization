// Copyright 2026 ZQU-Foray
//
// 与 ROS 无关的点云容器与基础操作。地图与配准后端共用同一表示，
// 避免 PCL/ROS 类型泄漏到核心算法。

#ifndef DIESEL_LOCALIZATION_MAP_POINT_CLOUD_HPP_
#define DIESEL_LOCALIZATION_MAP_POINT_CLOUD_HPP_

#include <Eigen/Geometry>

#include <cstddef>
#include <vector>

namespace diesel::localization_map {

/// 以 float 存储的 XYZ 点云；不保留强度/环号等传感器字段（定位只用几何）。
struct PointCloud {
    std::vector<Eigen::Vector3f> points;

    bool empty() const { return points.empty(); }
    std::size_t size() const { return points.size(); }
};

/// 体素降采样：同一体素内取质心。`leaf_size_m <= 0` 时原样返回。
PointCloud voxel_downsample(const PointCloud& cloud, double leaf_size_m);

/// 按刚体变换逐点变换。
PointCloud transform_points(const PointCloud& cloud, const Eigen::Isometry3d& transform);

/// 按视场角裁切点云（传感器系，decision.md D23）。
///
/// `horizontal_deg` 取 (0, 360]、`vertical_deg` 取 (0, 180]，`center_yaw_deg` 是水平张角的
/// 中心方位（绕 +z，0 指向 +x）。水平 360 且垂直 180 时原样返回——这样「不裁切」与
/// 「裁到全视场」是同一个代码路径，默认配置下行为与改造前逐字节一致。
/// 视场是**传感器系下的锥形区域**，不是机体坐标系：先裁切再做 TF 变换，
/// 否则机体姿态一转，裁切方向就跟着转了。
PointCloud crop_by_fov(const PointCloud& cloud, double horizontal_deg, double vertical_deg,
                       double center_yaw_deg);

} // namespace diesel::localization_map

#endif // DIESEL_LOCALIZATION_MAP_POINT_CLOUD_HPP_
