// Copyright 2026 ZQU-Foray
//
// 2D 占据栅格读取与查询。2D 地图由同一份 3D 点云投影生成（decision.md D4），
// 几何参数以 manifest 为准，不在 PGM 里重复记录。

#ifndef DIESEL_LOCALIZATION_MAP_GRID_MAP_HPP_
#define DIESEL_LOCALIZATION_MAP_GRID_MAP_HPP_

#include <Eigen/Core>

#include <array>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace diesel::localization_map {

struct PgmImage {
    int width{0};
    int height{0};
    int max_value{0};
    /// 行优先，第一行是图像顶部（与 ROS map_server 的 PGM 约定一致）。
    std::vector<std::uint8_t> pixels;
};

struct PgmLoadResult {
    bool ok{false};
    PgmImage image;
    std::string error;
};

PgmLoadResult load_pgm(const std::filesystem::path& path);

/// 占据值：0 空闲，100 占据，-1 未知。与 nav_msgs/OccupancyGrid 的语义一致。
struct GridMap {
    int width{0};
    int height{0};
    double resolution{0.0};
    Eigen::Vector2d origin{Eigen::Vector2d::Zero()};
    /// 栅格原点绕 +Z 的旋转（弧度），来自 manifest.origin[2]。
    double yaw{0.0};
    std::vector<std::int8_t> cells;

    /// 世界坐标 -> 栅格索引；越界返回 false。
    bool world_to_cell(double x, double y, int* ix, int* iy) const;

    /// 返回 0..100 或 -1（越界/未知）。
    std::int8_t occupancy_at(double x, double y) const;

    bool is_occupied(double x, double y) const;
};

/// 由 PGM 与 manifest 的几何参数构造栅格，并按 map_server 的 trinary 规则
/// 把灰度映射为占据值（0.65/0.196 阈值，见 decision.md D4）。
GridMap grid_from_pgm(const PgmImage& image, double resolution,
                      const std::array<double, 3>& origin);

} // namespace diesel::localization_map

#endif // DIESEL_LOCALIZATION_MAP_GRID_MAP_HPP_
