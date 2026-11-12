// Copyright 2026 ZQU-Foray
//
// 最小 PCD 读取器（ascii / binary，uncompressed）。不依赖 PCL，
// 以便在没有 PCL 的构建环境与 CI 中仍能加载先验地图。

#ifndef DIESEL_LOCALIZATION_MAP_PCD_LOADER_HPP_
#define DIESEL_LOCALIZATION_MAP_PCD_LOADER_HPP_

#include "diesel/localization_map/point_cloud.hpp"

#include <filesystem>
#include <string>

namespace diesel::localization_map {

struct PcdLoadResult {
    bool ok{false};
    PointCloud cloud;
    std::string error;
    /// ascii 段里被丢弃的行数：字段数不足的行，以及 x/y/z 非有限（NaN/inf）的点。
    ///
    /// 这两类行不会进 `cloud`，加载也仍然成功——但「地图比声明少几个点」必须是**可见事实**：
    /// 静默丢点会让 `plan.md` 的点数统计与实测对不上，也会让「地图里有空洞」这种真正的
    /// 数据问题一直没人发现（审计 L18，`decision.md` D54）。调用方必须把非 0 值报出来。
    std::size_t skipped_lines{0};
};

/// 读取 PCD 文件。支持 FIELDS 含 x/y/z 的 ascii 与 binary 数据；
/// `binary_compressed` 明确报错（地图构建工具应导出未压缩格式）。
PcdLoadResult load_pcd(const std::filesystem::path& path);

} // namespace diesel::localization_map

#endif // DIESEL_LOCALIZATION_MAP_PCD_LOADER_HPP_
