// Copyright 2026 ZQU-Foray
//
// 先验地图：3D PCD + 2D 栅格 + manifest 三者绑定。任何几何参数不一致
// 都必须在加载期失败，而不是让配准在错误的地图上“成功”。

#ifndef FORAY_LOCALIZATION_MAP_PRIOR_MAP_HPP_
#define FORAY_LOCALIZATION_MAP_PRIOR_MAP_HPP_

#include "foray/localization/manifest.hpp"
#include "foray/localization_map/grid_map.hpp"
#include "foray/localization_map/pcd_loader.hpp"
#include "foray/localization_map/point_cloud.hpp"

#include <filesystem>
#include <string>

namespace foray::localization_map {

struct PriorMapLoadResult {
    bool ok{false};
    std::string error;
    /// 3D 地图里被跳过的行数（见 `PcdLoadResult::skipped_lines`）。非 0 时调用方必须告警。
    std::size_t skipped_pcd_lines{0};
};

/// 已加载并校验的先验地图。字段可直接被配准器与地图服务使用。
struct PriorMap {
    foray::localization::MapManifest manifest;
    PointCloud cloud;
    GridMap grid;

    bool empty() const { return cloud.empty(); }
};

/// 从 manifest 加载整张地图。`verify_hashes=false` 仅用于构建/调试工具；
/// 运行期必须为 true（默认）。
PriorMapLoadResult load_prior_map(const std::filesystem::path& manifest_path, PriorMap* map,
                                  bool verify_hashes = true);

} // namespace foray::localization_map

#endif // FORAY_LOCALIZATION_MAP_PRIOR_MAP_HPP_
