// Copyright 2026 ZQU-Foray

#include "foray/localization_map/prior_map.hpp"

#include "foray/localization/sha256.hpp"

#include <string>

namespace foray::localization_map {

PriorMapLoadResult load_prior_map(const std::filesystem::path& manifest_path, PriorMap* map,
                                  bool verify_hashes) {
    PriorMapLoadResult result;
    if (map == nullptr) {
        result.error = "PriorMap 输出指针为空";
        return result;
    }

    const foray::localization::ManifestLoadResult manifest_result =
        foray::localization::load_map_manifest(manifest_path, verify_hashes);
    if (!manifest_result.ok) {
        result.error = "manifest 校验失败: " + manifest_result.error;
        return result;
    }
    const foray::localization::MapManifest& manifest = manifest_result.manifest;

    const PcdLoadResult pcd = load_pcd(manifest.pcd_path());
    if (!pcd.ok) {
        result.error = "3D 地图加载失败: " + pcd.error;
        return result;
    }
    // 丢点不是错误，但也不能悄没声儿地过去（审计 L18）。
    result.skipped_pcd_lines = pcd.skipped_lines;

    const PgmLoadResult pgm = load_pgm(manifest.grid_path());
    if (!pgm.ok) {
        result.error = "2D 地图加载失败: " + pgm.error;
        return result;
    }

    // 2D 与 3D 必须同源、同分辨率、同原点；manifest 是唯一事实源。
    if (pgm.image.width != manifest.width || pgm.image.height != manifest.height) {
        result.error = "2D 栅格尺寸与 manifest 不一致: pgm=" + std::to_string(pgm.image.width) +
                       "x" + std::to_string(pgm.image.height) +
                       " manifest=" + std::to_string(manifest.width) + "x" +
                       std::to_string(manifest.height);
        return result;
    }

    map->manifest = manifest;
    map->cloud = pcd.cloud;
    map->grid = grid_from_pgm(pgm.image, manifest.resolution, manifest.origin);

    if (map->cloud.empty()) {
        result.error = "3D 地图为空";
        return result;
    }
    result.ok = true;
    return result;
}

} // namespace foray::localization_map
