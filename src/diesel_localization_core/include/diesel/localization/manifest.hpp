// Copyright 2026 ZQU-Foray
//
// 地图 manifest 的解析与强校验。3D PCD、2D 栅格、坐标系与原点由同一个
// manifest 绑定，启动时校验失败必须拒绝启动（地图/参数错配是安全风险）。

#ifndef DIESEL_LOCALIZATION_MANIFEST_HPP_
#define DIESEL_LOCALIZATION_MANIFEST_HPP_

#include <array>
#include <filesystem>
#include <string>

namespace diesel::localization {

/// manifest.yaml 的强类型视图。
struct MapManifest {
    std::string version;
    std::string frame_id{"map"};
    /// 2D 栅格左下角在 map 系的位置 [x, y, yaw]。
    std::array<double, 3> origin{0.0, 0.0, 0.0};
    /// 米/像素。
    double resolution{0.0};
    int width{0};
    int height{0};
    std::string pcd_file;
    std::string grid_file;
    std::string pcd_sha256;
    std::string grid_sha256;
    std::string created_at;
    std::string source;
    std::filesystem::path manifest_path;
    std::filesystem::path directory;

    std::filesystem::path pcd_path() const;
    std::filesystem::path grid_path() const;
};

/// 加载结果。失败时 `ok=false`，`error` 包含可直接打印的中文诊断。
struct ManifestLoadResult {
    bool ok{false};
    MapManifest manifest;
    std::string error;
};

/// 解析并校验 manifest。默认校验 PCD/栅格存在且 sha256 匹配。
ManifestLoadResult load_map_manifest(const std::filesystem::path& manifest_path,
                                     bool verify_hashes = true);

/// 仅校验 manifest 引用文件的 sha256；成功返回 true。
bool verify_manifest_files(const MapManifest& manifest, std::string* error);

/// 校验字符串是 64 位十六进制 sha256。
bool is_sha256_hex(const std::string& value);

} // namespace diesel::localization

#endif // DIESEL_LOCALIZATION_MANIFEST_HPP_
