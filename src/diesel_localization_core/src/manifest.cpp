// Copyright 2026 ZQU-Foray

#include "diesel/localization/manifest.hpp"

#include "diesel/localization/sha256.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>

namespace diesel::localization {
namespace {

std::string to_lower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return value;
}

bool is_safe_relative_file(const std::string& value) {
    if (value.empty()) {
        return false;
    }
    const std::filesystem::path path(value);
    if (path.is_absolute()) {
        return false;
    }
    for (const auto& component : path) {
        if (component == "..") {
            return false;
        }
    }
    return true;
}

bool read_required_scalar(const YAML::Node& node, const char* key, std::string* out,
                          std::string* error) {
    const YAML::Node value = node[key];
    if (!value || !value.IsScalar()) {
        *error = std::string("manifest 缺少字符串字段: ") + key;
        return false;
    }
    try {
        *out = value.as<std::string>();
    } catch (const std::exception& exception) {
        *error =
            std::string("manifest 字段无法解析为字符串: ") + key + " (" + exception.what() + ")";
        return false;
    }
    if (out->empty()) {
        *error = std::string("manifest 字段为空: ") + key;
        return false;
    }
    return true;
}

bool read_origin(const YAML::Node& node, std::array<double, 3>* origin, std::string* error) {
    const YAML::Node value = node["origin"];
    if (!value || !value.IsSequence() || value.size() != 3) {
        *error = "manifest.origin 必须是 [x, y, yaw] 三个数";
        return false;
    }
    for (std::size_t index = 0; index < 3; ++index) {
        try {
            (*origin)[index] = value[index].as<double>();
        } catch (const std::exception& exception) {
            *error = "manifest.origin 含非数值元素: " + std::string(exception.what());
            return false;
        }
        if (!std::isfinite((*origin)[index])) {
            *error = "manifest.origin 含非有限数值";
            return false;
        }
    }
    return true;
}

bool read_positive_int(const YAML::Node& node, const char* key, int* out, std::string* error) {
    const YAML::Node value = node[key];
    if (!value || !value.IsScalar()) {
        *error = std::string("manifest 缺少整数字段: ") + key;
        return false;
    }
    try {
        *out = value.as<int>();
    } catch (const std::exception& exception) {
        *error = std::string("manifest 字段无法解析为整数: ") + key + " (" + exception.what() + ")";
        return false;
    }
    if (*out <= 0 || *out > 100000) {
        *error = std::string("manifest 字段超出合理范围: ") + key;
        return false;
    }
    return true;
}

std::string normalize_hash(const std::string& value) {
    return to_lower(value);
}

} // namespace

std::filesystem::path MapManifest::pcd_path() const {
    return directory / pcd_file;
}

std::filesystem::path MapManifest::grid_path() const {
    return directory / grid_file;
}

bool is_sha256_hex(const std::string& value) {
    if (value.size() != 64) {
        return false;
    }
    return std::all_of(value.begin(), value.end(),
                       [](unsigned char character) { return std::isxdigit(character) != 0; });
}

bool verify_manifest_files(const MapManifest& manifest, std::string* error) {
    const auto fail = [error](const std::string& message) {
        if (error != nullptr) {
            *error = message;
        }
        return false;
    };

    const std::filesystem::path pcd = manifest.pcd_path();
    const std::filesystem::path grid = manifest.grid_path();

    std::error_code code;
    if (!std::filesystem::is_regular_file(pcd, code)) {
        return fail("manifest 指向的点云文件不存在: " + pcd.string());
    }
    if (!std::filesystem::is_regular_file(grid, code)) {
        return fail("manifest 指向的栅格文件不存在: " + grid.string());
    }

    std::string hash_error;
    const std::string pcd_hash = sha256_file(pcd, &hash_error);
    if (pcd_hash.empty()) {
        return fail(hash_error);
    }
    if (normalize_hash(pcd_hash) != normalize_hash(manifest.pcd_sha256)) {
        return fail("点云 sha256 不匹配（地图被改动或 manifest 过期）: " + pcd.string() + " 期望 " +
                    manifest.pcd_sha256 + " 实际 " + pcd_hash);
    }

    const std::string grid_hash = sha256_file(grid, &hash_error);
    if (grid_hash.empty()) {
        return fail(hash_error);
    }
    if (normalize_hash(grid_hash) != normalize_hash(manifest.grid_sha256)) {
        return fail("栅格 sha256 不匹配（地图被改动或 manifest 过期）: " + grid.string() +
                    " 期望 " + manifest.grid_sha256 + " 实际 " + grid_hash);
    }

    if (error != nullptr) {
        error->clear();
    }
    return true;
}

ManifestLoadResult load_map_manifest(const std::filesystem::path& manifest_path,
                                     bool verify_hashes) {
    ManifestLoadResult result;
    result.manifest.manifest_path = manifest_path;
    result.manifest.directory = manifest_path.parent_path();

    std::error_code code;
    if (!std::filesystem::is_regular_file(manifest_path, code)) {
        result.error = "manifest 文件不存在: " + manifest_path.string();
        return result;
    }

    YAML::Node root;
    try {
        root = YAML::LoadFile(manifest_path.string());
    } catch (const std::exception& exception) {
        result.error =
            "manifest 无法解析: " + manifest_path.string() + " (" + exception.what() + ")";
        return result;
    }
    if (!root.IsMap()) {
        result.error = "manifest 根节点必须是映射: " + manifest_path.string();
        return result;
    }

    MapManifest& manifest = result.manifest;
    std::string error;
    if (!read_required_scalar(root, "version", &manifest.version, &error) ||
        !read_required_scalar(root, "frame_id", &manifest.frame_id, &error) ||
        !read_origin(root, &manifest.origin, &error) ||
        !read_positive_int(root, "width", &manifest.width, &error) ||
        !read_positive_int(root, "height", &manifest.height, &error) ||
        !read_required_scalar(root, "pcd_file", &manifest.pcd_file, &error) ||
        !read_required_scalar(root, "grid_file", &manifest.grid_file, &error) ||
        !read_required_scalar(root, "created_at", &manifest.created_at, &error)) {
        result.error = error;
        return result;
    }

    // source 允许缺省（旧 manifest），存在时必须是字符串。
    const YAML::Node source = root["source"];
    if (source) {
        if (!source.IsScalar()) {
            result.error = "manifest.source 必须是字符串";
            return result;
        }
        manifest.source = source.as<std::string>();
    }

    const YAML::Node resolution = root["resolution"];
    if (!resolution || !resolution.IsScalar()) {
        result.error = "manifest 缺少 resolution";
        return result;
    }
    try {
        manifest.resolution = resolution.as<double>();
    } catch (const std::exception& exception) {
        result.error = "manifest.resolution 无法解析为数值 (" + std::string(exception.what()) + ")";
        return result;
    }

    const YAML::Node pcd_sha = root["pcd_sha256"];
    const YAML::Node grid_sha = root["grid_sha256"];
    if (!pcd_sha || !pcd_sha.IsScalar() || !grid_sha || !grid_sha.IsScalar()) {
        result.error = "manifest 缺少 pcd_sha256 或 grid_sha256";
        return result;
    }
    manifest.pcd_sha256 = pcd_sha.as<std::string>();
    manifest.grid_sha256 = grid_sha.as<std::string>();

    if (manifest.frame_id != "map") {
        result.error = "manifest.frame_id 必须是 map，实际: " + manifest.frame_id;
        return result;
    }
    if (!std::isfinite(manifest.resolution) || manifest.resolution <= 0.0 ||
        manifest.resolution > 1.0) {
        result.error = "manifest.resolution 必须在 (0, 1.0] 米/像素之间";
        return result;
    }
    if (!is_safe_relative_file(manifest.pcd_file) || !is_safe_relative_file(manifest.grid_file)) {
        result.error = "manifest.pcd_file/grid_file 必须是包内相对路径且不得包含 ..";
        return result;
    }
    if (!is_sha256_hex(manifest.pcd_sha256) || !is_sha256_hex(manifest.grid_sha256)) {
        result.error = "manifest 的 pcd_sha256/grid_sha256 必须是小写或大写的 64 位十六进制";
        return result;
    }

    if (verify_hashes && !verify_manifest_files(manifest, &error)) {
        result.error = error;
        return result;
    }

    result.ok = true;
    return result;
}

} // namespace diesel::localization
