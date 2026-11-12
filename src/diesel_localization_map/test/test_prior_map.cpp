// Copyright 2026 ZQU-Foray

#include "diesel/localization_map/prior_map.hpp"

#include "diesel/localization/sha256.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;
using diesel::localization::sha256_file;
using diesel::localization_map::load_prior_map;
using diesel::localization_map::PriorMap;

struct TempDir {
    fs::path path;

    TempDir() {
        const auto suffix = std::chrono::steady_clock::now().time_since_epoch().count();
        path = fs::temp_directory_path() / ("diesel_prior_test_" + std::to_string(suffix));
        fs::create_directories(path);
    }

    ~TempDir() {
        std::error_code code;
        fs::remove_all(path, code);
    }
};

std::string make_pcd_text() {
    return "VERSION 0.7\n"
           "FIELDS x y z\n"
           "SIZE 4 4 4\n"
           "TYPE F F F\n"
           "COUNT 1 1 1\n"
           "WIDTH 3\n"
           "HEIGHT 1\n"
           "POINTS 3\n"
           "DATA ascii\n"
           "0 0 0\n"
           "1 0 0\n"
           "0 1 0\n";
}

std::string make_manifest(const std::string& pcd_sha, const std::string& grid_sha, int width,
                          int height) {
    return "version: 2027_3v3_test\n"
           "frame_id: map\n"
           "origin: [0.0, 0.0, 0.0]\n"
           "resolution: 0.5\n"
           "width: " +
           std::to_string(width) +
           "\n"
           "height: " +
           std::to_string(height) +
           "\n"
           "pcd_file: map.pcd\n"
           "grid_file: map.pgm\n"
           "pcd_sha256: " +
           pcd_sha +
           "\n"
           "grid_sha256: " +
           grid_sha +
           "\n"
           "created_at: 2026-10-25T00:00:00+08:00\n"
           "source: sim\n";
}

bool write_file(const fs::path& path, const std::string& content) {
    std::ofstream out(path, std::ios::binary);
    if (!out) {
        return false;
    }
    out.write(content.data(), static_cast<std::streamsize>(content.size()));
    return static_cast<bool>(out);
}

TEST(PriorMapTest, LoadsConsistentMap) {
    TempDir dir;
    ASSERT_TRUE(write_file(dir.path / "map.pcd", make_pcd_text()));
    const std::vector<unsigned char> pixels{0, 255, 255, 255};
    {
        std::ofstream out(dir.path / "map.pgm", std::ios::binary);
        out << "P5\n2 2\n255\n";
        out.write(reinterpret_cast<const char*>(pixels.data()), 4);
    }

    std::string error;
    const std::string pcd_sha = sha256_file(dir.path / "map.pcd", &error);
    ASSERT_FALSE(pcd_sha.empty()) << error;
    const std::string grid_sha = sha256_file(dir.path / "map.pgm", &error);
    ASSERT_FALSE(grid_sha.empty()) << error;
    ASSERT_TRUE(write_file(dir.path / "manifest.yaml", make_manifest(pcd_sha, grid_sha, 2, 2)));

    PriorMap map;
    const auto result = load_prior_map(dir.path / "manifest.yaml", &map, true);
    ASSERT_TRUE(result.ok) << result.error;
    EXPECT_EQ(map.cloud.size(), 3U);
    EXPECT_EQ(map.grid.width, 2);
    EXPECT_EQ(map.grid.height, 2);
    EXPECT_TRUE(map.grid.is_occupied(0.25, 0.75));
    EXPECT_EQ(map.manifest.frame_id, "map");
}

TEST(PriorMapTest, RejectsHashMismatch) {
    TempDir dir;
    ASSERT_TRUE(write_file(dir.path / "map.pcd", make_pcd_text()));
    {
        std::ofstream out(dir.path / "map.pgm", std::ios::binary);
        out << "P5\n2 2\n255\n";
        const std::vector<unsigned char> pixels{255, 255, 255, 255};
        out.write(reinterpret_cast<const char*>(pixels.data()), 4);
    }

    std::string error;
    const std::string pcd_sha = sha256_file(dir.path / "map.pcd", &error);
    const std::string grid_sha = sha256_file(dir.path / "map.pgm", &error);
    ASSERT_TRUE(write_file(dir.path / "manifest.yaml", make_manifest(pcd_sha, grid_sha, 2, 2)));

    // 地图被改动但 manifest 未更新：加载必须失败，而不是在错误地图上继续配准。
    ASSERT_TRUE(write_file(dir.path / "map.pcd", make_pcd_text() + "9 9 9\n"));
    PriorMap map;
    const auto result = load_prior_map(dir.path / "manifest.yaml", &map, true);
    EXPECT_FALSE(result.ok);
}

TEST(PriorMapTest, RejectsGridSizeMismatch) {
    TempDir dir;
    ASSERT_TRUE(write_file(dir.path / "map.pcd", make_pcd_text()));
    {
        std::ofstream out(dir.path / "map.pgm", std::ios::binary);
        out << "P5\n2 2\n255\n";
        const std::vector<unsigned char> pixels{255, 255, 255, 255};
        out.write(reinterpret_cast<const char*>(pixels.data()), 4);
    }

    std::string error;
    const std::string pcd_sha = sha256_file(dir.path / "map.pcd", &error);
    const std::string grid_sha = sha256_file(dir.path / "map.pgm", &error);
    // manifest 声明 3x3，实际 PGM 是 2x2。
    ASSERT_TRUE(write_file(dir.path / "manifest.yaml", make_manifest(pcd_sha, grid_sha, 3, 3)));

    PriorMap map;
    const auto result = load_prior_map(dir.path / "manifest.yaml", &map, true);
    EXPECT_FALSE(result.ok);
}

} // namespace
