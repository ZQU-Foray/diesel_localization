// Copyright 2026 ZQU-Foray

#include "diesel/localization_map/grid_map.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;
using diesel::localization_map::grid_from_pgm;
using diesel::localization_map::load_pgm;
using diesel::localization_map::PgmImage;

struct TempDir {
    fs::path path;

    TempDir() {
        const auto suffix = std::chrono::steady_clock::now().time_since_epoch().count();
        path = fs::temp_directory_path() / ("diesel_grid_test_" + std::to_string(suffix));
        fs::create_directories(path);
    }

    ~TempDir() {
        std::error_code code;
        fs::remove_all(path, code);
    }
};

TEST(GridMapTest, LoadsBinaryPgmAndFlipsRows) {
    TempDir dir;
    const fs::path file = dir.path / "map.pgm";
    const std::vector<unsigned char> pixels{0, 255, 255, 255};
    {
        std::ofstream out(file, std::ios::binary);
        out << "P5\n2 2\n255\n";
        out.write(reinterpret_cast<const char*>(pixels.data()), 4);
    }

    const auto loaded = load_pgm(file);
    ASSERT_TRUE(loaded.ok) << loaded.error;
    ASSERT_EQ(loaded.image.width, 2);
    ASSERT_EQ(loaded.image.height, 2);

    const auto grid = grid_from_pgm(loaded.image, 0.5, {0.0, 0.0, 0.0});
    // PGM 顶行是图像顶部；地图 y 向上，所以顶行对应 y 较大的栅格行。
    EXPECT_EQ(grid.occupancy_at(0.25, 0.75), 100);
    EXPECT_EQ(grid.occupancy_at(0.25, 0.25), 0);
    EXPECT_EQ(grid.occupancy_at(-1.0, 0.0), -1);
}

TEST(GridMapTest, LoadsAsciiPgm) {
    TempDir dir;
    const fs::path file = dir.path / "map.pgm";
    {
        std::ofstream out(file);
        out << "P2\n# comment\n2 1\n255\n255 0\n";
    }

    const auto loaded = load_pgm(file);
    ASSERT_TRUE(loaded.ok) << loaded.error;
    ASSERT_EQ(loaded.image.width, 2);
    ASSERT_EQ(loaded.image.height, 1);
    EXPECT_EQ(loaded.image.pixels[0], 255);
    EXPECT_EQ(loaded.image.pixels[1], 0);
}

TEST(GridMapTest, UnknownValuesStayUnknown) {
    PgmImage image;
    image.width = 2;
    image.height = 1;
    image.max_value = 255;
    image.pixels = {128, 0};

    const auto grid = grid_from_pgm(image, 1.0, {0.0, 0.0, 0.0});
    EXPECT_EQ(grid.occupancy_at(0.5, 0.5), -1);
    EXPECT_EQ(grid.occupancy_at(1.5, 0.5), 100);
}

TEST(GridMapTest, AppliesOriginOffset) {
    PgmImage image;
    image.width = 1;
    image.height = 1;
    image.max_value = 255;
    image.pixels = {0};

    const auto grid = grid_from_pgm(image, 1.0, {10.0, 20.0, 0.0});
    EXPECT_TRUE(grid.is_occupied(10.5, 20.5));
    EXPECT_FALSE(grid.is_occupied(0.5, 0.5));
}

TEST(GridMapTest, AppliesYaw) {
    PgmImage image;
    image.width = 2;
    image.height = 1;
    image.max_value = 255;
    image.pixels = {0, 255};

    const double half_pi = 1.5707963267948966;
    const auto grid = grid_from_pgm(image, 1.0, {0.0, 0.0, half_pi});
    // 栅格 +X 轴旋转 90° 后指向世界 +Y：cell(0,0) 覆盖世界 x∈[-1,0), y∈[0,1)。
    EXPECT_TRUE(grid.is_occupied(-0.5, 0.5));
    EXPECT_EQ(grid.occupancy_at(0.0, 1.0), 0);
}

} // namespace
