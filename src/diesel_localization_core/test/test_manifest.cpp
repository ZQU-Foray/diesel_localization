// Copyright 2026 ZQU-Foray

#include "diesel/localization/manifest.hpp"

#include "diesel/localization/sha256.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

namespace diesel::localization {
namespace {

constexpr const char* kPcdPayload = "fake-pcd-payload";
constexpr const char* kGridPayload = "fake-pgm-payload";

std::string hash_of(const std::string& payload) {
    return sha256_hex(payload.data(), payload.size());
}

void write_text(const std::filesystem::path& path, const std::string& content) {
    std::ofstream stream(path, std::ios::binary);
    ASSERT_TRUE(stream.good());
    stream.write(content.data(), static_cast<std::streamsize>(content.size()));
}

std::string manifest_text(const std::string& pcd_hash, const std::string& grid_hash,
                          const std::string& frame_id = "map",
                          const std::string& pcd_file = "map.pcd",
                          const std::string& grid_file = "map.pgm") {
    return "version: \"2027.1\"\n"
           "frame_id: " +
           frame_id +
           "\n"
           "origin: [-6.0, -4.0, 0.0]\n"
           "resolution: 0.05\n"
           "width: 240\n"
           "height: 160\n"
           "pcd_file: " +
           pcd_file +
           "\n"
           "grid_file: " +
           grid_file +
           "\n"
           "pcd_sha256: " +
           pcd_hash +
           "\n"
           "grid_sha256: " +
           grid_hash +
           "\n"
           "created_at: \"2026-10-25T00:00:00Z\"\n"
           "source: sim\n";
}

class ManifestTest : public ::testing::Test {
  protected:
    void SetUp() override {
        directory_ = std::filesystem::temp_directory_path() /
                     ("diesel_manifest_test_" +
                      std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        std::filesystem::remove_all(directory_);
        std::filesystem::create_directories(directory_);
        write_text(directory_ / "map.pcd", kPcdPayload);
        write_text(directory_ / "map.pgm", kGridPayload);
    }

    void TearDown() override { std::filesystem::remove_all(directory_); }

    std::filesystem::path write_manifest(const std::string& pcd_hash, const std::string& grid_hash,
                                         const std::string& frame_id = "map",
                                         const std::string& pcd_file = "map.pcd") {
        const std::filesystem::path path = directory_ / "manifest.yaml";
        write_text(path, manifest_text(pcd_hash, grid_hash, frame_id, pcd_file, "map.pgm"));
        return path;
    }

    std::filesystem::path directory_;
};

TEST_F(ManifestTest, LoadsValidManifest) {
    const std::filesystem::path path = write_manifest(hash_of(kPcdPayload), hash_of(kGridPayload));

    const ManifestLoadResult result = load_map_manifest(path);
    ASSERT_TRUE(result.ok) << result.error;
    EXPECT_EQ(result.manifest.version, "2027.1");
    EXPECT_EQ(result.manifest.frame_id, "map");
    EXPECT_EQ(result.manifest.width, 240);
    EXPECT_EQ(result.manifest.height, 160);
    EXPECT_DOUBLE_EQ(result.manifest.resolution, 0.05);
    EXPECT_DOUBLE_EQ(result.manifest.origin[0], -6.0);
    EXPECT_EQ(result.manifest.pcd_path(), directory_ / "map.pcd");
    EXPECT_EQ(result.manifest.grid_path(), directory_ / "map.pgm");
}

TEST_F(ManifestTest, RejectsHashMismatch) {
    const std::filesystem::path path = write_manifest(hash_of("other"), hash_of(kGridPayload));

    const ManifestLoadResult result = load_map_manifest(path);
    EXPECT_FALSE(result.ok);
    EXPECT_NE(result.error.find("sha256"), std::string::npos);
}

TEST_F(ManifestTest, RejectsWrongFrame) {
    const std::filesystem::path path =
        write_manifest(hash_of(kPcdPayload), hash_of(kGridPayload), "odom");

    const ManifestLoadResult result = load_map_manifest(path);
    EXPECT_FALSE(result.ok);
    EXPECT_NE(result.error.find("frame_id"), std::string::npos);
}

TEST_F(ManifestTest, RejectsPathTraversal) {
    const std::filesystem::path path =
        write_manifest(hash_of(kPcdPayload), hash_of(kGridPayload), "map", "../map.pcd");

    const ManifestLoadResult result = load_map_manifest(path);
    EXPECT_FALSE(result.ok);
}

TEST_F(ManifestTest, CanSkipHashVerification) {
    const std::filesystem::path path = write_manifest(hash_of("other"), hash_of(kGridPayload));

    const ManifestLoadResult result = load_map_manifest(path, false);
    ASSERT_TRUE(result.ok) << result.error;
}

TEST_F(ManifestTest, RejectsMissingReferencedFile) {
    std::filesystem::remove(directory_ / "map.pcd");
    const std::filesystem::path path = write_manifest(hash_of(kPcdPayload), hash_of(kGridPayload));

    const ManifestLoadResult result = load_map_manifest(path);
    EXPECT_FALSE(result.ok);
    EXPECT_NE(result.error.find("不存在"), std::string::npos);
}

TEST(ManifestHashFormat, AcceptsOnlySixtyFourHexDigits) {
    EXPECT_TRUE(is_sha256_hex(std::string(64, 'a')));
    EXPECT_TRUE(is_sha256_hex("0123456789abcdef0123456789abcdef0123456789abcdef0123456789ABCDEF"));
    EXPECT_FALSE(is_sha256_hex(std::string(63, 'a')));
    EXPECT_FALSE(is_sha256_hex(std::string(64, 'g')));
}

} // namespace
} // namespace diesel::localization
