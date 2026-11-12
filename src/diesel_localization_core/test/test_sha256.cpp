// Copyright 2026 ZQU-Foray

#include "diesel/localization/sha256.hpp"

#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

namespace diesel::localization {
namespace {

TEST(Sha256, MatchesKnownVectors) {
    const std::string abc = "abc";
    EXPECT_EQ(sha256_hex(abc.data(), abc.size()),
              "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    EXPECT_EQ(sha256_hex(nullptr, 0),
              "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");

    const std::string long_input(1000, 'a');
    EXPECT_EQ(sha256_hex(long_input.data(), long_input.size()),
              "41edece42d63e8d9bf515a9ba6932e1c20cbc9f5a5d134645adb5db1b9737ea3");
}

TEST(Sha256, HashesFileLikeBuffer) {
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "diesel_sha256_test.bin";
    const std::string payload = "diesel-localization";
    {
        std::ofstream stream(path, std::ios::binary);
        ASSERT_TRUE(stream.good());
        stream.write(payload.data(), static_cast<std::streamsize>(payload.size()));
    }

    std::string error;
    const std::string file_hash = sha256_file(path, &error);
    EXPECT_TRUE(error.empty());
    EXPECT_EQ(file_hash, sha256_hex(payload.data(), payload.size()));

    std::filesystem::remove(path);
}

TEST(Sha256, ReportsMissingFile) {
    std::string error;
    EXPECT_TRUE(sha256_file("/nonexistent/diesel_sha256_test.bin", &error).empty());
    EXPECT_FALSE(error.empty());
}

} // namespace
} // namespace diesel::localization
