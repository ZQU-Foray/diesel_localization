// Copyright 2026 ZQU-Foray

#include "foray/localization_map/pcd_loader.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;

struct TempDir {
    fs::path path;

    TempDir() {
        const auto suffix = std::chrono::steady_clock::now().time_since_epoch().count();
        path = fs::temp_directory_path() / ("foray_pcd_test_" + std::to_string(suffix));
        fs::create_directories(path);
    }

    ~TempDir() {
        std::error_code code;
        fs::remove_all(path, code);
    }
};

const char* kAsciiHeader = "# .PCD v0.7\n"
                           "VERSION 0.7\n"
                           "FIELDS x y z\n"
                           "SIZE 4 4 4\n"
                           "TYPE F F F\n"
                           "COUNT 1 1 1\n"
                           "WIDTH 3\n"
                           "HEIGHT 1\n"
                           "POINTS 3\n"
                           "DATA ascii\n";

TEST(PcdLoaderTest, LoadsAsciiCloud) {
    TempDir dir;
    const fs::path file = dir.path / "map.pcd";
    {
        std::ofstream out(file);
        out << kAsciiHeader;
        out << "1.0 2.0 3.0\n";
        out << "-1.5 0.0 0.5\n";
        out << "0.25 0.5 0.75\n";
    }

    const auto result = foray::localization_map::load_pcd(file);
    ASSERT_TRUE(result.ok) << result.error;
    ASSERT_EQ(result.cloud.size(), 3U);
    EXPECT_FLOAT_EQ(result.cloud.points[0].x(), 1.0F);
    EXPECT_FLOAT_EQ(result.cloud.points[1].x(), -1.5F);
    EXPECT_FLOAT_EQ(result.cloud.points[2].z(), 0.75F);
}

TEST(PcdLoaderTest, LoadsBinaryCloud) {
    TempDir dir;
    const fs::path file = dir.path / "map.pcd";
    const std::string header = "VERSION 0.7\n"
                               "FIELDS x y z\n"
                               "SIZE 4 4 4\n"
                               "TYPE F F F\n"
                               "COUNT 1 1 1\n"
                               "WIDTH 2\n"
                               "HEIGHT 1\n"
                               "POINTS 2\n"
                               "DATA binary\n";
    const std::vector<float> points{0.5F, 1.5F, 2.5F, -0.5F, -1.5F, -2.5F};
    {
        std::ofstream out(file, std::ios::binary);
        out.write(header.data(), static_cast<std::streamsize>(header.size()));
        out.write(reinterpret_cast<const char*>(points.data()),
                  static_cast<std::streamsize>(points.size() * sizeof(float)));
    }

    const auto result = foray::localization_map::load_pcd(file);
    ASSERT_TRUE(result.ok) << result.error;
    ASSERT_EQ(result.cloud.size(), 2U);
    EXPECT_FLOAT_EQ(result.cloud.points[1].x(), -0.5F);
    EXPECT_FLOAT_EQ(result.cloud.points[1].z(), -2.5F);
}

TEST(PcdLoaderTest, RejectsCompressedAndMissingFile) {
    TempDir dir;
    const fs::path compressed = dir.path / "compressed.pcd";
    {
        std::ofstream out(compressed);
        out << "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
               "POINTS 1\nDATA binary_compressed\n";
    }
    const auto compressed_result = foray::localization_map::load_pcd(compressed);
    EXPECT_FALSE(compressed_result.ok);
    EXPECT_NE(compressed_result.error.find("binary_compressed"), std::string::npos);

    const auto missing_result = foray::localization_map::load_pcd(dir.path / "missing.pcd");
    EXPECT_FALSE(missing_result.ok);
}

TEST(PcdLoaderTest, SkipsInvalidPoints) {
    TempDir dir;
    const fs::path file = dir.path / "nan.pcd";
    {
        std::ofstream out(file);
        out << kAsciiHeader;
        out << "nan 0 0\n";
        out << "1 1 1\n";
        out << "2 2 2\n";
    }

    const auto result = foray::localization_map::load_pcd(file);
    ASSERT_TRUE(result.ok) << result.error;
    // nan 行被跳过，剩下两行有效点。
    EXPECT_EQ(result.cloud.size(), 2U);
    // 审计 L18：父提交在这里只是 `continue`，丢了 1 行却没有任何出口能说出来。
    // 现在跳过数必须被计数——「地图比声明少点」是操作者需要知道的事实。
    printf("L18 回执(NaN 行): ok=%d 装载=%zu 点 跳过=%zu 行\n", static_cast<int>(result.ok),
           result.cloud.size(), result.skipped_lines);
    EXPECT_EQ(result.skipped_lines, 1U);
}

// L10 用例共用的头部拼装助手（FIELDS x y z 固定）。
std::string header_with(const std::string& size_line, const std::string& count_line,
                        const std::string& points_line, const std::string& data_line) {
    return "VERSION 0.7\nFIELDS x y z\n" + size_line + "TYPE F F F\n" + count_line +
           "WIDTH 3\nHEIGHT 1\n" + points_line + data_line;
}

TEST(PcdLoaderTest, RejectsDeclaredPointCountAboveLimit) {
    TempDir dir;
    const fs::path file = dir.path / "huge.pcd";
    {
        std::ofstream out(file);
        out << header_with("SIZE 4 4 4\n", "COUNT 1 1 1\n", "POINTS 2000000000\n", "DATA ascii\n");
        out << "0 0 0\n";
    }

    const auto result = foray::localization_map::load_pcd(file);
    EXPECT_FALSE(result.ok);
    EXPECT_NE(result.error.find("上限"), std::string::npos) << result.error;
}

TEST(PcdLoaderTest, RejectsNonNumericFieldSize) {
    TempDir dir;
    const fs::path file = dir.path / "bad_size.pcd";
    {
        std::ofstream out(file);
        out << header_with("SIZE abc 4 4\n", "COUNT 1 1 1\n", "POINTS 1\n", "DATA ascii\n");
        out << "0 0 0\n";
    }

    const auto result = foray::localization_map::load_pcd(file);
    EXPECT_FALSE(result.ok);
    EXPECT_NE(result.error.find("SIZE"), std::string::npos) << result.error;
}

TEST(PcdLoaderTest, CountsRowsWithMissingFields) {
    // 字段数不足的行（导出工具写坏了格式、或者夹杂注释）同样是丢点，同样要计数；
    // 但它不能被当成「文件到这就没了」——`index` 不前进，所以声明点数与内容不符时
    // 依然会报「数据长度不足」。
    TempDir dir;
    const fs::path file = dir.path / "short_rows.pcd";
    {
        std::ofstream out(file);
        out << header_with("SIZE 4 4 4\n", "COUNT 1 1 1\n", "POINTS 2\n", "DATA ascii\n");
        out << "1 1 1\n";
        out << "2\n";
        out << "3 3 3\n";
    }

    const auto result = foray::localization_map::load_pcd(file);
    ASSERT_TRUE(result.ok) << result.error;
    EXPECT_EQ(result.cloud.size(), 2U);
    printf("L18 回执(字段数不足): ok=%d 装载=%zu 点 跳过=%zu 行\n", static_cast<int>(result.ok),
           result.cloud.size(), result.skipped_lines);
    EXPECT_EQ(result.skipped_lines, 1U);
}

// L10：PCD 头是不可信输入。下面四条分别覆盖「声明点数超上限」「字段尺寸畸形」
// 「ascii 声明点数与内容不符」「尺寸/偏移溢出」，共同判据只有一条：
// load_pcd 必须返回 ok=false 且带可读 error，而不是抛异常或按声明值分配内存。
TEST(PcdLoaderTest, RejectsTruncatedAsciiData) {
    TempDir dir;
    const fs::path file = dir.path / "truncated.pcd";
    {
        std::ofstream out(file);
        out << header_with("SIZE 4 4 4\n", "COUNT 1 1 1\n", "POINTS 1000\n", "DATA ascii\n");
        for (int i = 0; i < 10; ++i) {
            out << i << " 0 0\n";
        }
    }

    const auto result = foray::localization_map::load_pcd(file);
    EXPECT_FALSE(result.ok);
    EXPECT_NE(result.error.find("数据长度不足"), std::string::npos) << result.error;
}

TEST(PcdLoaderTest, RejectsOversizedFieldBytes) {
    TempDir dir;
    const fs::path file = dir.path / "field_overflow.pcd";
    {
        std::ofstream out(file);
        out << header_with("SIZE 2000000000 4 4\n", "COUNT 1 1 1\n", "POINTS 1\n", "DATA ascii\n");
        out << "0 0 0\n";
    }

    const auto result = foray::localization_map::load_pcd(file);
    EXPECT_FALSE(result.ok);
    EXPECT_NE(result.error.find("字段尺寸非法"), std::string::npos) << result.error;
}

} // namespace
