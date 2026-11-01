// Copyright 2026 ZQU-Foray

#include "foray/localization_map/pcd_loader.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <new>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

namespace foray::localization_map {
namespace {

// 上限来源（audit/tasks/02-foray_localization.md L10，2026-10-31）：PCD 头是不可信输入，
// 一个 `POINTS 2000000000` 就能让 reserve 抛 bad_alloc 直接终止定位进程。20M 点在 0.25 m
// 体素下相当于 1.25 km² 场地，比现有赛场地图（< 3M 点）宽 6 倍；512 MiB 是 20M 点按
// 25 字节/点（x/y/z + intensity 等字段的最大排布）估算的两倍。
constexpr std::size_t kMaxPcdPoints = 20'000'000U;
constexpr std::uintmax_t kMaxPcdBytes = 512U * 1024U * 1024U;
// 单字段上限：8 字节类型配 COUNT 4 已是 PCL 常见排布的上界，再大只可能是畸形头。
constexpr int kMaxPcdFieldBytes = 32;
constexpr int kMaxPcdFieldCount = 64;

// 头部数值一律走不抛异常的解析：畸形头必须变成 result.error，而不是把异常抛给调用方。
bool parse_size_token(const std::string& token, std::size_t* value) {
    if (token.empty()) {
        return false;
    }
    std::size_t parsed_value = 0U;
    const char* begin = token.data();
    const char* end = begin + token.size();
    const auto parsed = std::from_chars(begin, end, parsed_value);
    if (parsed.ec != std::errc{} || parsed.ptr != end) {
        return false;
    }
    *value = parsed_value;
    return true;
}

bool parse_int_token(const std::string& token, int* value) {
    if (token.empty()) {
        return false;
    }
    int parsed_value = 0;
    const char* begin = token.data();
    const char* end = begin + token.size();
    const auto parsed = std::from_chars(begin, end, parsed_value);
    if (parsed.ec != std::errc{} || parsed.ptr != end) {
        return false;
    }
    *value = parsed_value;
    return true;
}

bool parse_float_token(const std::string& token, float* value) {
    try {
        std::size_t consumed = 0U;
        const float parsed_value = std::stof(token, &consumed);
        if (consumed != token.size()) {
            return false;
        }
        *value = parsed_value;
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

std::vector<std::string> split_tokens(const std::string& line) {
    std::vector<std::string> tokens;
    std::istringstream stream(line);
    std::string token;
    while (stream >> token) {
        tokens.push_back(token);
    }
    return tokens;
}

struct PcdHeader {
    std::vector<std::string> fields;
    std::vector<int> sizes;
    std::vector<std::string> types;
    std::vector<int> counts;
    std::size_t width{0};
    std::size_t height{0};
    std::size_t points{0};
    std::string data;
};

bool parse_header(std::ifstream& file, PcdHeader* header, std::string* error) {
    std::string line;
    while (std::getline(file, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        if (line.empty() || line[0] == '#') {
            continue;
        }
        const std::vector<std::string> tokens = split_tokens(line);
        if (tokens.empty()) {
            continue;
        }
        const std::string& key = tokens[0];
        if (key == "FIELDS") {
            header->fields.assign(tokens.begin() + 1, tokens.end());
        } else if (key == "SIZE") {
            header->sizes.clear();
            for (std::size_t i = 1; i < tokens.size(); ++i) {
                int size = 0;
                if (!parse_int_token(tokens[i], &size)) {
                    *error = "PCD 头 SIZE 不是整数: " + tokens[i];
                    return false;
                }
                header->sizes.push_back(size);
            }
        } else if (key == "TYPE") {
            header->types.assign(tokens.begin() + 1, tokens.end());
        } else if (key == "COUNT") {
            header->counts.clear();
            for (std::size_t i = 1; i < tokens.size(); ++i) {
                int count = 0;
                if (!parse_int_token(tokens[i], &count)) {
                    *error = "PCD 头 COUNT 不是整数: " + tokens[i];
                    return false;
                }
                header->counts.push_back(count);
            }
        } else if (key == "WIDTH" || key == "HEIGHT" || key == "POINTS") {
            std::size_t value = 0U;
            if (tokens.size() < 2 || !parse_size_token(tokens[1], &value)) {
                *error = "PCD 头 " + key + " 不是非负整数";
                return false;
            }
            if (key == "WIDTH") {
                header->width = value;
            } else if (key == "HEIGHT") {
                header->height = value;
            } else {
                header->points = value;
            }
        } else if (key == "DATA") {
            header->data = tokens.size() >= 2 ? tokens[1] : std::string();
            if (header->points == 0U) {
                header->points = header->width * header->height;
            }
            return true;
        }
    }
    *error = "PCD 头缺少 DATA 行";
    return false;
}

struct FieldLayout {
    int x_offset{-1};
    int y_offset{-1};
    int z_offset{-1};
    int x_size{0};
    int y_size{0};
    int z_size{0};
    std::size_t x_type_index{0};
    std::size_t y_type_index{0};
    std::size_t z_type_index{0};
    std::size_t stride{0};
};

bool field_layout(const PcdHeader& header, FieldLayout* layout, std::string* error) {
    const std::size_t count = header.fields.size();
    if (header.sizes.size() != count || header.types.size() != count ||
        header.counts.size() != count) {
        *error = "PCD 头 FIELDS/SIZE/TYPE/COUNT 数量不一致";
        return false;
    }

    // 字段尺寸/个数先夹在合理范围，再用 size_t 累加偏移：父提交用 int 累加
    // `sizes[i] * counts[i]`，一个 `SIZE 2000000000` 就能把偏移算成负数（L10）。
    std::size_t offset = 0U;
    for (std::size_t i = 0; i < count; ++i) {
        if (header.sizes[i] <= 0 || header.sizes[i] > kMaxPcdFieldBytes || header.counts[i] <= 0 ||
            header.counts[i] > kMaxPcdFieldCount) {
            *error = "PCD 头字段尺寸非法: SIZE " + std::to_string(header.sizes[i]) + " COUNT " +
                     std::to_string(header.counts[i]);
            return false;
        }
        const auto bytes =
            static_cast<std::size_t>(header.sizes[i]) * static_cast<std::size_t>(header.counts[i]);
        if (header.fields[i] == "x") {
            layout->x_offset = static_cast<int>(offset);
            layout->x_size = header.sizes[i];
            layout->x_type_index = i;
        } else if (header.fields[i] == "y") {
            layout->y_offset = static_cast<int>(offset);
            layout->y_size = header.sizes[i];
            layout->y_type_index = i;
        } else if (header.fields[i] == "z") {
            layout->z_offset = static_cast<int>(offset);
            layout->z_size = header.sizes[i];
            layout->z_type_index = i;
        }
        offset += bytes;
        if (offset > kMaxPcdBytes) {
            *error = "PCD 点步长超过上限（" + std::to_string(kMaxPcdBytes) + " 字节）";
            return false;
        }
    }
    layout->stride = offset;
    if (layout->x_offset < 0 || layout->y_offset < 0 || layout->z_offset < 0) {
        *error = "PCD 缺少 x/y/z 字段";
        return false;
    }
    return true;
}

bool read_numeric(const char* base, int offset, int size, const std::string& type, float* value) {
    if (type == "F" && size == 4) {
        float v = 0.0F;
        std::memcpy(&v, base + offset, sizeof(float));
        *value = v;
        return true;
    }
    if (type == "F" && size == 8) {
        double v = 0.0;
        std::memcpy(&v, base + offset, sizeof(double));
        *value = static_cast<float>(v);
        return true;
    }
    if (type == "I" && size == 2) {
        std::int16_t v = 0;
        std::memcpy(&v, base + offset, sizeof(v));
        *value = static_cast<float>(v);
        return true;
    }
    if (type == "I" && size == 4) {
        std::int32_t v = 0;
        std::memcpy(&v, base + offset, sizeof(v));
        *value = static_cast<float>(v);
        return true;
    }
    if (type == "U" && size == 2) {
        std::uint16_t v = 0;
        std::memcpy(&v, base + offset, sizeof(v));
        *value = static_cast<float>(v);
        return true;
    }
    return false;
}

} // namespace

PcdLoadResult load_pcd(const std::filesystem::path& path) {
    PcdLoadResult result;
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        result.error = "无法打开 PCD 文件: " + path.string();
        return result;
    }

    PcdHeader header;
    if (!parse_header(file, &header, &result.error)) {
        return result;
    }
    if (header.data != "ascii" && header.data != "binary") {
        result.error =
            "不支持的 PCD DATA 格式: " + header.data + "（仅支持 ascii/binary，请导出未压缩格式）";
        return result;
    }

    FieldLayout layout;
    if (!field_layout(header, &layout, &result.error)) {
        return result;
    }
    if (layout.stride == 0U) {
        result.error = "PCD 点步长为 0";
        return result;
    }

    const std::size_t point_count = header.points;
    if (point_count == 0U) {
        result.error = "PCD 点数声明为 0";
        return result;
    }
    if (point_count > kMaxPcdPoints) {
        result.error = "PCD 点数超过上限（声明 " + std::to_string(point_count) + "，上限 " +
                       std::to_string(kMaxPcdPoints) + "）";
        return result;
    }
    // 先按 uintmax_t 乘再比上限：父提交直接把 `stride * point_count` 交给 vector，
    // 32 位下会回绕成一个小值，先把「声明得起但装不下」的文件挡在分配之前（L10）。
    const auto required_bytes =
        static_cast<std::uintmax_t>(layout.stride) * static_cast<std::uintmax_t>(point_count);
    if (required_bytes > kMaxPcdBytes) {
        result.error = "PCD 数据量超过上限（需要 " + std::to_string(required_bytes) +
                       " 字节，上限 " + std::to_string(kMaxPcdBytes) + "）";
        return result;
    }
    try {
        result.cloud.points.reserve(point_count);
    } catch (const std::bad_alloc&) {
        result.error = "PCD 点数预分配失败（声明 " + std::to_string(point_count) + " 点）";
        return result;
    }

    if (header.data == "ascii") {
        std::string line;
        std::size_t index = 0;
        while (index < point_count && std::getline(file, line)) {
            if (!line.empty() && line.back() == '\r') {
                line.pop_back();
            }
            const std::vector<std::string> tokens = split_tokens(line);
            if (tokens.size() < header.fields.size()) {
                continue;
            }
            float x = 0.0F;
            float y = 0.0F;
            float z = 0.0F;
            if (!parse_float_token(tokens[layout.x_type_index], &x) ||
                !parse_float_token(tokens[layout.y_type_index], &y) ||
                !parse_float_token(tokens[layout.z_type_index], &z)) {
                result.error = "PCD ascii 第 " + std::to_string(index + 1U) + " 行 x/y/z 不是数值";
                return result;
            }
            if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
                result.cloud.points.emplace_back(x, y, z);
            }
            ++index;
        }
        if (index < point_count) {
            result.error = "PCD ascii 数据长度不足（声明 " + std::to_string(point_count) +
                           " 点，实际 " + std::to_string(index) + " 行）";
            return result;
        }
    } else {
        // 二进制段：先量文件剩余字节再分配，别让「声明 20 亿点」直接落到 vector 上。
        const std::streampos data_begin = file.tellg();
        file.seekg(0, std::ios::end);
        const std::streampos data_end = file.tellg();
        if (data_begin < 0 || data_end < data_begin) {
            result.error = "无法确定 PCD 二进制段长度";
            return result;
        }
        file.seekg(data_begin);
        const auto remaining = static_cast<std::uintmax_t>(data_end - data_begin);
        if (remaining < required_bytes) {
            result.error = "PCD 二进制数据长度不足（需要 " + std::to_string(required_bytes) +
                           " 字节，文件只剩 " + std::to_string(remaining) + " 字节）";
            return result;
        }
        std::vector<char> buffer;
        try {
            buffer.resize(static_cast<std::size_t>(required_bytes));
        } catch (const std::bad_alloc&) {
            result.error =
                "PCD 二进制缓冲区分配失败（" + std::to_string(required_bytes) + " 字节）";
            return result;
        }
        file.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
        if (static_cast<std::size_t>(file.gcount()) != buffer.size()) {
            result.error = "PCD 二进制段读取不完整（短读）";
            return result;
        }
        for (std::size_t i = 0; i < point_count; ++i) {
            const char* base = buffer.data() + i * layout.stride;
            float x = 0.0F;
            float y = 0.0F;
            float z = 0.0F;
            if (!read_numeric(base, layout.x_offset, layout.x_size,
                              header.types[layout.x_type_index], &x) ||
                !read_numeric(base, layout.y_offset, layout.y_size,
                              header.types[layout.y_type_index], &y) ||
                !read_numeric(base, layout.z_offset, layout.z_size,
                              header.types[layout.z_type_index], &z)) {
                result.error =
                    "PCD 的 x/y/z 类型不受支持（仅支持 F 的 4/8 字节与 I/U 的 2/4 字节）";
                return result;
            }
            if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
                result.cloud.points.emplace_back(x, y, z);
            }
        }
    }

    if (result.cloud.points.empty()) {
        result.error = "PCD 解析后没有有效点";
        return result;
    }
    result.ok = true;
    return result;
}

} // namespace foray::localization_map
