// Copyright 2026 ZQU-Foray

#include "diesel/localization_map/grid_map.hpp"

#include <cmath>
#include <fstream>
#include <iterator>
#include <string>

namespace diesel::localization_map {
namespace {

bool is_space(char c) {
    return c == ' ' || c == '\t' || c == '\r' || c == '\n';
}

/// 读取下一个 PGM 记号；`#` 到行尾为注释。
bool read_token(const std::string& data, std::size_t* pos, std::string* token) {
    while (*pos < data.size()) {
        if (data[*pos] == '#') {
            while (*pos < data.size() && data[*pos] != '\n') {
                ++*pos;
            }
        } else if (is_space(data[*pos])) {
            ++*pos;
        } else {
            break;
        }
    }
    if (*pos >= data.size()) {
        return false;
    }
    const std::size_t begin = *pos;
    while (*pos < data.size() && !is_space(data[*pos])) {
        ++*pos;
    }
    *token = data.substr(begin, *pos - begin);
    return true;
}

} // namespace

PgmLoadResult load_pgm(const std::filesystem::path& path) {
    PgmLoadResult result;
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        result.error = "无法打开 PGM 文件: " + path.string();
        return result;
    }
    const std::string data((std::istreambuf_iterator<char>(file)),
                           std::istreambuf_iterator<char>());

    std::size_t pos = 0;
    std::string magic;
    std::string width_token;
    std::string height_token;
    std::string max_token;
    if (!read_token(data, &pos, &magic) || !read_token(data, &pos, &width_token) ||
        !read_token(data, &pos, &height_token) || !read_token(data, &pos, &max_token)) {
        result.error = "PGM 头不完整";
        return result;
    }
    if (magic != "P5" && magic != "P2") {
        result.error = "仅支持 P2/P5 格式的 PGM，实际: " + magic;
        return result;
    }

    PgmImage& image = result.image;
    image.width = std::stoi(width_token);
    image.height = std::stoi(height_token);
    image.max_value = std::stoi(max_token);
    if (image.width <= 0 || image.height <= 0 || image.max_value <= 0 || image.max_value > 255) {
        result.error = "PGM 尺寸或最大值非法";
        return result;
    }

    const std::size_t pixel_count =
        static_cast<std::size_t>(image.width) * static_cast<std::size_t>(image.height);
    image.pixels.resize(pixel_count);

    if (magic == "P5") {
        // P5 在 maxval 之后恰好一个空白字符，然后紧跟原始字节。
        if (pos < data.size() && is_space(data[pos])) {
            ++pos;
        }
        if (data.size() - pos < pixel_count) {
            result.error = "PGM 二进制像素数据不足";
            return result;
        }
        for (std::size_t i = 0; i < pixel_count; ++i) {
            image.pixels[i] = static_cast<std::uint8_t>(static_cast<unsigned char>(data[pos + i]));
        }
    } else {
        for (std::size_t i = 0; i < pixel_count; ++i) {
            std::string token;
            if (!read_token(data, &pos, &token)) {
                result.error = "PGM 文本像素数据不足";
                return result;
            }
            image.pixels[i] = static_cast<std::uint8_t>(std::stoi(token));
        }
    }

    result.ok = true;
    return result;
}

GridMap grid_from_pgm(const PgmImage& image, double resolution,
                      const std::array<double, 3>& origin) {
    GridMap grid;
    grid.width = image.width;
    grid.height = image.height;
    grid.resolution = resolution;
    grid.origin = Eigen::Vector2d(origin[0], origin[1]);
    grid.yaw = origin[2];
    grid.cells.assign(
        static_cast<std::size_t>(image.width) * static_cast<std::size_t>(image.height), -1);

    const double max_value = image.max_value > 0 ? static_cast<double>(image.max_value) : 255.0;
    for (int py = 0; py < image.height; ++py) {
        // PGM 的第一行是图像顶部；地图栅格的第一行是 y 最小处，因此翻转。
        const int row = image.height - 1 - py;
        for (int px = 0; px < image.width; ++px) {
            const std::uint8_t value =
                image.pixels[static_cast<std::size_t>(py) * image.width + px];
            const double occupancy = (max_value - static_cast<double>(value)) / max_value;
            std::int8_t cell = -1;
            // trinary 阈值与 nav2 map_server 一致：>0.65 占据，<0.196 空闲，其余未知。
            if (occupancy > 0.65) {
                cell = 100;
            } else if (occupancy < 0.196) {
                cell = 0;
            }
            grid.cells[static_cast<std::size_t>(row) * image.width + px] = cell;
        }
    }
    return grid;
}

bool GridMap::world_to_cell(double x, double y, int* ix, int* iy) const {
    if (resolution <= 0.0) {
        return false;
    }
    const double dx = x - origin.x();
    const double dy = y - origin.y();
    const double cos_yaw = std::cos(-yaw);
    const double sin_yaw = std::sin(-yaw);
    const double local_x = cos_yaw * dx - sin_yaw * dy;
    const double local_y = sin_yaw * dx + cos_yaw * dy;
    const int cell_x = static_cast<int>(std::floor(local_x / resolution));
    const int cell_y = static_cast<int>(std::floor(local_y / resolution));
    if (cell_x < 0 || cell_y < 0 || cell_x >= width || cell_y >= height) {
        return false;
    }
    *ix = cell_x;
    *iy = cell_y;
    return true;
}

std::int8_t GridMap::occupancy_at(double x, double y) const {
    int ix = 0;
    int iy = 0;
    if (!world_to_cell(x, y, &ix, &iy)) {
        return -1;
    }
    return cells[static_cast<std::size_t>(iy) * width + ix];
}

bool GridMap::is_occupied(double x, double y) const {
    return occupancy_at(x, y) == 100;
}

} // namespace diesel::localization_map
