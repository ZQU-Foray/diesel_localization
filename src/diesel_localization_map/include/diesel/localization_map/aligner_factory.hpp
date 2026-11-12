// Copyright 2026 ZQU-Foray
//
// 配准后端工厂：把「编译期选了哪个后端」收在一处，ROS 节点与测试都不写 #ifdef。
// 默认后端是 PCL GICP（decision.md D21）；DIESEL_WITH_PCL=OFF 时退回内置点到点 ICP。

#ifndef DIESEL_LOCALIZATION_MAP_ALIGNER_FACTORY_HPP_
#define DIESEL_LOCALIZATION_MAP_ALIGNER_FACTORY_HPP_

#include "diesel/localization_map/aligner.hpp"

#include <memory>
#include <string>

namespace diesel::localization_map {

/// 按编译期开关创建配准后端。
std::unique_ptr<Aligner> make_aligner(const AlignOptions& options = AlignOptions{});

/// 当前构建实际使用的后端名（`pcl_gicp` 或 `icp（未启用 PCL）`）。
/// 节点把它打在启动日志里，避免出现「文档说 GICP、实际跑 ICP」的误会。
std::string default_aligner_name();

} // namespace diesel::localization_map

#endif // DIESEL_LOCALIZATION_MAP_ALIGNER_FACTORY_HPP_
