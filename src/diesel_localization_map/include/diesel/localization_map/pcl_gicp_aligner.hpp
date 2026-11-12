// Copyright 2026 ZQU-Foray
//
// PCL GICP 配准后端（默认后端，见 decision.md D21、D27）。
//
// 本头不包含任何 PCL 头：PCL 只在 .cpp 里出现，ROS 包与测试 include 它时
// 不会连带继承 PCL 的编译选项、宏与版本耦合。头文件本身不区分
// DIESEL_WITH_PCL：未启用 PCL 的构建也能 include（只是不会构造本类）。
//
// 后端内部复用同一个 GICP 对象（PCL 类型藏在 pimpl 后面），只有 target 换
// 了才重新 setInputTarget——离线基准显示这样比每帧新建快 2 倍，且位姿与
// fitness 逐位相同（decision.md D27）。

#ifndef DIESEL_LOCALIZATION_MAP_PCL_GICP_ALIGNER_HPP_
#define DIESEL_LOCALIZATION_MAP_PCL_GICP_ALIGNER_HPP_

#include "diesel/localization_map/aligner.hpp"

#include <memory>

namespace diesel::localization_map {

/// 以 PCL GeneralizedIterativeClosestPoint 实现的配准后端。
class PclGicpAligner final : public Aligner {
  public:
    explicit PclGicpAligner(AlignOptions options = AlignOptions{});
    ~PclGicpAligner() override;

    // 内部持有 PCL 对象与 target 预处理缓存，不支持拷贝；需要独立实例请走 make_aligner。
    PclGicpAligner(const PclGicpAligner&) = delete;
    PclGicpAligner& operator=(const PclGicpAligner&) = delete;

    AlignResult align(const PointCloud& source, const PointCloud& target,
                      const Eigen::Isometry3d& initial_guess) override;

    const AlignOptions& options() const { return options_; }

    /// 替换配准参数。下一次 `align()` 生效——参数在每次配准时下发，
    /// 不依赖「构造时配置一次」的隐式假设（审计 L14）。
    void set_options(const AlignOptions& options);

  private:
    struct Impl;

    AlignOptions options_;
    std::unique_ptr<Impl> impl_;
};

} // namespace diesel::localization_map

#endif // DIESEL_LOCALIZATION_MAP_PCL_GICP_ALIGNER_HPP_
