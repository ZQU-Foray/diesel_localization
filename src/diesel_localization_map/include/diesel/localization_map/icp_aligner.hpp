// Copyright 2026 ZQU-Foray
//
// 内置点到点 ICP。不依赖 PCL/small_gicp，保证在最小构建环境与 CI 中也能
// 完成配准与健康状态验证。默认后端是 PCL GICP（decision.md D21），本类在
// -DDIESEL_WITH_PCL=OFF 的构建里才是实际使用的后端；精度低于 GICP，
// 仅用于无 PCL 环境与降级排查。

#ifndef DIESEL_LOCALIZATION_MAP_ICP_ALIGNER_HPP_
#define DIESEL_LOCALIZATION_MAP_ICP_ALIGNER_HPP_

#include "diesel/localization_map/aligner.hpp"

namespace diesel::localization_map {

class IcpAligner final : public Aligner {
  public:
    explicit IcpAligner(AlignOptions options = AlignOptions{});

    AlignResult align(const PointCloud& source, const PointCloud& target,
                      const Eigen::Isometry3d& initial_guess) override;

    const AlignOptions& options() const { return options_; }

  private:
    AlignOptions options_;
};

} // namespace diesel::localization_map

#endif // DIESEL_LOCALIZATION_MAP_ICP_ALIGNER_HPP_
