// Copyright 2026 ZQU-Foray

#include "diesel/localization_map/aligner_factory.hpp"

#include "diesel/localization_map/icp_aligner.hpp"

#if defined(DIESEL_WITH_PCL)
#include "diesel/localization_map/pcl_gicp_aligner.hpp"
#endif

namespace diesel::localization_map {

std::unique_ptr<Aligner> make_aligner(const AlignOptions& options) {
#if defined(DIESEL_WITH_PCL)
    return std::make_unique<PclGicpAligner>(options);
#else
    return std::make_unique<IcpAligner>(options);
#endif
}

std::string default_aligner_name() {
#if defined(DIESEL_WITH_PCL)
    return "pcl_gicp";
#else
    return "icp（未启用 PCL）";
#endif
}

} // namespace diesel::localization_map
