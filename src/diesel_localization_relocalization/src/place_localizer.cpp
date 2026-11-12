// Copyright 2026 ZQU-Foray

#include "diesel/localization_relocalization/place_localizer.hpp"

#include <utility>

#if defined(DIESEL_WITH_ONNX)
#include <onnxruntime_cxx_api.h>
#endif

namespace diesel::localization_relocalization {

bool onnx_backend_available() {
#if defined(DIESEL_WITH_ONNX)
    return true;
#else
    return false;
#endif
}

PlaceLocalizer::PlaceLocalizer(PlaceConfig config) : config_(std::move(config)) {}

const PlaceConfig& PlaceLocalizer::config() const {
    return config_;
}

void PlaceLocalizer::set_config(const PlaceConfig& config) {
    config_ = config;
}

PlaceResult PlaceLocalizer::estimate(const diesel::localization_map::PointCloud& cloud,
                                     const diesel::localization_map::PriorMap& map) {
    PlaceResult result;
    if (!config_.enabled) {
        result.error = "地点识别后端未启用（D10：默认关闭）";
        return result;
    }
    if (cloud.empty() || map.empty()) {
        result.error = "点云或先验地图为空";
        return result;
    }
#if !defined(DIESEL_WITH_ONNX)
    result.error = "未链接 ONNX Runtime，地点识别不可用；请以 -DDIESEL_WITH_ONNX=ON 并安装纯 CPU "
                   "onnxruntime 重新构建";
    return result;
#else
    // 首版只实现接口与模型装载校验；推理管线待 P2 赛季数据到位后接入。
    result.error = "ONNX 推理管线尚未实现（P2 范围）";
    return result;
#endif
}

} // namespace diesel::localization_relocalization
