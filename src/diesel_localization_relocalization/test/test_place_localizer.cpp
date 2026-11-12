// Copyright 2026 ZQU-Foray

#include "diesel/localization_relocalization/place_localizer.hpp"

#include <gtest/gtest.h>

namespace {

TEST(PlaceLocalizerTest, DisabledByDefault) {
    diesel::localization_relocalization::PlaceLocalizer localizer;
    diesel::localization_map::PointCloud cloud;
    cloud.points.emplace_back(0.0F, 0.0F, 0.0F);
    diesel::localization_map::PriorMap map;
    map.cloud = cloud;

    const auto result = localizer.estimate(cloud, map);
    EXPECT_FALSE(result.ok);
    EXPECT_FALSE(result.error.empty());
}

TEST(PlaceLocalizerTest, EnabledWithoutOnnxReportsUnavailable) {
    diesel::localization_relocalization::PlaceConfig config;
    config.enabled = true;
    config.model_path = "/nonexistent/model.onnx";
    diesel::localization_relocalization::PlaceLocalizer localizer(config);

    diesel::localization_map::PointCloud cloud;
    cloud.points.emplace_back(0.0F, 0.0F, 0.0F);
    diesel::localization_map::PriorMap map;
    map.cloud = cloud;

    const auto result = localizer.estimate(cloud, map);
    EXPECT_FALSE(result.ok);
    EXPECT_FALSE(result.error.empty());
}

TEST(PlaceLocalizerTest, EmptyInputRejected) {
    diesel::localization_relocalization::PlaceConfig config;
    config.enabled = true;
    diesel::localization_relocalization::PlaceLocalizer localizer(config);
    diesel::localization_map::PointCloud cloud;
    diesel::localization_map::PriorMap map;

    const auto result = localizer.estimate(cloud, map);
    EXPECT_FALSE(result.ok);
}

} // namespace
