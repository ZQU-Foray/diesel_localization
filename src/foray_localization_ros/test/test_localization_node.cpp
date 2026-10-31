// Copyright 2026 ZQU-Foray
//
// 节点级测试：在进程内构造 LocalizationNode，用真实话题喂数据、从真实话题读结果。
// 这里只放「基建可用性」与后续 L1 用例；地图失败、位姿源仲裁等另有
// test_localization_node_map_errors.cpp（decision.md D29）。

#include "localization_node_test_support.hpp"

#include <gtest/gtest.h>

#include <memory>

namespace foray::localization_ros::test_support {
namespace {

using foray_localization_ros::msg::LocalizationState;

std::vector<rclcpp::Parameter> map_overrides(const TempMapDir& map) {
    return {rclcpp::Parameter("map_manifest", map.manifest_path())};
}

/// 基建冒烟：节点能在测试进程里被构造、按节拍发状态、收里程计并真的跑配准。
/// 这条用例存在本身就是证据——父提交的节点没有进库，测试根本链接不到它。
TEST(LocalizationNodeTest, HarnessPublishesStateAndRunsAlignment) {
    TempMapDir map;
    NodeHarness harness(map_overrides(map), 5ms);

    ASSERT_TRUE(harness.wait_until([&harness]() { return harness.state_count() > 5; }, 3000ms))
        << "节点没有按 state_rate_hz 发布状态";
    EXPECT_FALSE(harness.states().empty());

    harness.publish_odom(0.0, 0.0);
    harness.publish_cloud(10);

    ASSERT_TRUE(harness.wait_until(
        [&harness]() {
            const auto diagnostics = harness.diagnostics();
            return diagnostics.odom_received > 0 && diagnostics.align_runs > 0;
        },
        3000ms))
        << "里程计没进回调或配准定时器没跑起来";

    const auto diagnostics = harness.diagnostics();
    EXPECT_GE(diagnostics.odom_processed, 1U);
    EXPECT_GE(diagnostics.max_align_ms, 0.0);
    EXPECT_GE(diagnostics.max_odom_lag_sec, 0.0);

    const auto state = harness.last_state();
    EXPECT_GT(state.stamp_ns, 0);
}

} // namespace
} // namespace foray::localization_ros::test_support

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
