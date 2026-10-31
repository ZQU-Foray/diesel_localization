// Copyright 2026 ZQU-Foray
//
// 节点级测试：在进程内构造 LocalizationNode，用真实话题喂数据、从真实话题读结果。
// 这里只放「基建可用性」与后续 L1 用例；地图失败、位姿源仲裁等另有
// test_localization_node_map_errors.cpp（decision.md D29）。

#include "localization_node_test_support.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdio>
#include <memory>
#include <vector>

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

/// L1（blocker）：注入 200 ms 配准耗时，里程计 200 Hz、状态 50 Hz 连跑 6 s。
///
/// 父提交（单线程 executor + 全部回调挤在默认 MutuallyExclusive 组里）的故障形态是：
/// 配准占满执行线程 → 里程计回调被饿死 → `last_odom_ns_` 不再推进 → 健康状态报
/// "odometry timeout" 转 LOST → TF 停发。本用例要求：不 LOST、reason 里不出现
/// odometry stale、里程计积压峰值远小于 odom_timeout_sec、状态发布频率不受配准影响。
TEST(LocalizationNodeTest, SlowAlignmentKeepsHealthOk) {
    TempMapDir map;
    std::vector<rclcpp::Parameter> overrides = {
        rclcpp::Parameter("map_manifest", map.manifest_path()),
        // 200 ms 配准 @ 5 Hz：单线程下配准占满整条执行线程。
        rclcpp::Parameter("align_rate_hz", 5.0),
        rclcpp::Parameter("state_rate_hz", 50.0),
        rclcpp::Parameter("odom_timeout_sec", 0.5),
    };
    NodeHarness harness(overrides, std::chrono::milliseconds(200));

    // 先喂到 OK（health 要求连续 warmup_frames=3 帧合格配准）。
    bool became_ok = false;
    int cloud_tick = 0;
    harness.drive(std::chrono::milliseconds(6000), std::chrono::milliseconds(5),
                  [&](std::int64_t) {
                      harness.publish_odom(0.0, 0.0);
                      if (++cloud_tick % 20 == 0) {
                          harness.publish_cloud(10);
                      }
                      became_ok = became_ok ||
                                  harness.last_state().state == LocalizationState::STATE_OK;
                  });
    // 预热阶段的计数也要打出来：父提交下本用例的失败点就在这几项上
    // （里程计没被处理、积压爆掉、状态节拍跑不动），不能只报一句「没进 OK」。
    const auto warmup_diagnostics = harness.diagnostics();
    std::printf("L1 预热回执: became_ok=%d odom_received=%llu odom_processed=%llu "
                "max_odom_lag_ms=%.1f align_runs=%llu states=%zu reason=%s\n",
                became_ok ? 1 : 0,
                static_cast<unsigned long long>(warmup_diagnostics.odom_received),
                static_cast<unsigned long long>(warmup_diagnostics.odom_processed),
                1000.0 * warmup_diagnostics.max_odom_lag_sec,
                static_cast<unsigned long long>(warmup_diagnostics.align_runs),
                harness.state_count(), harness.last_state().reason.c_str());
    ASSERT_TRUE(became_ok) << "注入后端没有让健康状态进入 OK，用例前提不成立：odom_received="
                           << warmup_diagnostics.odom_received
                           << " odom_processed=" << warmup_diagnostics.odom_processed
                           << " max_odom_lag_ms=" << 1000.0 * warmup_diagnostics.max_odom_lag_sec
                           << " align_runs=" << warmup_diagnostics.align_runs
                           << " states=" << harness.state_count()
                           << " reason=" << harness.last_state().reason;

    const std::size_t states_before = harness.state_count();
    const auto diagnostics_before = harness.diagnostics();
    harness.drive(std::chrono::milliseconds(6000), std::chrono::milliseconds(5),
                  [&](std::int64_t elapsed_ms) {
                      harness.publish_odom(0.0, 0.0);
                      if (elapsed_ms % 100 < 5) {
                          harness.publish_cloud(10);
                      }
                  });
    const auto diagnostics = harness.diagnostics();
    const std::size_t published = harness.state_count() - states_before;
    const double observed_state_hz = static_cast<double>(published) / 6.0;

    std::printf("L1 回执: LOST=%d state_hz=%.1f max_odom_lag_ms=%.1f align_runs=%llu "
                "max_align_ms=%.1f\n",
                harness.has_lost() ? 1 : 0, observed_state_hz,
                1000.0 * diagnostics.max_odom_lag_sec,
                static_cast<unsigned long long>(diagnostics.align_runs),
                diagnostics.max_align_ms);

    EXPECT_FALSE(harness.has_lost()) << "配准慢不该把健康状态打成 LOST";
    EXPECT_FALSE(harness.has_reason("odometry stale"))
        << "里程计被配准饿死时报了 odometry stale，reason=" << harness.last_state().reason;
    EXPECT_FALSE(harness.has_reason("no odometry received"));
    EXPECT_LT(diagnostics.max_odom_lag_sec, 0.05)
        << "里程计帧在队列里等了 " << 1000.0 * diagnostics.max_odom_lag_sec << " ms";
    EXPECT_GE(published, static_cast<std::size_t>(0.8 * 50.0 * 6.0))
        << "状态发布被配准拖慢到 " << observed_state_hz << " Hz";
    EXPECT_GT(diagnostics.align_runs, diagnostics_before.align_runs);
    // 配准确实在慢，否则本用例什么也没验证。
    EXPECT_GT(diagnostics.max_align_ms, 150.0);
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
