// Copyright 2026 ZQU-Foray
//
// 节点级测试：在进程内构造 LocalizationNode，用真实话题喂数据、从真实话题读结果。
// 这里只放「基建可用性」与后续 L1 用例；地图失败、位姿源仲裁等另有
// test_localization_node_map_errors.cpp（decision.md D29）。

#include "localization_node_test_support.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <limits>
#include <memory>
#include <vector>

namespace diesel::localization_ros::test_support {
namespace {

using diesel_localization_ros::msg::LocalizationState;

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
    harness.drive(std::chrono::milliseconds(6000), std::chrono::milliseconds(5), [&](std::int64_t) {
        harness.publish_odom(0.0, 0.0);
        if (++cloud_tick % 20 == 0) {
            harness.publish_cloud(10);
        }
        became_ok = became_ok || harness.last_state().state == LocalizationState::STATE_OK;
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
                static_cast<unsigned long long>(diagnostics.align_runs), diagnostics.max_align_ms);

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

/// L16：算法路径（5 Hz 配准 + 聚类 + 可观性）占住 CPU 时，里程计与状态发布不许被饿死。
///
/// 与 L1 的差别在负载形态与对照组：这里用**忙等**占核心（GICP 真在算的样子，不是
/// 睡 200 ms 让出 CPU），并且把同样的负载放到单线程执行器上跑一遍——那一遍必须塌，
/// 否则这条用例证明不了「是执行器与回调组把阻塞解开的」。
TEST(LocalizationNodeTest, AlgorithmPathDoesNotStarveTheOdomPath) {
    struct LoadOutcome {
        double state_hz{0.0};
        double max_odom_lag_sec{0.0};
        std::uint64_t odom_processed{0};
        std::uint64_t align_runs{0};
        bool lost{false};
        bool became_ok{false};
        std::string reason;
    };

    // 跑两遍同一份负载，区别只在执行器线程数。
    auto run = [](std::size_t threads) {
        TempMapDir map;
        std::vector<rclcpp::Parameter> overrides = {
            rclcpp::Parameter("map_manifest", map.manifest_path()),
            rclcpp::Parameter("align_rate_hz", 5.0),
            rclcpp::Parameter("state_rate_hz", 50.0),
            rclcpp::Parameter("odom_timeout_sec", 0.5),
        };
        // 150 ms 忙等 @ 5 Hz：算法路径占掉 75% 的一个核心。
        NodeHarness harness(overrides, std::chrono::milliseconds(150), threads,
                            /*busy_align=*/true);
        LoadOutcome outcome;
        int cloud_tick = 0;
        harness.drive(std::chrono::milliseconds(2000), std::chrono::milliseconds(5),
                      [&](std::int64_t) {
                          harness.publish_odom(0.0, 0.0);
                          if (++cloud_tick % 20 == 0) {
                              harness.publish_cloud(10);
                          }
                          outcome.became_ok = outcome.became_ok || harness.last_state().state ==
                                                                       LocalizationState::STATE_OK;
                      });

        const std::size_t states_before = harness.state_count();
        harness.drive(std::chrono::milliseconds(2000), std::chrono::milliseconds(5),
                      [&](std::int64_t elapsed_ms) {
                          harness.publish_odom(0.0, 0.0);
                          if (elapsed_ms % 100 < 5) {
                              harness.publish_cloud(10);
                          }
                      });
        const auto diagnostics = harness.diagnostics();
        outcome.state_hz = static_cast<double>(harness.state_count() - states_before) / 2.0;
        outcome.max_odom_lag_sec = diagnostics.max_odom_lag_sec;
        outcome.odom_processed = diagnostics.odom_processed;
        outcome.align_runs = diagnostics.align_runs;
        outcome.lost = harness.has_lost();
        outcome.reason = harness.last_state().reason;
        return outcome;
    };

    const LoadOutcome multithreaded = run(0);
    std::printf("L16 回执(多线程): became_ok=%d LOST=%d state_hz=%.1f max_odom_lag_ms=%.1f "
                "odom_processed=%llu align_runs=%llu reason=%s\n",
                multithreaded.became_ok ? 1 : 0, multithreaded.lost ? 1 : 0, multithreaded.state_hz,
                1000.0 * multithreaded.max_odom_lag_sec,
                static_cast<unsigned long long>(multithreaded.odom_processed),
                static_cast<unsigned long long>(multithreaded.align_runs),
                multithreaded.reason.c_str());

    ASSERT_TRUE(multithreaded.became_ok)
        << "忙等负载下没进 OK，用例前提不成立：reason=" << multithreaded.reason;
    EXPECT_FALSE(multithreaded.lost) << "reason=" << multithreaded.reason;
    EXPECT_LT(multithreaded.max_odom_lag_sec, 0.05)
        << "里程计帧等了 " << 1000.0 * multithreaded.max_odom_lag_sec << " ms";
    EXPECT_GE(multithreaded.state_hz, 0.8 * 50.0)
        << "状态发布被算法路径拖到 " << multithreaded.state_hz << " Hz";
    EXPECT_GT(multithreaded.align_runs, 0U);

    // 对照组：同样的负载、单线程执行器（L1/L16 之前的产线配置）。它必须塌，否则上面几条
    // 断言什么也没证明——这个负载根本没让执行器成为瓶颈。
    const LoadOutcome single_threaded = run(1);
    std::printf("L16 回执(单线程对照): became_ok=%d LOST=%d state_hz=%.1f max_odom_lag_ms=%.1f "
                "odom_processed=%llu align_runs=%llu reason=%s\n",
                single_threaded.became_ok ? 1 : 0, single_threaded.lost ? 1 : 0,
                single_threaded.state_hz, 1000.0 * single_threaded.max_odom_lag_sec,
                static_cast<unsigned long long>(single_threaded.odom_processed),
                static_cast<unsigned long long>(single_threaded.align_runs),
                single_threaded.reason.c_str());
    EXPECT_TRUE(single_threaded.max_odom_lag_sec > 0.2 || single_threaded.state_hz < 25.0)
        << "单线程对照组没有复现饥饿（lag=" << 1000.0 * single_threaded.max_odom_lag_sec
        << " ms, state_hz=" << single_threaded.state_hz << "），本用例的负载不足";
}

/// 附带 (a)：聚类的新鲜度必须用**节点时钟**上的「收到时刻」判，不能用点云 header 里
/// 的传感器时间戳——两个时钟域相差几秒时，跨域相减得到的是无意义的差值。
TEST(LocalizationNodeTest, ObstacleClusteringUsesReceiveTimeNotSensorStamp) {
    TempMapDir map;
    std::vector<rclcpp::Parameter> overrides = {
        rclcpp::Parameter("map_manifest", map.manifest_path()),
        // 聚类跑得比配准快，配准慢一点让「点云被取走」的时刻尽量少影响观测。
        rclcpp::Parameter("obstacle_rate_hz", 50.0),
        rclcpp::Parameter("align_rate_hz", 1.0),
    };
    NodeHarness harness(overrides, std::chrono::milliseconds(20));

    // 传感器时钟比节点时钟慢 10 s：父提交的 `now - header.stamp` 于是恒 > 1 s，
    // 每一帧新鲜点云都被判成过期，`/localization/dynamic_obstacles` 一帧都不发。
    constexpr std::int64_t kSensorClockSkewNs = -10'000'000'000LL;
    int cloud_tick = 0;
    harness.drive(std::chrono::milliseconds(2000), std::chrono::milliseconds(5), [&](std::int64_t) {
        harness.publish_odom(0.0, 0.0);
        if (++cloud_tick % 20 == 0) {
            harness.publish_cloud(10, "base_footprint", kSensorClockSkewNs);
        }
    });

    const std::int64_t published_stamp = harness.last_obstacle_stamp_ns();
    const std::int64_t cloud_stamp = harness.last_cloud_stamp_ns();
    std::printf("附带(a) 回执: 传感器时钟偏移 -10s，2000ms 内障碍帧数=%zu 发布戳偏差=%lld ms\n",
                harness.obstacle_count(),
                static_cast<long long>((published_stamp - cloud_stamp) / 1000000LL));

    EXPECT_GT(harness.obstacle_count(), 0U)
        << "点云在新鲜期内到达，聚类却被跨时钟域的比较判成了过期";
    // 发布出去的时间戳仍然是数据自身的时刻（不改消息语义，只改判新鲜度用的时钟）。
    EXPECT_EQ(published_stamp, cloud_stamp);
}

/// L3：`Odometry.pose` 是 `odom → child_frame_id`，不是 `odom → base_footprint`。
/// 上游（point_lio）按 `odom → livox_frame` 发布，两者的差就是雷达外参。
/// 这里给一条 0.11 m 的 `base_footprint → livox_frame` 静态外参，看状态位姿是否按它换算。
TEST(LocalizationNodeTest, OdometryChildFrameIsConvertedThroughTf) {
    TempMapDir map;
    NodeHarness harness(map_overrides(map), 5ms);
    harness.publish_static_transform("base_footprint", "livox_frame", 0.11, 0.0);

    // 静态外参是异步进 buffer 的：持续发里程计，直到真的有帧被接受。
    harness.drive(1000ms, 10ms,
                  [&harness](std::int64_t) { harness.publish_odom(1.0, 0.0, "livox_frame"); });
    ASSERT_TRUE(harness.wait_until(
        [&harness]() { return harness.diagnostics().odom_processed > 0; }, 3000ms))
        << "odom → livox_frame 的帧全部被丢弃：TF 没接上或子帧校验过严";
    ASSERT_TRUE(harness.wait_until([&harness]() { return harness.state_count() > 3; }, 3000ms));

    const double base_x = harness.last_state().x;
    std::printf("L3 回执: odom→livox_frame x=1.000, base_footprint→livox_frame x=0.110, "
                "state x=%.3f\n",
                base_x);
    // odom → base = odom → livox * (base ← livox) = 1.00 - 0.11。
    EXPECT_NEAR(base_x, 0.89, 0.02)
        << "子帧外参没有被换算进 odom → base（父提交直接当成 odom → base 用，会得到 1.000）";
}

/// L3：子帧对不上又查不到 TF 时必须丢帧。父提交不看子帧，照收不误。
TEST(LocalizationNodeTest, OdometryWithoutChildFrameTfIsDropped) {
    TempMapDir map;
    NodeHarness harness(map_overrides(map), 5ms);

    harness.drive(800ms, 10ms,
                  [&harness](std::int64_t) { harness.publish_odom(1.0, 0.0, "livox_frame"); });

    const auto diagnostics = harness.diagnostics();
    std::printf("L3 回执: odom_received=%llu odom_processed=%llu\n",
                static_cast<unsigned long long>(diagnostics.odom_received),
                static_cast<unsigned long long>(diagnostics.odom_processed));
    EXPECT_GT(diagnostics.odom_received, 0U);
    EXPECT_EQ(diagnostics.odom_processed, 0U)
        << "查不到 base ← livox_frame 却仍然把 odom → livox 当成 odom → base 用了";
    // 一帧都没被接受 ⇒ 从来没有 odom → base，TF 也就一帧都不该发。
    // （「收不到任何里程计」这条路径下状态机目前停在 INITIALIZING 而不会转 LOST，
    //   因为 publish_state() 的各分支都以 `has_odom_` 为前置；这是本轮新发现，另行上报。）
    EXPECT_EQ(harness.diagnostics().tf_published, 0U);
    harness.drive(600ms, 10ms, [&harness](std::int64_t) { harness.publish_cloud(10); });
    EXPECT_NE(harness.last_state().state, LocalizationState::STATE_OK);
}

/// L8：README 把 `/aft_mapped_to_init` 声明为 sensor_data，节点就必须收得下 best_effort 的上游。
///
/// 父提交对里程计用 `rclcpp::QoS(50)`（reliable）：上游按 README 用 best_effort 发布时
/// DDS 判定 QoS 不兼容，回调一帧都收不到，于是「读不到里程计」被误报成上游断流。
TEST(LocalizationNodeTest, BestEffortOdometryUpstreamIsReceived) {
    TempMapDir map;
    NodeHarness harness(map_overrides(map), 5ms);

    harness.drive(1500ms, 10ms, [&harness](std::int64_t) {
        harness.publish_odom(0.5, 0.0, "base_footprint", /*reliable=*/false);
    });

    const auto diagnostics = harness.diagnostics();
    std::printf("L8 回执: best_effort 上游 1500ms → odom_received=%llu odom_processed=%llu\n",
                static_cast<unsigned long long>(diagnostics.odom_received),
                static_cast<unsigned long long>(diagnostics.odom_processed));
    EXPECT_GT(diagnostics.odom_received, 0U)
        << "best_effort 上游的里程计一帧都没进回调：订阅端与 README 的 sensor_data 不兼容";
    EXPECT_GT(diagnostics.odom_processed, 0U);
}

/// 一直喂里程计 + 点云直到健康状态进入 OK（warmup_frames 帧合格配准）。
bool drive_until_ok(NodeHarness& harness, std::chrono::milliseconds timeout,
                    std::size_t cloud_points = 10) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        harness.publish_odom(0.0, 0.0);
        harness.publish_cloud(cloud_points);
        if (harness.last_state().state == LocalizationState::STATE_OK) {
            return true;
        }
        std::this_thread::sleep_for(20ms);
    }
    return false;
}

/// 一份 ascii PCD：声明 4 行，其中 1 行是 `nan` 点（PCD 里常见的无效回波占位）。
/// 载入后只有 3 个点，跳过 1 行。
std::string pcd_with_one_nan_row() {
    return "VERSION 0.7\n"
           "FIELDS x y z\n"
           "SIZE 4 4 4\n"
           "TYPE F F F\n"
           "COUNT 1 1 1\n"
           "WIDTH 4\n"
           "HEIGHT 1\n"
           "POINTS 4\n"
           "DATA ascii\n"
           "0 0 0\n"
           "0.5 0 0\n"
           "nan 0 0\n"
           "1.0 0 0\n";
}

/// L18：地图里夹着坏行时，装载照旧成功、定位照旧进 OK——但「丢了几行」必须有出口。
/// 父提交的节点对 NaN 行只是 `continue`：点数比声明少、日志里一个字都没有。
TEST(LocalizationNodeTest, MapWithSkippedRowsStillLocalizes) {
    TempMapDir map(pcd_with_one_nan_row());
    NodeHarness harness(map_overrides(map), 5ms);

    ASSERT_TRUE(drive_until_ok(harness, 5000ms)) << "带坏行的地图不该阻止定位进入 OK";
    // 进入 OK 之后再多跑几拍：`/tf` 有自己的节拍，闸门也只放行 OK/DEGRADED。
    harness.drive(300ms, 10ms, [&harness](std::int64_t) {
        harness.publish_odom(0.0, 0.0);
        harness.publish_cloud(10);
    });
    const auto diagnostics = harness.diagnostics();
    printf("L18 回执(节点): state=%d tf_published=%zu 跳过行数=%zu\n", harness.last_state().state,
           diagnostics.tf_published, diagnostics.skipped_pcd_lines);
    // 计数本身由 map 包的 `SkipsInvalidPoints` / `CountsRowsWithMissingFields` 钉死；
    // 这里补的是「装载成功后节点不会因此罢工」+「节点自己也拿得到这个数字」。
    EXPECT_GT(diagnostics.tf_published, 0U);
    EXPECT_EQ(diagnostics.skipped_pcd_lines, 1U) << "节点必须能报出地图丢了几行";
}

void print_source_receipt(const char* label, const StateRecord& state) {
    std::printf("L4 回执(%s): state=%u source=%u pose_valid=%d x=%.3f reason=%s\n", label,
                static_cast<unsigned>(state.state), static_cast<unsigned>(state.source),
                state.pose_valid ? 1 : 0, state.x, state.reason.c_str());
}

/// L4 用例一：`pose_source: auto` + 雷达位姿新鲜时，外部位姿必须被忽略。
/// 父提交只挡了 `kLidar`，于是三份兵种配置的 `auto` 里外部位姿会把配准结果整条盖掉。
TEST(LocalizationNodeTest, AutoPolicyIgnoresExternalPoseWhileLidarIsFresh) {
    TempMapDir map;
    NodeHarness harness(map_overrides(map), 5ms);
    ASSERT_TRUE(drive_until_ok(harness, 5000ms)) << "雷达链路没有进入 OK";

    // 雷达始终新鲜，同时持续灌外部位姿（x = 5.0，与配准结果明显不同）。
    harness.drive(1000ms, 20ms, [&harness](std::int64_t) {
        harness.publish_odom(0.0, 0.0);
        harness.publish_cloud(10);
        harness.publish_external_pose(5.0, 0.0);
    });

    const auto state = harness.last_state();
    print_source_receipt("auto+雷达新鲜", state);
    EXPECT_EQ(state.state, LocalizationState::STATE_OK);
    EXPECT_EQ(state.source, LocalizationState::SOURCE_SENTRY_LIDAR);
    EXPECT_LT(state.x, 1.0) << "外部位姿把 map → base 直接盖掉了雷达结果";
}

/// L4 用例二：雷达断流超过 `lidar_timeout_sec` 后，`auto` 允许外部位姿接管。
TEST(LocalizationNodeTest, AutoPolicyAcceptsExternalPoseWhenLidarStalls) {
    TempMapDir map;
    NodeHarness harness(map_overrides(map), 5ms);
    ASSERT_TRUE(drive_until_ok(harness, 5000ms)) << "雷达链路没有进入 OK";

    // 停发点云，只留里程计与外部位姿：雷达位姿超时 → 外部位姿接管。
    harness.drive(800ms, 20ms, [&harness](std::int64_t) {
        harness.publish_odom(0.0, 0.0);
        harness.publish_external_pose(5.0, 0.0);
    });

    const auto state = harness.last_state();
    print_source_receipt("auto+雷达断流", state);
    EXPECT_EQ(state.source, LocalizationState::SOURCE_EXTERNAL_ODOM);
    EXPECT_NEAR(state.x, 5.0, 0.05);
    EXPECT_EQ(state.state, LocalizationState::STATE_OK) << "外部位姿新鲜却被判成不可用";
}

/// L4 用例三：雷达恢复后位姿源必须翻回雷达。
/// 父提交把 active_source_ 写死在收外部位姿那一刻，此用例永远是 EXTERNAL_ODOM。
TEST(LocalizationNodeTest, AutoPolicySwitchesBackToLidarWhenItReturns) {
    TempMapDir map;
    NodeHarness harness(map_overrides(map), 5ms);
    ASSERT_TRUE(drive_until_ok(harness, 5000ms)) << "雷达链路没有进入 OK";

    harness.drive(600ms, 20ms, [&harness](std::int64_t) {
        harness.publish_odom(0.0, 0.0);
        harness.publish_external_pose(5.0, 0.0);
    });
    const auto taken_over = harness.last_state();
    print_source_receipt("雷达断流接管", taken_over);
    ASSERT_EQ(taken_over.source, LocalizationState::SOURCE_EXTERNAL_ODOM);

    harness.drive(600ms, 20ms, [&harness](std::int64_t) {
        harness.publish_odom(0.0, 0.0);
        harness.publish_cloud(10);
        harness.publish_external_pose(5.0, 0.0);
    });
    const auto state = harness.last_state();
    print_source_receipt("雷达恢复", state);
    EXPECT_EQ(state.source, LocalizationState::SOURCE_SENTRY_LIDAR);
}

std::vector<rclcpp::Parameter> with_min_source_points(const TempMapDir& map, int minimum) {
    auto overrides = map_overrides(map);
    overrides.emplace_back("min_source_points", minimum);
    return overrides;
}

/// L5 用例：`min_source_points` 必须在构造期就进健康状态机。
/// 父提交在 `health_` 构造之后才 declare 这两条参数，配置写进去也永远不生效，
/// 于是「点数不足」这条判据整条是死的。
TEST(LocalizationNodeTest, MinSourcePointsParameterTakesEffect) {
    TempMapDir map;
    NodeHarness harness(with_min_source_points(map, 200), 5ms);

    // 250 点满足下限：正常进 OK。
    ASSERT_TRUE(drive_until_ok(harness, 5000ms, 250)) << "满足下限的点云没有进入 OK";

    // 降到 30 点（< min_source_points）：必须落 DEGRADED，且理由直指点数。
    harness.drive(800ms, 20ms, [&harness](std::int64_t) {
        harness.publish_odom(0.0, 0.0);
        harness.publish_cloud(30);
    });

    const auto state = harness.last_state();
    std::printf("L5 回执: state=%u pose_valid=%d reason=%s\n", static_cast<unsigned>(state.state),
                state.pose_valid ? 1 : 0, state.reason.c_str());
    EXPECT_EQ(state.state, LocalizationState::STATE_DEGRADED);
    EXPECT_EQ(state.reason, "insufficient source points");
}

/// L9 用例：1 m/s 匀速直线时 `map → odom` 必须是一条水平线。
/// 父提交在配准结束（此处 200 ms 后）才读 odom_to_base_，于是每一次配准都把 v·Δ
/// 记进 `map → odom`，40 次配准能漂出几米；本提交锁内快照后应当保持在毫米级。
TEST(LocalizationNodeTest, MapToOdomJitterUnderConstantVelocity) {
    TempMapDir map;
    NodeHarness harness(map_overrides(map), 200ms);

    std::vector<double> map_to_odom_x;
    harness.drive(std::chrono::milliseconds(8000), 20ms,
                  [&harness, &map_to_odom_x](std::int64_t elapsed_ms) {
                      // 1 m/s 匀速：odom → base 的 x 直接等于经过的秒数。
                      harness.publish_odom(0.001 * static_cast<double>(elapsed_ms), 0.0);
                      if (elapsed_ms % 100 < 20) {
                          harness.publish_cloud(10);
                      }
                      geometry_msgs::msg::TransformStamped transform;
                      if (harness.lookup_map_to_odom(&transform)) {
                          map_to_odom_x.push_back(transform.transform.translation.x);
                      }
                  });

    ASSERT_GE(map_to_odom_x.size(), 20U) << "没有采到足够多的 map → odom 样本";
    const auto bounds = std::minmax_element(map_to_odom_x.begin(), map_to_odom_x.end());
    const double spread = *bounds.second - *bounds.first;
    const auto registrations = harness.diagnostics().align_runs;
    std::printf("L9 回执: samples=%zu align_runs=%llu map_to_odom x=[%.4f, %.4f] 极差=%.4fm\n",
                map_to_odom_x.size(), static_cast<unsigned long long>(registrations), *bounds.first,
                *bounds.second, spread);
    EXPECT_GE(registrations, 15U) << "配准次数太少，样本不足以下结论";
    EXPECT_LT(spread, 0.005) << "map → odom 在匀速直行时被配准时刻差拉出了漂移";
}

/// L6 用例（TF 时间戳）：`map → odom` 只在配准成功那一刻变化，所以它的 header.stamp
/// 必须是「被消费点云的时刻」，不是发布时刻。父提交写 `now()`，于是 20 Hz 的 TF 流
/// 每个样本都指着当前时刻，而位姿其实来自最多 200 ms 前的点云。
TEST(LocalizationNodeTest, TransformIsStampedWithTheConsumedCloudTime) {
    TempMapDir map;
    NodeHarness harness(map_overrides(map), 20ms);
    ASSERT_TRUE(drive_until_ok(harness, std::chrono::milliseconds(6000))) << "预热没进 OK";

    std::vector<std::int64_t> cloud_stamps;
    std::vector<std::int64_t> tf_stamps;
    harness.drive(std::chrono::milliseconds(3000), 20ms,
                  [&harness, &cloud_stamps, &tf_stamps](std::int64_t elapsed_ms) {
                      harness.publish_odom(0.0, 0.0);
                      if (elapsed_ms % 100 < 20) {
                          harness.publish_cloud(10);
                          cloud_stamps.push_back(harness.last_cloud_stamp_ns());
                      }
                      geometry_msgs::msg::TransformStamped transform;
                      if (harness.lookup_map_to_odom(&transform)) {
                          tf_stamps.push_back(rclcpp::Time(transform.header.stamp).nanoseconds());
                      }
                  });

    ASSERT_FALSE(tf_stamps.empty()) << "3 s 内一次 map → odom 都没发";
    ASSERT_FALSE(cloud_stamps.empty()) << "3 s 内一帧点云都没发";
    std::sort(tf_stamps.begin(), tf_stamps.end());
    ASSERT_GE(std::unique(tf_stamps.begin(), tf_stamps.end()) - tf_stamps.begin(), 5)
        << "TF 时间戳几乎没有变化，说明配准没在跑";

    // 每个 TF 时间戳都必须能对上某一帧点云的时刻；对不上就说明它用的是发布时刻。
    std::sort(cloud_stamps.begin(), cloud_stamps.end());
    // 采样开始前那次配准留下的戳（上一帧点云早已不在本次记录里）不属于本窗口，
    // 只从第一帧记录点云起算。
    tf_stamps.erase(std::remove_if(tf_stamps.begin(), tf_stamps.end(),
                                   [&cloud_stamps](std::int64_t stamp) {
                                       return stamp < cloud_stamps.front();
                                   }),
                    tf_stamps.end());
    const std::size_t in_window = tf_stamps.size();
    std::int64_t worst_gap_ns = 0;
    for (const std::int64_t stamp : tf_stamps) {
        const auto it = std::lower_bound(cloud_stamps.begin(), cloud_stamps.end(), stamp);
        std::int64_t gap = std::numeric_limits<std::int64_t>::max();
        if (it != cloud_stamps.end()) {
            gap = *it - stamp;
        }
        if (it != cloud_stamps.begin()) {
            gap = std::min(gap, stamp - *(it - 1));
        }
        worst_gap_ns = std::max(worst_gap_ns, gap);
    }
    ASSERT_GT(in_window, 0U) << "采样窗口里一个 TF 样本都没有";
    std::printf("L6 回执(TF 戳): samples=%zu 不同戳=%zu TF 与最近点云戳最大偏差=%.3fms\n",
                in_window,
                static_cast<std::size_t>(std::unique(tf_stamps.begin(), tf_stamps.end()) -
                                         tf_stamps.begin()),
                1e-6 * static_cast<double>(worst_gap_ns));
    EXPECT_LT(1e-6 * static_cast<double>(worst_gap_ns), 2.0)
        << "TF 时间戳对不上任何一帧被消费的点云：它用的是发布时刻";
}

/// L6 用例（丢帧）：配准要 200 ms，期间到达的点云必须留给下一轮，而不是在配准结束时
/// 被当成「已消费」丢掉。父提交在放锁之后才无条件清 `has_cloud_`，于是这一帧消失。
/// 帧的身份用点数表示（配准替身记下每次收到的源点数），这样才分得清「它被配了」还是
/// 「配的还是上一帧」。
TEST(LocalizationNodeTest, CloudArrivingDuringAlignmentIsNotDropped) {
    TempMapDir map;
    NodeHarness harness(map_overrides(map), 200ms);
    ASSERT_TRUE(drive_until_ok(harness, std::chrono::milliseconds(8000))) << "预热没进 OK";

    // 预热用的是 10 点云。配准替身在进入时记一笔，所以「日志刚出现一条」＝这次配准
    // 正在跑（它要睡 200 ms），此刻发一帧 77 点的特殊帧，必然落在这次配准期间。
    harness.align_log()->clear();
    ASSERT_TRUE(
        harness.wait_until([&harness]() { return !harness.align_log()->source_sizes().empty(); },
                           std::chrono::milliseconds(2000)))
        << "配准节拍停了";
    harness.publish_cloud(77);
    const auto consumed_before = harness.diagnostics().align_runs;
    harness.drive(std::chrono::milliseconds(600), 5ms,
                  [&harness](std::int64_t) { harness.publish_odom(0.0, 0.0); });
    const auto consumed_after = harness.diagnostics().align_runs;

    const auto sizes = harness.align_log()->source_sizes();
    const bool special_frame_aligned = std::find(sizes.begin(), sizes.end(), 77) != sizes.end();
    std::printf("L6 回执(丢帧): 配准期间到达 1 帧(77 点) → 600ms 内配准 %llu 次，"
                "其中源帧点数={",
                static_cast<unsigned long long>(consumed_after - consumed_before));
    for (const auto size : sizes) {
        std::printf("%zu,", size);
    }
    std::printf("} 该帧是否被消费=%s\n", special_frame_aligned ? "是" : "否");
    EXPECT_TRUE(special_frame_aligned)
        << "配准期间到达的那一帧被丢掉了：之后 600 ms 的配准用的还是上一帧";
}

/// L7：状态消息里的协方差必须如实反映配准质量，而不是整块 0（审计 L7）。
///
/// 父提交 `to_msg_state()` 里写的是 `message.pose.covariance.fill(0.0)`。在
/// `PoseWithCovariance` 的语义里 0 方差意味着「这个位姿绝对准确」——任何按协方差加权的
/// 下游（导航的 EKF、融合节点）都会把配准结果当成无限可信的观测，同时「其实不知道」这件事
/// 被彻底藏起来：下游既无法据此降权，也没有任何字段看得出这件事。
TEST(LocalizationNodeTest, StateCovarianceReflectsRegistrationQuality) {
    TempMapDir map;
    NodeHarness harness(map_overrides(map), 5ms);

    // 还没有任何一帧配准（INITIALIZING）：只能声明「不知道有多好」。
    ASSERT_TRUE(harness.wait_until([&harness]() { return harness.state_count() > 2; },
                                   std::chrono::milliseconds(3000)))
        << "节点没有发布状态";
    const auto initial = harness.last_state();
    std::printf("L7 回执(INITIALIZING): state=%u cov[x]=%.3f cov[yaw]=%.3f\n", initial.state,
                initial.covariance[0], initial.covariance[3 * 7]);
    for (std::size_t dof = 0; dof < 6; ++dof) {
        EXPECT_DOUBLE_EQ(initial.covariance[dof * 7], 1000.0) << "dof=" << dof;
    }

    ASSERT_TRUE(drive_until_ok(harness, std::chrono::milliseconds(5000))) << "雷达链路没有进入 OK";
    const auto ok = harness.last_state();
    // 配准替身给的是残差 0 / 内点率 1：σ 落到 1 cm 下限，方差 1e-4 —— 既不是 0，
    // 也不是「无信息」的 1e3。
    const double expected = 0.01 * 0.01;
    std::printf("L7 回执(OK): state=%u cov[x]=%.6f cov[yaw]=%.6f cov[xy]=%.6f\n", ok.state,
                ok.covariance[0], ok.covariance[3 * 7], ok.covariance[1]);
    EXPECT_NEAR(ok.covariance[0], expected, 1e-9);
    EXPECT_NEAR(ok.covariance[3 * 7], expected, 1e-9);
    EXPECT_DOUBLE_EQ(ok.covariance[1], 0.0) << "非对角项凭空表示了两个方向的相关性";
}

/// L7 第二面：外部位姿源没有可测的配准质量，协方差必须退回「不知道有多好」，
/// 而不是沿用上一帧雷达配准的数值——那会让外部位姿看起来和雷达一样准（D38 同一条口径）。
TEST(LocalizationNodeTest, ExternalSourceCovarianceDoesNotReuseLidarQuality) {
    TempMapDir map;
    NodeHarness harness(map_overrides(map), 5ms);
    ASSERT_TRUE(drive_until_ok(harness, std::chrono::milliseconds(5000))) << "雷达链路没有进入 OK";

    harness.drive(std::chrono::milliseconds(800), 20ms, [&harness](std::int64_t) {
        harness.publish_odom(0.0, 0.0);
        harness.publish_external_pose(5.0, 0.0);
    });

    const auto state = harness.last_state();
    std::printf("L7 回执(外部位姿): source=%u cov[x]=%.3f\n", state.source, state.covariance[0]);
    ASSERT_EQ(state.source, LocalizationState::SOURCE_EXTERNAL_ODOM);
    EXPECT_DOUBLE_EQ(state.covariance[0], 1000.0);
}

} // namespace
} // namespace diesel::localization_ros::test_support

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
