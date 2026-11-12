// Copyright 2026 ZQU-Foray

#include "diesel/localization/health.hpp"

#include <gtest/gtest.h>

namespace diesel::localization {
namespace {

constexpr Timestamp kSecond = 1'000'000'000;

RegistrationSample make_sample(Timestamp stamp_ns, bool valid = true, double mean_residual_m = 0.05,
                               double inlier_ratio = 0.9) {
    RegistrationSample sample;
    sample.stamp_ns = stamp_ns;
    sample.valid = valid;
    sample.mean_residual_m = mean_residual_m;
    sample.inlier_ratio = inlier_ratio;
    return sample;
}

HealthConfig test_config() {
    HealthConfig config;
    config.warmup_frames = 2;
    config.recover_frames = 2;
    config.lost_timeout_sec = 0.5;
    config.odom_timeout_sec = 0.5;
    config.degraded_hold_sec = 0.0;
    return config;
}

TEST(HealthMonitor, WarmsUpThenStaysOk) {
    HealthMonitor monitor(test_config());
    const Timestamp start = kSecond;

    EXPECT_EQ(monitor.update(make_sample(start), start, start).state, HealthState::kInitializing);
    EXPECT_EQ(
        monitor
            .update(make_sample(start + kSecond / 10), start + kSecond / 10, start + kSecond / 10)
            .state,
        HealthState::kOk);

    const HealthStatus status = monitor.update(make_sample(start + 2 * kSecond / 10),
                                               start + 2 * kSecond / 10, start + 2 * kSecond / 10);
    EXPECT_EQ(status.state, HealthState::kOk);
    EXPECT_GT(status.confidence, 0.5);
}

TEST(HealthMonitor, DegradesAndRecovers) {
    HealthMonitor monitor(test_config());
    const Timestamp start = kSecond;
    monitor.update(make_sample(start), start, start);
    monitor.update(make_sample(start + 1), start + 1, start + 1);

    const Timestamp degraded_at = start + 2;
    const HealthStatus degraded =
        monitor.update(make_sample(degraded_at, true, 0.20, 0.5), degraded_at, degraded_at);
    EXPECT_EQ(degraded.state, HealthState::kDegraded);
    EXPECT_LT(degraded.confidence, 1.0);

    EXPECT_EQ(monitor.update(make_sample(degraded_at + 1), degraded_at + 1, degraded_at + 1).state,
              HealthState::kDegraded);
    EXPECT_EQ(monitor.update(make_sample(degraded_at + 2), degraded_at + 2, degraded_at + 2).state,
              HealthState::kOk);
}

TEST(HealthMonitor, AcceptsResidualAboveLegacyThreshold) {
    // 0.25 m 体素先验图上，配准收敛后的平均残差实测约 0.10 m，
    // 旧阈值（0.10）把完美配准判成不合格，导致永远无法完成 warmup。
    HealthMonitor monitor(test_config());
    const Timestamp start = kSecond;

    monitor.update(make_sample(start, true, 0.12, 0.95), start, start);
    const HealthStatus status = monitor.update(make_sample(start + kSecond / 10, true, 0.12, 0.95),
                                               start + kSecond / 10, start + kSecond / 10);
    EXPECT_EQ(status.state, HealthState::kOk);
}

TEST(HealthMonitor, LostAfterNoValidRegistration) {
    HealthMonitor monitor(test_config());
    const Timestamp start = kSecond;
    monitor.update(make_sample(start), start, start);
    monitor.update(make_sample(start + kSecond / 10), start + kSecond / 10, start + kSecond / 10);

    EXPECT_EQ(
        monitor.update(make_sample(start + kSecond, false), start + kSecond, start + kSecond).state,
        HealthState::kDegraded);
    EXPECT_EQ(monitor
                  .update(make_sample(start + 2 * kSecond, false), start + 2 * kSecond,
                          start + 2 * kSecond)
                  .state,
              HealthState::kLost);

    // LOST 是粘滞的：好样本也不能静默恢复。
    EXPECT_EQ(monitor
                  .update(make_sample(start + 2 * kSecond + kSecond / 5),
                          start + 2 * kSecond + kSecond / 5, start + 2 * kSecond + kSecond / 5)
                  .state,
              HealthState::kLost);

    monitor.reset();
    EXPECT_EQ(monitor.state(), HealthState::kInitializing);
}

TEST(HealthMonitor, OdomTimeoutIsLost) {
    HealthMonitor monitor(test_config());
    // 两个周期过去、里程计停在更早的那一帧：这是「收到过但现在没有新帧」。
    const Timestamp now = 2 * kSecond;
    // 原来的断言是 reason == "odometry timeout"。那句话把「我算太慢、帧还在队列里」
    // 和「里程计真的没数据」写成同一句，现场照着它去查里程计驱动是查不到的（L1/Q1）。
    const HealthStatus status = monitor.update(make_sample(now), now, kSecond);

    EXPECT_EQ(status.state, HealthState::kLost);
    EXPECT_DOUBLE_EQ(status.confidence, 0.0);
    EXPECT_EQ(status.reason, "odometry stale: 1.000s (no samples)");
}

TEST(HealthMonitor, OdomBacklogIsReportedAsProcessingBacklog) {
    HealthMonitor monitor(test_config());
    const Timestamp now = kSecond;
    // 里程计新鲜但队列里积压了 0.45 s（> 0.5 * odom_timeout_sec）：说明卡的是本节点，
    // 不是传感器。这是 L1 那条「odometry timeout」误诊的直接修复点（decision.md D30）。
    const HealthStatus status = monitor.update(make_sample(now), now, now - 4 * kSecond / 5, 0.45);

    EXPECT_EQ(status.state, HealthState::kLost);
    EXPECT_EQ(status.reason, "odometry stale: 0.800s (processing backlog 0.450s)");
}

TEST(HealthMonitor, SmallBacklogDoesNotChangeTheReason) {
    HealthMonitor monitor(test_config());
    const Timestamp now = kSecond;
    // 积压只有 50 ms：仍属正常抖动，不该被写成 backlog 故障。
    const HealthStatus status = monitor.update(make_sample(now), now, now - 4 * kSecond / 5, 0.05);

    EXPECT_EQ(status.reason, "odometry stale: 0.800s (no samples)");
}

TEST(HealthMonitor, EnterLostKeepsTheFirstReason) {
    HealthMonitor monitor(test_config());
    const Timestamp start = kSecond;
    EXPECT_EQ(monitor.update(make_sample(start), start, start).state, HealthState::kInitializing);

    // 地图加载失败这类故障没有配准帧可喂，只能显式置 LOST（L2）。
    const HealthStatus lost = monitor.enter_lost("map unavailable: manifest missing", start);
    EXPECT_EQ(lost.state, HealthState::kLost);
    EXPECT_EQ(lost.reason, "map unavailable: manifest missing");
    EXPECT_DOUBLE_EQ(lost.confidence, 0.0);

    // 再置一次不许改掉第一次的 reason：先到的成因才是现场要看的那个。
    const HealthStatus again = monitor.enter_lost("another reason", start + kSecond / 10);
    EXPECT_EQ(again.state, HealthState::kLost);
    EXPECT_EQ(again.reason, "map unavailable: manifest missing");
}

TEST(HealthMonitor, LostBeforeFirstSuccessRecoversFromStartup) {
    HealthMonitor monitor(test_config());
    const Timestamp start = kSecond;
    // 启动早期 TF 查不到，一帧配准都喂不进来，于是被显式判成 LOST（L3/L4 的现场形态）。
    monitor.enter_lost("tf unavailable: base_footprint<-body", start);
    EXPECT_FALSE(monitor.ever_ok());

    // TF 补上、合格帧开始到来：连续 warmup_frames(2) 帧就自恢复，
    // 而不是像旧行为那样一直冻在 LOST 直到有人手工 reset（decision.md D57）。
    const Timestamp first = start + kSecond / 5;
    EXPECT_EQ(monitor.update(make_sample(first), first, first).state, HealthState::kLost);

    const Timestamp second = start + 2 * kSecond / 5;
    const HealthStatus recovered = monitor.update(make_sample(second), second, second);
    EXPECT_EQ(recovered.state, HealthState::kOk);
    EXPECT_EQ(recovered.reason, "recovered during startup");
    EXPECT_TRUE(monitor.ever_ok());
}

TEST(HealthMonitor, EnterLostIsStickyAfterFirstSuccess) {
    HealthMonitor monitor(test_config());
    const Timestamp start = kSecond;
    // 先正常跑起来（warmup_frames=2）。
    monitor.update(make_sample(start), start, start);
    const Timestamp warm = start + kSecond / 5;
    EXPECT_EQ(monitor.update(make_sample(warm), warm, warm).state, HealthState::kOk);
    ASSERT_TRUE(monitor.ever_ok());

    // 成功过之后再丢，就不是启动期问题了：合格样本不许把它拉回来，只能 reset()。
    monitor.enter_lost("map unavailable: manifest missing", start + 2 * kSecond / 5);
    const Timestamp now = start + 3 * kSecond / 5;
    EXPECT_EQ(monitor.update(make_sample(now), now, now).state, HealthState::kLost);

    const Timestamp later = now + kSecond / 5;
    const HealthStatus status = monitor.update(make_sample(later), later, later);
    EXPECT_EQ(status.state, HealthState::kLost);
    EXPECT_DOUBLE_EQ(status.confidence, 0.0);
}

TEST(HealthMonitor, SmallTimestampInversionIsTolerated) {
    HealthMonitor monitor(test_config());
    const Timestamp start = kSecond;
    // 里程计时间戳比评估时间新 20 ms（小于 200 ms 容差）：不应误判为断流。
    EXPECT_EQ(monitor.update(make_sample(start), start, start + kSecond / 50).state,
              HealthState::kInitializing);
    EXPECT_EQ(monitor
                  .update(make_sample(start + kSecond / 10), start + kSecond / 10,
                          start + kSecond / 10 + kSecond / 50)
                  .state,
              HealthState::kOk);
}

TEST(HealthMonitor, LargeTimestampInversionStillTimesOut) {
    HealthMonitor monitor(test_config());
    const Timestamp start = kSecond;
    // 反序 500 ms 超出容差，仍按时间基准异常处理。
    // 时间戳反序超出容差时 odom_age 是 +inf，reason 必须说「根本没收到」而不是 stale：
    // 队列表里那帧的时间戳在「未来」，用它算出的积压没有意义。
    const HealthStatus status = monitor.update(make_sample(start), start, start + kSecond / 2);
    EXPECT_EQ(status.state, HealthState::kLost);
    EXPECT_EQ(status.reason, "no odometry received");
}

TEST(PoseSourceSelection, FollowsPolicy) {
    EXPECT_EQ(select_pose_source(PoseSourcePolicy::kLidar, false, true), PoseSource::kUnknown);
    EXPECT_EQ(select_pose_source(PoseSourcePolicy::kExternal, true, false), PoseSource::kUnknown);
    EXPECT_EQ(select_pose_source(PoseSourcePolicy::kAuto, false, true), PoseSource::kExternalOdom);
    EXPECT_EQ(select_pose_source(PoseSourcePolicy::kAuto, true, true), PoseSource::kSentryLidar);
}

} // namespace
} // namespace diesel::localization
