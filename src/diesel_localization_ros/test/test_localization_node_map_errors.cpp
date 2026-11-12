// Copyright 2026 ZQU-Foray
//
// 节点级测试：地图不可用时「一帧 TF 都不发」（审计 L2 / decision.md D36）。
// 父提交上 `load_map()` 只打一行 ERROR 就 return，健康状态停在 `INITIALIZING`，
// 而 `publish_tf()` 的闸门只挡 `LOST`，于是没有地图的节点照样以 20 Hz 广播单位阵
// `map → odom`。三条用例分别覆盖三种地图失败成因，断言完全一致：
// state 转 `LOST`、`pose_valid == false`、`reason` 以 `map unavailable:` 开头、
// `/tf` 上没有任何 `map → odom`（既看节点自己的计数，也看订阅侧的 TF 树）。

#include "localization_node_test_support.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <vector>

namespace diesel::localization_ros::test_support {
namespace {

using std::chrono::milliseconds;

using diesel_localization_ros::msg::LocalizationState;

/// 一份头部合法、但二进制段被截断的 PCD：`POINTS` 声明 1000 点，实际只有 3 个点。
/// sha256 与 manifest 一致（文件被人为写成这样，不是传输损坏），所以校验挡不住它。
std::string truncated_binary_pcd() {
    std::string text = "VERSION 0.7\n"
                       "FIELDS x y z\n"
                       "SIZE 4 4 4\n"
                       "TYPE F F F\n"
                       "COUNT 1 1 1\n"
                       "WIDTH 1000\n"
                       "HEIGHT 1\n"
                       "POINTS 1000\n"
                       "DATA binary\n";
    for (int i = 0; i < 3; ++i) {
        const float xyz[3] = {static_cast<float>(i), 0.0F, 0.0F};
        text.append(reinterpret_cast<const char*>(xyz), sizeof(xyz));
    }
    return text;
}

/// 头部合法、但声明了 20 亿点的 ascii PCD：父提交按声明值 reserve、再把
/// 「实际只有 1 行」当成正常文件接受（audit L10）。上限与「实际行数」校验落地后
/// 它必须在 `load_pcd` 里就被拒。
std::string oversized_point_count_pcd() {
    return "VERSION 0.7\n"
           "FIELDS x y z\n"
           "SIZE 4 4 4\n"
           "TYPE F F F\n"
           "COUNT 1 1 1\n"
           "WIDTH 2000000000\n"
           "HEIGHT 1\n"
           "POINTS 2000000000\n"
           "DATA ascii\n"
           "0 0 0\n";
}

/// 地图不可用时的共同断言。`fragment` 用来钉住「哪一种成因」，避免三条用例互相顶替。
void expect_map_unavailable(NodeHarness& harness, const std::string& fragment) {
    // 必须先让里程计在流：`publish_tf()` 还有 `!has_odom_` 这道前置 return，
    // 不给里程计时父提交也发不出 TF，用例就测不到闸门本身。
    harness.drive(milliseconds(1000), milliseconds(5),
                  [&harness](std::int64_t) { harness.publish_odom(0.0, 0.0); });
    ASSERT_TRUE(
        harness.wait_until([&harness]() { return harness.state_count() >= 5; }, milliseconds(5000)))
        << "地图不可用时状态话题仍在按节拍发布（下游要靠它读到 LOST）";
    const StateRecord state = harness.last_state();
    EXPECT_EQ(state.state, LocalizationState::STATE_LOST) << "state=" << state.state;
    EXPECT_FALSE(state.pose_valid) << "LOST 下 pose_valid 必须为 false";
    EXPECT_NE(state.reason.find("map unavailable:"), std::string::npos)
        << "reason=" << state.reason;
    EXPECT_NE(state.reason.find(fragment), std::string::npos) << "reason=" << state.reason;

    // 两个独立观测：节点自报的 TF 发送次数，以及订阅侧 TF 树里到底有没有这条变换。
    EXPECT_EQ(harness.diagnostics().tf_published, 0U) << "地图不可用时不得发送 map → odom";
    geometry_msgs::msg::TransformStamped transform;
    EXPECT_FALSE(harness.lookup_map_to_odom(&transform)) << "TF 树里仍然查到了 map → odom";
}

TEST(LocalizationNodeMapErrors, MissingManifestSendsNoTransform) {
    NodeHarness harness({}, milliseconds(5));
    expect_map_unavailable(harness, "map_manifest 未设置");
}

TEST(LocalizationNodeMapErrors, Sha256MismatchSendsNoTransform) {
    TempMapDir map;
    // 只改 pcd、不刷 manifest：地图被改动而清单没跟上。
    map.tamper_pcd(make_pcd_text(4));
    NodeHarness harness({rclcpp::Parameter("map_manifest", map.manifest_path())}, milliseconds(5));
    expect_map_unavailable(harness, "sha256");
}

TEST(LocalizationNodeMapErrors, TruncatedPcdSendsNoTransform) {
    TempMapDir map(truncated_binary_pcd());
    NodeHarness harness({rclcpp::Parameter("map_manifest", map.manifest_path())}, milliseconds(5));
    expect_map_unavailable(harness, "PCD");
}

TEST(LocalizationNodeMapErrors, OversizedPointCountSendsNoTransform) {
    TempMapDir map;
    map.rewrite_pcd(oversized_point_count_pcd());
    NodeHarness harness({rclcpp::Parameter("map_manifest", map.manifest_path())}, milliseconds(5));
    expect_map_unavailable(harness, "上限");
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
