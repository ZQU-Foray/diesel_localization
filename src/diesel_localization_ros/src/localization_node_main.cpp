// Copyright 2026 ZQU-Foray

#include "diesel/localization_ros/localization_node.hpp"

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cstddef>
#include <memory>
#include <thread>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<diesel::localization_ros::LocalizationNode>(rclcpp::NodeOptions{});

    // 单线程 executor 下 5 Hz 配准（单次可达上百毫秒）会与 200 Hz 里程计互相阻塞，
    // 里程计回调被饿死后健康状态会误报 odometry timeout（decision.md D30）。
    // 线程数取硬件并发但不低于 2、不超过 4：回调组已经按「谁阻塞谁」分好了，
    // 再多线程只是让几条毫秒级回调互相插队，没有额外收益。
    const auto hardware_threads = static_cast<std::size_t>(std::thread::hardware_concurrency());
    const std::size_t thread_count = std::clamp<std::size_t>(hardware_threads, 2, 4);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, thread_count);
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
