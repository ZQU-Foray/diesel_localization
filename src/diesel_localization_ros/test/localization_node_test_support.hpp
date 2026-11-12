// Copyright 2026 ZQU-Foray
//
// 节点级测试夹具：临时先验地图、参数注入、消息构造、执行器与状态记录。
// 只被 test_localization_node*.cpp 使用，不进入安装产物（decision.md D29）。

#ifndef DIESEL_LOCALIZATION_ROS_TEST_LOCALIZATION_NODE_TEST_SUPPORT_HPP_
#define DIESEL_LOCALIZATION_ROS_TEST_LOCALIZATION_NODE_TEST_SUPPORT_HPP_

#include "diesel/localization/sha256.hpp"
#include "diesel/localization_ros/localization_node.hpp"
#include "diesel_localization_ros/msg/dynamic_obstacle_array.hpp"
#include "diesel_localization_ros/msg/localization_state.hpp"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace diesel::localization_ros::test_support {

namespace fs = std::filesystem;
using namespace std::chrono_literals;

using diesel::localization::sha256_file;
using diesel::localization_map::Aligner;
using diesel::localization_map::AlignResult;
using diesel::localization_map::PointCloud;

/// 单点 PCD 文本（ascii）。点数由调用方给定，用来精确控制 source_points。
inline std::string make_pcd_text(std::size_t point_count) {
    std::string text = "VERSION 0.7\n"
                       "FIELDS x y z\n"
                       "SIZE 4 4 4\n"
                       "TYPE F F F\n"
                       "COUNT 1 1 1\n"
                       "WIDTH " +
                       std::to_string(point_count) +
                       "\n"
                       "HEIGHT 1\n"
                       "POINTS " +
                       std::to_string(point_count) +
                       "\n"
                       "DATA ascii\n";
    for (std::size_t index = 0; index < point_count; ++index) {
        text += std::to_string(0.5 * static_cast<double>(index)) + " 0 0\n";
    }
    return text;
}

/// 先验地图目录：默认写出一份自洽的 pcd + pgm + manifest。
/// `rewrite_pcd` 会同步刷新 manifest 里的 sha256（自洽），`tamper_pcd` 故意不刷新（sha 不符）。
class TempMapDir {
  public:
    TempMapDir() : TempMapDir(make_pcd_text(3)) {}

    explicit TempMapDir(std::string pcd_text) {
        const auto suffix = std::chrono::steady_clock::now().time_since_epoch().count();
        path_ = fs::temp_directory_path() / ("diesel_localization_test_" + std::to_string(suffix));
        fs::create_directories(path_);
        // 2x2 灰度图：只要求尺寸与 manifest 一致，聚类不使用栅格。
        std::ofstream grid(path_ / "map.pgm", std::ios::binary);
        grid << "P5\n2 2\n255\n";
        const std::vector<unsigned char> pixels{255, 255, 255, 255};
        grid.write(reinterpret_cast<const char*>(pixels.data()),
                   static_cast<std::streamsize>(pixels.size()));
        grid.close();
        write_pcd(std::move(pcd_text));
    }

    ~TempMapDir() {
        std::error_code code;
        fs::remove_all(path_, code);
    }

    TempMapDir(const TempMapDir&) = delete;
    TempMapDir& operator=(const TempMapDir&) = delete;

    std::string manifest_path() const { return (path_ / "manifest.yaml").string(); }

    /// 重写 pcd 并按新内容刷新 manifest 的 sha256：地图自洽，只是内容变了。
    void rewrite_pcd(const std::string& pcd_text) { write_pcd(pcd_text); }

    /// 只改 pcd 不刷 manifest：制造「地图被改动但清单没更新」。
    void tamper_pcd(const std::string& pcd_text) { write_file(path_ / "map.pcd", pcd_text); }

  private:
    static void write_file(const fs::path& path, const std::string& content) {
        std::ofstream out(path, std::ios::binary);
        out.write(content.data(), static_cast<std::streamsize>(content.size()));
    }

    void write_pcd(std::string pcd_text) {
        write_file(path_ / "map.pcd", pcd_text);
        std::string error;
        const std::string pcd_sha = sha256_file(path_ / "map.pcd", &error);
        const std::string grid_sha = sha256_file(path_ / "map.pgm", &error);
        std::string manifest = "version: 2027_3v3_test\n"
                               "frame_id: map\n"
                               "origin: [0.0, 0.0, 0.0]\n"
                               "resolution: 0.5\n"
                               "width: 2\n"
                               "height: 2\n"
                               "pcd_file: map.pcd\n"
                               "grid_file: map.pgm\n"
                               "pcd_sha256: " +
                               pcd_sha +
                               "\n"
                               "grid_sha256: " +
                               grid_sha +
                               "\n"
                               "created_at: 2026-10-25T00:00:00+08:00\n"
                               "source: sim\n";
        write_file(path_ / "manifest.yaml", manifest);
    }

    fs::path path_;
};

/// 只做「耗时」的配准后端：睡眠固定时长后把初值原样返回。
/// 用它把配准耗时变成可控变量，从而单独观察执行器与健康判定（L1 验收）。
/// 记录每次配准收到的源点数。测试用它给点云帧「编身份」：审计 L6 要证明配准期间
/// 到达的那一帧确实进了下一轮配准，光看次数分不清是「它被配了」还是「老帧被配了」。
class AlignCallLog {
  public:
    void record(std::size_t source_points) {
        std::lock_guard<std::mutex> lock(mutex_);
        source_sizes_.push_back(source_points);
    }

    std::vector<std::size_t> source_sizes() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return source_sizes_;
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        source_sizes_.clear();
    }

  private:
    mutable std::mutex mutex_;
    std::vector<std::size_t> source_sizes_;
};

class FixedDelayAligner : public Aligner {
  public:
    FixedDelayAligner(std::chrono::microseconds delay, std::shared_ptr<AlignCallLog> log)
        : delay_(delay), log_(std::move(log)) {}

    AlignResult align(const PointCloud& source, const PointCloud& /*target*/,
                      const Eigen::Isometry3d& initial_guess) override {
        if (log_) {
            log_->record(source.size());
        }
        std::this_thread::sleep_for(delay_);
        AlignResult result;
        result.converged = true;
        result.transform = initial_guess;
        result.fitness = 1.0;
        result.mean_residual_m = 0.0;
        result.inlier_ratio = source.empty() ? 0.0 : 1.0;
        result.iterations = 1;
        return result;
    }

  private:
    std::chrono::microseconds delay_;
    std::shared_ptr<AlignCallLog> log_;
};

/// 用 CPU 把配准时长「烧」出来，而不是 `sleep`。
///
/// 审计 L16 要证明「算法路径不再独占执行器」：`FixedDelayAligner` 睡着的时候会让出 CPU，
/// 单线程执行器也能在那段时间里跑其他回调，测不出真实的争用。这里用忙等占住一个核心，
/// 等价于 GICP 真在算的形态。
class BusyAligner : public Aligner {
  public:
    BusyAligner(std::chrono::microseconds duration, std::shared_ptr<AlignCallLog> log)
        : duration_(duration), log_(std::move(log)) {}

    AlignResult align(const PointCloud& source, const PointCloud& target,
                      const Eigen::Isometry3d& initial_guess) override {
        if (log_) {
            log_->record(source.size());
        }
        volatile double sink = 0.0;
        const auto begin = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - begin < duration_) {
            for (int index = 0; index < 256; ++index) {
                sink = sink + static_cast<double>(index) * 1e-9;
            }
        }
        (void)sink;
        (void)target;
        AlignResult result;
        result.converged = true;
        result.transform = initial_guess;
        result.mean_residual_m = 0.0;
        result.inlier_ratio = source.empty() ? 0.0 : 1.0;
        result.iterations = 1;
        return result;
    }

  private:
    std::chrono::microseconds duration_;
    std::shared_ptr<AlignCallLog> log_;
};

/// 记录到的状态消息（只留断言要用的字段）。
struct StateRecord {
    std::uint8_t state{0};
    std::uint8_t source{0};
    bool pose_valid{false};
    std::string reason;
    double x{0.0};
    std::int64_t stamp_ns{0};
    /// 6×6 位置/姿态协方差的 36 个值（行主序）。审计 L7 要求位姿与可信度一起发布。
    std::array<double, 36> covariance{};
};

inline sensor_msgs::msg::PointCloud2 make_cloud_msg(const std::string& frame,
                                                    const builtin_interfaces::msg::Time& stamp,
                                                    std::size_t count) {
    sensor_msgs::msg::PointCloud2 message;
    message.header.frame_id = frame;
    message.header.stamp = stamp;
    message.height = 1;
    message.width = static_cast<std::uint32_t>(count);
    message.is_dense = true;
    message.is_bigendian = false;
    message.point_step = 12;
    message.row_step = message.point_step * message.width;
    const char* names[] = {"x", "y", "z"};
    for (std::uint32_t index = 0; index < 3U; ++index) {
        sensor_msgs::msg::PointField field;
        field.name = names[index];
        field.offset = index * 4U;
        field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field.count = 1;
        message.fields.push_back(field);
    }
    message.data.resize(message.row_step);
    auto* values = reinterpret_cast<float*>(message.data.data());
    for (std::size_t index = 0; index < count; ++index) {
        values[3 * index] = 0.5F * static_cast<float>(index);
        values[3 * index + 1] = 0.0F;
        values[3 * index + 2] = 0.0F;
    }
    return message;
}

inline nav_msgs::msg::Odometry make_odom_msg(const std::string& child_frame,
                                             const builtin_interfaces::msg::Time& stamp, double x,
                                             double y) {
    nav_msgs::msg::Odometry message;
    message.header.frame_id = "odom";
    message.header.stamp = stamp;
    message.child_frame_id = child_frame;
    message.pose.pose.position.x = x;
    message.pose.pose.position.y = y;
    message.pose.pose.orientation.w = 1.0;
    return message;
}

inline geometry_msgs::msg::PoseWithCovarianceStamped
make_external_pose_msg(const std::string& frame, const builtin_interfaces::msg::Time& stamp,
                       double x, double y) {
    geometry_msgs::msg::PoseWithCovarianceStamped message;
    message.header.frame_id = frame;
    message.header.stamp = stamp;
    message.pose.pose.position.x = x;
    message.pose.pose.position.y = y;
    message.pose.pose.orientation.w = 1.0;
    return message;
}

/// 节点 + 观测节点的组合：同一个多线程执行器上跑，后台线程 spin。
/// 这里刻意用 `MultiThreadedExecutor`：节点在生产入口用的就是它，
/// 测试若用单线程执行器，就测不出回调组划分是否真的解开了阻塞（L1）。
class NodeHarness {
  public:
    /// `threads` 传 1 表示用**单线程执行器**起来：那是 L1/L16 之前的产线配置，测试里用它
    /// 当对照组，证明「算法路径不再独占执行器」这件事真的由执行器与回调组决定。
    /// 传 0 用 rclcpp 默认（按硬件并发数）。
    /// `busy_align` 为真时用忙等的 `BusyAligner`（占住 CPU），否则用 `FixedDelayAligner`。
    NodeHarness(std::vector<rclcpp::Parameter> overrides, std::chrono::microseconds align_delay,
                std::size_t threads = 0, bool busy_align = false) {
        auto options = rclcpp::NodeOptions();
        options.parameter_overrides(overrides);
        align_log_ = std::make_shared<AlignCallLog>();
        if (busy_align) {
            node_ = std::make_unique<LocalizationNode>(
                options, std::make_unique<BusyAligner>(align_delay, align_log_));
        } else {
            node_ = std::make_unique<LocalizationNode>(
                options, std::make_unique<FixedDelayAligner>(align_delay, align_log_));
        }
        spy_ = std::make_shared<rclcpp::Node>("diesel_localization_test_spy");
        state_sub_ = spy_->create_subscription<diesel_localization_ros::msg::LocalizationState>(
            "/localization/state", rclcpp::QoS(50).reliable().transient_local(),
            [this](const diesel_localization_ros::msg::LocalizationState::ConstSharedPtr message) {
                std::lock_guard<std::mutex> lock(record_mutex_);
                StateRecord record;
                record.state = message->state;
                record.source = message->source;
                record.pose_valid = message->pose_valid;
                record.reason = message->reason;
                record.x = message->pose.pose.position.x;
                record.covariance = message->pose.covariance;
                record.stamp_ns =
                    static_cast<std::int64_t>(message->header.stamp.sec) * 1000000000LL +
                    static_cast<std::int64_t>(message->header.stamp.nanosec);
                states_.push_back(record);
            });
        obstacle_sub_ =
            spy_->create_subscription<diesel_localization_ros::msg::DynamicObstacleArray>(
                "/localization/dynamic_obstacles", rclcpp::QoS(10).reliable(),
                [this](const diesel_localization_ros::msg::DynamicObstacleArray::ConstSharedPtr
                           message) {
                    std::lock_guard<std::mutex> lock(record_mutex_);
                    last_obstacle_stamp_ns_ =
                        static_cast<std::int64_t>(message->header.stamp.sec) * 1000000000LL +
                        static_cast<std::int64_t>(message->header.stamp.nanosec);
                    obstacle_count_ = obstacle_count_ + 1;
                });
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(spy_->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, spy_);
        if (threads == 1) {
            executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        } else {
            executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
                rclcpp::ExecutorOptions(), threads);
        }
        executor_->add_node(node_->get_node_base_interface());
        executor_->add_node(spy_);
        spinner_ = std::thread([this]() { executor_->spin(); });
    }

    ~NodeHarness() {
        executor_->cancel();
        if (spinner_.joinable()) {
            spinner_.join();
        }
    }

    NodeHarness(const NodeHarness&) = delete;
    NodeHarness& operator=(const NodeHarness&) = delete;

    LocalizationNode& node() { return *node_; }

    rclcpp::Node& spy() { return *spy_; }

    /// 以 sensor_data 语义发布一帧点云（节点侧的订阅就是 SensorDataQoS）。
    /// `stamp_offset_ns` 只改 header 时间戳（模拟传感器时钟与节点时钟的偏差），
    /// 收到时刻仍然是节点时钟——审计附带 (a) 要区分的正是这两个时钟域。
    void publish_cloud(std::size_t count, const std::string& frame = "base_footprint",
                       std::int64_t stamp_offset_ns = 0) {
        auto stamp = spy_->now();
        if (stamp_offset_ns != 0) {
            stamp = rclcpp::Time(stamp.nanoseconds() + stamp_offset_ns);
        }
        last_cloud_stamp_ns_ = stamp.nanoseconds();
        const auto message = make_cloud_msg(frame, stamp, count);
        std::lock_guard<std::mutex> lock(publish_mutex_);
        if (!cloud_pub_) {
            cloud_pub_ = spy_->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/livox/lidar", rclcpp::SensorDataQoS());
        }
        cloud_pub_->publish(message);
    }

    /// 发布一帧里程计。`reliable=false` 用来验证节点订阅的 QoS 是否真的兼容 best_effort。
    void publish_odom(double x, double y, const std::string& child_frame = "base_footprint",
                      bool reliable = true, std::int64_t stamp_offset_ns = 0) {
        auto stamp = spy_->now();
        if (stamp_offset_ns != 0) {
            stamp = rclcpp::Time(stamp.nanoseconds() + stamp_offset_ns);
        }
        const auto message = make_odom_msg(child_frame, stamp, x, y);
        std::lock_guard<std::mutex> lock(publish_mutex_);
        if (!odom_pub_) {
            odom_pub_ = spy_->create_publisher<nav_msgs::msg::Odometry>(
                "/aft_mapped_to_init", reliable ? rclcpp::QoS(50) : rclcpp::SensorDataQoS());
        }
        odom_pub_->publish(message);
    }

    void publish_external_pose(double x, double y, const std::string& frame = "map") {
        const auto stamp = spy_->now();
        const auto message = make_external_pose_msg(frame, stamp, x, y);
        std::lock_guard<std::mutex> lock(publish_mutex_);
        if (!external_pub_) {
            external_pub_ = spy_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "/localization/external_pose", rclcpp::QoS(20));
        }
        external_pub_->publish(message);
    }

    /// 在给定时长内按 `period` 周期执行一次 `step`，用于「边发边等」的回放循环。
    void drive(std::chrono::milliseconds duration, std::chrono::milliseconds period,
               const std::function<void(std::int64_t elapsed_ms)>& step) {
        const auto begin = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - begin < duration) {
            const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                     std::chrono::steady_clock::now() - begin)
                                     .count();
            step(elapsed);
            std::this_thread::sleep_for(period);
        }
    }

    bool wait_until(const std::function<bool()>& predicate, std::chrono::milliseconds timeout) {
        const auto begin = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - begin < timeout) {
            if (predicate()) {
                return true;
            }
            std::this_thread::sleep_for(5ms);
        }
        return predicate();
    }

    std::vector<StateRecord> states() {
        std::lock_guard<std::mutex> lock(record_mutex_);
        return states_;
    }

    std::size_t state_count() {
        std::lock_guard<std::mutex> lock(record_mutex_);
        return states_.size();
    }

    /// 最近一条状态（没有则返回默认值）。
    StateRecord last_state() {
        std::lock_guard<std::mutex> lock(record_mutex_);
        return states_.empty() ? StateRecord{} : states_.back();
    }

    /// 状态里是否出现过指定 reason 片段（用于区分「没收到」与「处理不及」）。
    bool has_reason(const std::string& fragment) {
        std::lock_guard<std::mutex> lock(record_mutex_);
        for (const auto& record : states_) {
            if (record.reason.find(fragment) != std::string::npos) {
                return true;
            }
        }
        return false;
    }

    /// 状态里是否出现过 LOST（用消息常量而不是魔数）。
    bool has_lost() {
        std::lock_guard<std::mutex> lock(record_mutex_);
        for (const auto& record : states_) {
            if (record.state == diesel_localization_ros::msg::LocalizationState::STATE_LOST) {
                return true;
            }
        }
        return false;
    }

    /// 从 /tf 实际收到的 `map -> odom`（不是发出去的计数）：能取到就说明真的上了话题。
    bool lookup_map_to_odom(geometry_msgs::msg::TransformStamped* out) {
        try {
            *out = tf_buffer_->lookupTransform("map", "odom", tf2::TimePointZero);
            return true;
        } catch (const tf2::TransformException&) {
            return false;
        }
    }

    /// 发布一条静态外参（`/tf_static`），用来把雷达/里程计子帧与 `base_footprint` 串起来。
    /// 静态变换只发一次即可，但 TF 监听器是异步收的，紧接着发数据可能还没进 buffer。
    void publish_static_transform(const std::string& parent, const std::string& child, double x,
                                  double y, double z = 0.0) {
        if (!static_broadcaster_) {
            static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(spy_);
        }
        geometry_msgs::msg::TransformStamped message;
        message.header.stamp = spy_->now();
        message.header.frame_id = parent;
        message.child_frame_id = child;
        message.transform.translation.x = x;
        message.transform.translation.y = y;
        message.transform.translation.z = z;
        message.transform.rotation.w = 1.0;
        static_broadcaster_->sendTransform(message);
    }

    LocalizationNode::Diagnostics diagnostics() { return node_->diagnostics(); }

    std::shared_ptr<AlignCallLog> align_log() const { return align_log_; }

    /// 最近一次 `publish_cloud()` 用的时间戳（节点时钟，ns）。audit L6 要断言
    /// `/tf` 上的 `map → odom` 用的是「被消费点云的时刻」而不是发布时刻。
    std::int64_t last_cloud_stamp_ns() const { return last_cloud_stamp_ns_; }

    /// 收到的 `/localization/dynamic_obstacles` 帧数（审计附带 (a)）。
    std::size_t obstacle_count() {
        std::lock_guard<std::mutex> lock(record_mutex_);
        return obstacle_count_;
    }

    /// 最近一帧障碍消息 header 上的时间戳（应当就是被处理那帧点云的时间戳）。
    std::int64_t last_obstacle_stamp_ns() {
        std::lock_guard<std::mutex> lock(record_mutex_);
        return last_obstacle_stamp_ns_;
    }

  private:
    std::unique_ptr<LocalizationNode> node_;
    rclcpp::Node::SharedPtr spy_;
    rclcpp::Subscription<diesel_localization_ros::msg::LocalizationState>::SharedPtr state_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr external_pub_;
    rclcpp::Subscription<diesel_localization_ros::msg::DynamicObstacleArray>::SharedPtr
        obstacle_sub_;
    std::size_t obstacle_count_{0};
    std::int64_t last_obstacle_stamp_ns_{0};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    std::int64_t last_cloud_stamp_ns_{0};
    std::shared_ptr<AlignCallLog> align_log_;
    std::shared_ptr<rclcpp::Executor> executor_;
    std::thread spinner_;
    std::mutex record_mutex_;
    std::mutex publish_mutex_;
    std::vector<StateRecord> states_;
};

} // namespace diesel::localization_ros::test_support

#endif // DIESEL_LOCALIZATION_ROS_TEST_LOCALIZATION_NODE_TEST_SUPPORT_HPP_
