// Copyright 2026 ZQU-Foray

#include "foray/localization_ros/map_server_node.hpp"

#include "foray/localization_ros/conversion.hpp"

#include <algorithm>
#include <chrono>

namespace foray::localization_ros {

MapServerNode::MapServerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("foray_map_server", options) {
    const auto manifest = declare_parameter<std::string>("map_manifest", "");
    const bool verify = declare_parameter<bool>("verify_map_hashes", true);
    const auto rate = declare_parameter<double>("publish_rate_hz", 1.0);

    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    meta_pub_ =
        create_publisher<foray_localization_ros::msg::MapMeta>("/localization/map_meta", qos);

    if (manifest.empty()) {
        RCLCPP_ERROR(get_logger(), "map_manifest 未设置，地图服务不可用");
    } else {
        bool loaded = false;
        std::string load_error;
        std::size_t skipped_pcd_lines = 0U;
        try {
            const auto result = foray::localization_map::load_prior_map(manifest, &map_, verify);
            loaded = result.ok;
            load_error = result.error;
            skipped_pcd_lines = result.skipped_pcd_lines;
        } catch (const std::exception& exception) {
            load_error = exception.what();
        }
        if (!loaded) {
            RCLCPP_ERROR(get_logger(), "加载先验地图失败: %s", load_error.c_str());
        } else {
            if (skipped_pcd_lines > 0U) {
                RCLCPP_WARN(get_logger(),
                            "先验地图有 %zu 行被跳过（字段数不足或 x/y/z 非有限），请检查导出流程",
                            skipped_pcd_lines);
            }
            loaded_ = true;
            RCLCPP_INFO(get_logger(), "地图版本 %s，%zu 点，栅格 %dx%d",
                        map_.manifest.version.c_str(), map_.cloud.size(), map_.grid.width,
                        map_.grid.height);
            publish_meta();
        }
    }

    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / std::max(1.0, rate)),
                               [this]() { publish_meta(); });
}

void MapServerNode::publish_meta() {
    if (!loaded_) {
        return;
    }
    meta_pub_->publish(to_msg_map_meta(map_.manifest, now().nanoseconds()));
}

} // namespace foray::localization_ros
