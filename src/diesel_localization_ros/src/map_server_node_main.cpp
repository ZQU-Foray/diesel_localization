// Copyright 2026 ZQU-Foray

#include "diesel/localization_ros/map_server_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<diesel::localization_ros::MapServerNode>(rclcpp::NodeOptions{});
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
