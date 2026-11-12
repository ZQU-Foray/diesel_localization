// Copyright 2026 ZQU-Foray
//
// 地图服务节点：校验并加载先验地图，以 transient_local 发布 MapMeta，
// 让下游在任何时刻加入都能拿到地图版本与原点。

#ifndef DIESEL_LOCALIZATION_ROS_MAP_SERVER_NODE_HPP_
#define DIESEL_LOCALIZATION_ROS_MAP_SERVER_NODE_HPP_

#include "diesel/localization_map/prior_map.hpp"
#include "diesel_localization_ros/msg/map_meta.hpp"

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace diesel::localization_ros {

class MapServerNode : public rclcpp::Node {
  public:
    explicit MapServerNode(const rclcpp::NodeOptions& options);

  private:
    void publish_meta();

    diesel::localization_map::PriorMap map_;
    bool loaded_{false};
    rclcpp::Publisher<diesel_localization_ros::msg::MapMeta>::SharedPtr meta_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace diesel::localization_ros

#endif // DIESEL_LOCALIZATION_ROS_MAP_SERVER_NODE_HPP_
