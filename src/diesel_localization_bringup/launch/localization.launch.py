# Copyright 2026 ZQU-Foray
#
# 定位层启动文件。默认只起地图服务与定位节点；point_lio 由本仓之外的驱动栈提供。
# 兵种差异放在 config/<config_name>，由 config_name 参数选择。

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_name = LaunchConfiguration("config_name")
    map_manifest = LaunchConfiguration("map_manifest")
    verify_map_hashes = LaunchConfiguration("verify_map_hashes")

    config_file = PathJoinSubstitution(
        [FindPackageShare("diesel_localization_bringup"), "config", config_name]
    )

    common_params = [
        config_file,
        {"map_manifest": map_manifest, "verify_map_hashes": verify_map_hashes},
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_name",
                default_value="sentry.yaml",
                description="兵种参数文件：sentry.yaml / infantry.yaml / heavy.yaml",
            ),
            DeclareLaunchArgument(
                "map_manifest",
                default_value="",
                description="先验地图 manifest.yaml 绝对路径",
            ),
            DeclareLaunchArgument(
                "start_map_server",
                default_value="true",
                description="是否同时启动地图服务节点",
            ),
            DeclareLaunchArgument(
                "verify_map_hashes",
                default_value="true",
                description="加载地图时校验 SHA-256",
            ),
            Node(
                package="diesel_localization_ros",
                executable="map_server_node",
                name="diesel_map_server",
                output="screen",
                parameters=common_params,
                condition=IfCondition(LaunchConfiguration("start_map_server")),
            ),
            Node(
                package="diesel_localization_ros",
                executable="localization_node",
                name="diesel_localization",
                output="screen",
                parameters=common_params,
            ),
        ]
    )
