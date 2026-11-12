cmake_minimum_required(VERSION 3.16)
project(diesel_localization_ros VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(diesel_localization_core REQUIRED)
find_package(diesel_localization_map REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/LocalizationState.msg"
    "msg/MapMeta.msg"
    "msg/DynamicObstacle.msg"
    "msg/DynamicObstacleArray.msg"
    "srv/Relocalize.srv"
    DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

# 节点实现进库：节点级测试要能直接构造它（决策见 decision.md D29）。
# 可执行文件只留 entry point，安装产物与之前一致。
add_library(${PROJECT_NAME}_lib SHARED
    src/conversion.cpp
    src/adapters.cpp
    src/localization_node.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
foreach(_dep diesel_localization_core diesel_localization_map)
    if(DEFINED ${_dep}_DIR)
        get_filename_component(_prefix "${${_dep}_DIR}/../../.." ABSOLUTE)
        target_include_directories(${PROJECT_NAME}_lib PUBLIC "${_prefix}/include")
    endif()
endforeach()
ament_target_dependencies(${PROJECT_NAME}_lib
    rclcpp
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    builtin_interfaces
    Eigen3
    tf2_ros
    diesel_localization_core
    diesel_localization_map
)
target_link_libraries(${PROJECT_NAME}_lib Eigen3::Eigen)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_lib ${cpp_typesupport_target})

add_executable(localization_node
    src/localization_node_main.cpp
)
target_link_libraries(localization_node ${PROJECT_NAME}_lib)

add_executable(map_server_node
    src/map_server_node_main.cpp
    src/map_server_node.cpp
)
target_link_libraries(map_server_node ${PROJECT_NAME}_lib)

install(
    DIRECTORY include/
    DESTINATION include
)
install(
    TARGETS ${PROJECT_NAME}_lib localization_node map_server_node
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    function(diesel_add_test name source)
        ament_add_gtest(${name} ${source})
        target_link_libraries(${name} ${PROJECT_NAME}_lib)
    endfunction()

    diesel_add_test(test_adapters test/test_adapters.cpp)
    diesel_add_test(test_conversion test/test_conversion.cpp)

    # 节点级测试自带 main（要先 rclcpp::init 再跑 gtest），不能用 diesel_add_test。
    ament_add_gtest(
        test_localization_node test/test_localization_node.cpp
        SKIP_LINKING_MAIN_LIBRARIES
        TIMEOUT 180
    )
    target_link_libraries(test_localization_node ${PROJECT_NAME}_lib)

    # 地图失败用例单独成文件：它们不依赖慢配准，跑得比 L1 的 12 s 用例快得多。
    ament_add_gtest(
        test_localization_node_map_errors test/test_localization_node_map_errors.cpp
        SKIP_LINKING_MAIN_LIBRARIES
        TIMEOUT 120
    )
    target_link_libraries(test_localization_node_map_errors ${PROJECT_NAME}_lib)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs builtin_interfaces tf2_ros
    diesel_localization_core diesel_localization_map
)
ament_package()
