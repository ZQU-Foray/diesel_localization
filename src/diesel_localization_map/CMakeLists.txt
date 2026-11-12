cmake_minimum_required(VERSION 3.16)
project(diesel_localization_map VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(diesel_localization_core REQUIRED)
find_package(ament_cmake QUIET)

# 配准后端编译期开关（decision.md D21）：
# ON  = PCL GICP（默认后端）；PCL 缺失时直接 configure 失败，不静默降级——
#       静默降级会产出「文档说 GICP、二进制跑 ICP」的假象。
# OFF = 内置点到点 ICP（精度较低，仅用于无 PCL 的最小构建与降级）。
option(DIESEL_WITH_PCL "启用 PCL GICP 配准后端" ON)

set(_diesel_map_alignment_sources src/alignment_metrics.cpp src/aligner_factory.cpp)
if(DIESEL_WITH_PCL)
    find_package(PCL 1.10 REQUIRED COMPONENTS common registration)
    list(APPEND _diesel_map_alignment_sources src/pcl_gicp_aligner.cpp)
endif()

add_library(${PROJECT_NAME} SHARED
    src/point_cloud.cpp
    src/pcd_loader.cpp
    src/grid_map.cpp
    src/prior_map.cpp
    src/icp_aligner.cpp
    src/residual_clusterer.cpp
    src/observability.cpp
    ${_diesel_map_alignment_sources}
)
add_library(diesel_localization::map ALIAS ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME} PROPERTIES EXPORT_NAME map)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# 跨包头文件依赖显式固定，避免 colcon 增量安装时 ament 变量未及时刷新。
if(DEFINED diesel_localization_core_DIR)
    get_filename_component(_diesel_core_prefix "${diesel_localization_core_DIR}/../../.." ABSOLUTE)
    target_include_directories(${PROJECT_NAME} PUBLIC "${_diesel_core_prefix}/include")
endif()

if(ament_cmake_FOUND)
    ament_target_dependencies(${PROJECT_NAME} diesel_localization_core)
    target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
else()
    target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen diesel_localization::core)
endif()

if(DIESEL_WITH_PCL)
    # PCL 头只在 pcl_gicp_aligner.cpp 里出现；SYSTEM 是为了不让 PCL 自身的
    # -Wall -Wextra -Wpedantic 噪声淹没本仓的警告。
    target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
    # 必须用 plain signature：ament_target_dependencies 已经用过 plain 形式，
    # CMake 不允许同一个 target 混用 plain 与 PRIVATE/PUBLIC/INTERFACE 关键字形式。
    target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
    target_compile_definitions(${PROJECT_NAME} PRIVATE DIESEL_WITH_PCL=1)
    message(STATUS "diesel_localization_map: 配准后端 = pcl_gicp (PCL ${PCL_VERSION})")
else()
    message(STATUS "diesel_localization_map: 配准后端 = icp（DIESEL_WITH_PCL=OFF）")
endif()

install(
    DIRECTORY include/
    DESTINATION include
)
install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
    EXPORT export_${PROJECT_NAME}
    NAMESPACE diesel_localization::
    DESTINATION share/${PROJECT_NAME}/cmake
)

if(NOT ament_cmake_FOUND)
include(CMakePackageConfigHelpers)
configure_package_config_file(
    cmake/${PROJECT_NAME}Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    INSTALL_DESTINATION share/${PROJECT_NAME}/cmake
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)
install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    DESTINATION share/${PROJECT_NAME}/cmake
)
endif()

enable_testing()

if(ament_cmake_FOUND)
    find_package(ament_cmake_gtest REQUIRED)

    function(diesel_add_test name source)
        ament_add_gtest(${name} ${source})
        target_link_libraries(${name} ${PROJECT_NAME})
    endfunction()
else()
    find_package(GTest REQUIRED)

    function(diesel_add_test name source)
        add_executable(${name} ${source})
        target_link_libraries(${name} ${PROJECT_NAME} GTest::gtest GTest::gtest_main)
        add_test(NAME ${name} COMMAND ${name})
    endfunction()
endif()

diesel_add_test(test_point_cloud test/test_point_cloud.cpp)
diesel_add_test(test_pcd_loader test/test_pcd_loader.cpp)
diesel_add_test(test_grid_map test/test_grid_map.cpp)
diesel_add_test(test_prior_map test/test_prior_map.cpp)
diesel_add_test(test_icp test/test_icp.cpp)
diesel_add_test(test_residual_clusterer test/test_residual_clusterer.cpp)
diesel_add_test(test_observability test/test_observability.cpp)
if(DIESEL_WITH_PCL)
    diesel_add_test(test_pcl_gicp test/test_pcl_gicp.cpp)
endif()

if(ament_cmake_FOUND)
    ament_export_include_directories(include)
    ament_export_libraries(${PROJECT_NAME})
    ament_export_dependencies(Eigen3 diesel_localization_core)
    ament_package()
endif()
