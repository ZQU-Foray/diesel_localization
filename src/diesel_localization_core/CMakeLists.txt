cmake_minimum_required(VERSION 3.16)
project(diesel_localization_core VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)
# 同一份 CMake 既支持 colcon/ament（ROS 2 工作区），也支持独立 cmake 构建：
# 缺少 ament 时不导出 ROS 包，只用 GTest/CTest 跑单测。
find_package(ament_cmake QUIET)

add_library(${PROJECT_NAME} SHARED
    src/geometry.cpp
    src/health.cpp
    src/manifest.cpp
    src/pose_ekf.cpp
    src/sha256.cpp
)
add_library(diesel_localization::core ALIAS ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME} PROPERTIES EXPORT_NAME core)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

if(TARGET yaml-cpp::yaml-cpp)
    set(DIESEL_YAML_CPP_TARGET yaml-cpp::yaml-cpp)
else()
    set(DIESEL_YAML_CPP_TARGET yaml-cpp)
endif()

target_link_libraries(${PROJECT_NAME} PUBLIC
    Eigen3::Eigen
    ${DIESEL_YAML_CPP_TARGET}
)

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

diesel_add_test(test_geometry test/test_geometry.cpp)
diesel_add_test(test_health test/test_health.cpp)
diesel_add_test(test_sha256 test/test_sha256.cpp)
diesel_add_test(test_manifest test/test_manifest.cpp)
diesel_add_test(test_pose_ekf test/test_pose_ekf.cpp)

if(ament_cmake_FOUND)
    ament_export_include_directories(include)
    ament_export_libraries(${PROJECT_NAME})
    ament_export_dependencies(Eigen3 yaml-cpp)
    ament_package()
endif()
