cmake_minimum_required(VERSION 3.16)
project(diesel_localization_relocalization VERSION 0.1.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(DIESEL_WITH_ONNX "链接纯 CPU ONNX Runtime（地点识别）" OFF)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(diesel_localization_core REQUIRED)
find_package(diesel_localization_map REQUIRED)

add_library(${PROJECT_NAME} SHARED src/place_localizer.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
foreach(_dep diesel_localization_core diesel_localization_map)
    if(DEFINED ${_dep}_DIR)
        get_filename_component(_prefix "${${_dep}_DIR}/../../.." ABSOLUTE)
        target_include_directories(${PROJECT_NAME} PUBLIC "${_prefix}/include")
    endif()
endforeach()
ament_target_dependencies(${PROJECT_NAME} diesel_localization_core diesel_localization_map)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

if(DIESEL_WITH_ONNX)
    find_package(onnxruntime REQUIRED)
    target_link_libraries(${PROJECT_NAME} onnxruntime::onnxruntime)
    target_compile_definitions(${PROJECT_NAME} PUBLIC DIESEL_WITH_ONNX=1)
    message(STATUS "diesel_localization_relocalization: ONNX Runtime 已启用（纯 CPU）")
else()
    message(STATUS "diesel_localization_relocalization: ONNX 后端关闭（默认）")
endif()

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_place_localizer test/test_place_localizer.cpp)
    target_link_libraries(test_place_localizer ${PROJECT_NAME})
endif()

install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 diesel_localization_core diesel_localization_map)
ament_package()
