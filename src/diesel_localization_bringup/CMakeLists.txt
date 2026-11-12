cmake_minimum_required(VERSION 3.16)
project(diesel_localization_bringup VERSION 0.1.0)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
