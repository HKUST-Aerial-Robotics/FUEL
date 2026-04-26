cmake_minimum_required(VERSION 3.8)
project(uav_utils)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type")
add_compile_options(-O3 -Wall)

find_package(ament_cmake REQUIRED)
find_package(fuel_ros2_compat REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

install(DIRECTORY include/ DESTINATION include)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts")
  install(DIRECTORY scripts/ DESTINATION share/${PROJECT_NAME}/scripts)
endif()
ament_export_include_directories(include)
ament_export_dependencies(
  fuel_ros2_compat
  rclcpp
  nav_msgs
  geometry_msgs)
ament_package()
