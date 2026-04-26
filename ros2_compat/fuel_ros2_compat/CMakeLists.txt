cmake_minimum_required(VERSION 3.8)
project(fuel_ros2_compat)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(${PROJECT_NAME} INTERFACE
  ament_index_cpp
  builtin_interfaces
  geometry_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  ament_index_cpp
  builtin_interfaces
  geometry_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros)
ament_package()
