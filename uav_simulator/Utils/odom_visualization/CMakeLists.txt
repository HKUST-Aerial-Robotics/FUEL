cmake_minimum_required(VERSION 3.8)
project(odom_visualization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type")
add_compile_options(-O3 -Wall)

find_package(ament_cmake REQUIRED)
find_package(fuel_ros2_compat REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(pose_utils REQUIRED)
find_package(Armadillo REQUIRED)

add_executable(odom_visualization src/odom_visualization.cpp)
target_include_directories(odom_visualization PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(odom_visualization
  fuel_ros2_compat
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  quadrotor_msgs
  pose_utils)
target_link_libraries(odom_visualization ${ARMADILLO_LIBRARIES})
install(TARGETS odom_visualization
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/meshes")
  install(DIRECTORY meshes/ DESTINATION share/${PROJECT_NAME}/meshes)
endif()
ament_export_dependencies(
  fuel_ros2_compat
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  quadrotor_msgs
  pose_utils)
ament_package()
