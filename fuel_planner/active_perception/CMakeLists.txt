cmake_minimum_required(VERSION 3.8)
project(active_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type")
add_compile_options(-O3 -Wall)

find_package(ament_cmake REQUIRED)
find_package(fuel_ros2_compat REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(plan_env REQUIRED)
find_package(bspline REQUIRED)
find_package(path_searching REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree search)

add_library(active_perception src/traj_visibility.cpp
  src/heading_planner.cpp
  src/frontier_finder.cpp
  src/graph_node.cpp
  src/perception_utils.cpp)
target_include_directories(active_perception PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(active_perception
  fuel_ros2_compat
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  cv_bridge
  pcl_conversions
  plan_env
  bspline
  path_searching)
target_link_libraries(active_perception ${PCL_LIBRARIES})
install(TARGETS active_perception
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  fuel_ros2_compat
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  cv_bridge
  pcl_conversions
  plan_env
  bspline
  path_searching)
ament_package()
