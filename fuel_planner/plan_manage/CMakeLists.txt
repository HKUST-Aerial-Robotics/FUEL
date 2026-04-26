cmake_minimum_required(VERSION 3.8)
project(plan_manage)

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
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(poly_traj REQUIRED)
find_package(plan_env REQUIRED)
find_package(path_searching REQUIRED)
find_package(bspline REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(traj_utils REQUIRED)
find_package(active_perception REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree search)

add_library(plan_manage src/planner_manager.cpp
  src/planner_manager_dev.cpp)
target_include_directories(plan_manage PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(plan_manage
  fuel_ros2_compat
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  quadrotor_msgs
  poly_traj
  plan_env
  path_searching
  bspline
  bspline_opt
  traj_utils
  active_perception
  cv_bridge
  pcl_conversions)
target_link_libraries(plan_manage ${PCL_LIBRARIES})
add_executable(fast_planner_node src/fast_planner_node.cpp
  src/kino_replan_fsm.cpp
  src/topo_replan_fsm.cpp
  test/local_explore_fsm.cpp
  src/planner_manager.cpp
  src/planner_manager_dev.cpp)
target_include_directories(fast_planner_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(fast_planner_node
  fuel_ros2_compat
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  quadrotor_msgs
  poly_traj
  plan_env
  path_searching
  bspline
  bspline_opt
  traj_utils
  active_perception
  cv_bridge
  pcl_conversions)
target_link_libraries(fast_planner_node ${PCL_LIBRARIES})
add_executable(traj_server src/traj_server.cpp)
target_include_directories(traj_server PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(traj_server
  fuel_ros2_compat
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  quadrotor_msgs
  poly_traj
  plan_env
  path_searching
  bspline
  bspline_opt
  traj_utils
  active_perception
  cv_bridge
  pcl_conversions)
target_link_libraries(traj_server ${PCL_LIBRARIES})

install(TARGETS plan_manage
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS fast_planner_node traj_server
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
endif()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
endif()
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  fuel_ros2_compat
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  quadrotor_msgs
  poly_traj
  plan_env
  path_searching
  bspline
  bspline_opt
  traj_utils
  active_perception
  cv_bridge
  pcl_conversions)
ament_package()
