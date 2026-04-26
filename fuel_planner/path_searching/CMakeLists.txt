cmake_minimum_required(VERSION 3.8)
project(path_searching)

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
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(plan_env REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree search)

add_library(path_searching src/kinodynamic_astar.cpp
  src/astar2.cpp
  src/topo_prm.cpp)
target_include_directories(path_searching PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(path_searching
  fuel_ros2_compat
  rclcpp
  std_msgs
  visualization_msgs
  sensor_msgs
  cv_bridge
  plan_env)
target_link_libraries(path_searching ${PCL_LIBRARIES})
install(TARGETS path_searching
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
  visualization_msgs
  sensor_msgs
  cv_bridge
  plan_env)
ament_package()
