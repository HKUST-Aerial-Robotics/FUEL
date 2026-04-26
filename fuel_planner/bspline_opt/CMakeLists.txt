cmake_minimum_required(VERSION 3.8)
project(bspline_opt)

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
find_package(active_perception REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree search)
find_path(NLOPT_INCLUDE_DIR nlopt.hpp PATHS /usr/local/include /usr/include)
find_library(NLOPT_LIBRARY nlopt PATHS /usr/local/lib /usr/lib /usr/lib/x86_64-linux-gnu /lib/x86_64-linux-gnu)
if(NOT NLOPT_INCLUDE_DIR OR NOT NLOPT_LIBRARY)
  message(FATAL_ERROR "NLOpt headers/library not found. Install libnlopt-dev or provide NLOPT_INCLUDE_DIR and NLOPT_LIBRARY.")
endif()

add_library(bspline_opt src/bspline_optimizer.cpp)
target_include_directories(bspline_opt PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  "${NLOPT_INCLUDE_DIR}")
ament_target_dependencies(bspline_opt
  fuel_ros2_compat
  rclcpp
  std_msgs
  visualization_msgs
  sensor_msgs
  cv_bridge
  plan_env
  active_perception)
target_link_libraries(bspline_opt ${NLOPT_LIBRARY} ${PCL_LIBRARIES})
install(TARGETS bspline_opt
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
  plan_env
  active_perception)
ament_package()
