cmake_minimum_required(VERSION 3.8)
project(so3_quadrotor_simulator)

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
find_package(quadrotor_msgs REQUIRED)
find_package(uav_utils REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Armadillo REQUIRED)

add_library(quadrotor_dynamics src/dynamics/Quadrotor.cpp)
target_include_directories(quadrotor_dynamics PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ode>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(quadrotor_dynamics
  fuel_ros2_compat
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  quadrotor_msgs
  uav_utils)
target_link_libraries(quadrotor_dynamics ${ARMADILLO_LIBRARIES})
add_executable(quadrotor_simulator_so3 src/quadrotor_simulator_so3.cpp)
target_include_directories(quadrotor_simulator_so3 PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ode>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(quadrotor_simulator_so3
  fuel_ros2_compat
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  quadrotor_msgs
  uav_utils)
target_link_libraries(quadrotor_simulator_so3 quadrotor_dynamics ${ARMADILLO_LIBRARIES})
install(TARGETS quadrotor_dynamics
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS quadrotor_simulator_so3
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
  geometry_msgs
  nav_msgs
  sensor_msgs
  quadrotor_msgs
  uav_utils)
ament_package()
