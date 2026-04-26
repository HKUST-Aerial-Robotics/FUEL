cmake_minimum_required(VERSION 3.8)
project(so3_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wall)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fuel_ros2_compat REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(SO3Control src/SO3Control.cpp)
set_target_properties(SO3Control PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(SO3Control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(SO3Control fuel_ros2_compat)

add_library(so3_control_component SHARED src/so3_control_component.cpp)
target_include_directories(so3_control_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(so3_control_component SO3Control)
ament_target_dependencies(so3_control_component
  fuel_ros2_compat
  nav_msgs
  quadrotor_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs)
rclcpp_components_register_nodes(so3_control_component "so3_control::SO3ControlComponent")

add_executable(so3_control_node src/so3_control_node.cpp)
target_link_libraries(so3_control_node so3_control_component SO3Control)
ament_target_dependencies(so3_control_node rclcpp)

install(TARGETS SO3Control so3_control_component
  EXPORT export_so3_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS so3_control_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

ament_export_include_directories(include)
ament_export_libraries(SO3Control so3_control_component)
ament_export_targets(export_so3_control HAS_LIBRARY_TARGET)
ament_export_dependencies(
  fuel_ros2_compat
  nav_msgs
  quadrotor_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs)

ament_package()
