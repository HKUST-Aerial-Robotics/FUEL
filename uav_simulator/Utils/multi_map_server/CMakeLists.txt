cmake_minimum_required(VERSION 3.8)
project(multi_map_server)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type")
add_compile_options(-O3 -Wall)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(fuel_ros2_compat REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pose_utils REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(Armadillo REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MultiOccupancyGrid.msg"
  "msg/MultiSparseMap3D.msg"
  "msg/SparseMap3D.msg"
  "msg/VerticalOccupancyGridList.msg"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
add_executable(multi_map_visualization src/multi_map_visualization.cc)
target_include_directories(multi_map_visualization PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>")
ament_target_dependencies(multi_map_visualization
  fuel_ros2_compat
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  pose_utils
  visualization_msgs
  sensor_msgs
  std_srvs
  laser_geometry
  quadrotor_msgs)
target_link_libraries(multi_map_visualization "${cpp_typesupport_target}" ${ARMADILLO_LIBRARIES})
add_dependencies(multi_map_visualization ${PROJECT_NAME})
install(TARGETS multi_map_visualization
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_export_dependencies(rosidl_default_runtime fuel_ros2_compat rclcpp std_msgs geometry_msgs nav_msgs pose_utils visualization_msgs sensor_msgs std_srvs laser_geometry quadrotor_msgs)
ament_package()
