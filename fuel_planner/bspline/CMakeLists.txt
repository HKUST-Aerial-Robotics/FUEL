cmake_minimum_required(VERSION 3.8)
project(bspline)

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
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(plan_env REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree search)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Bspline.msg"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
add_library(bspline_lib src/non_uniform_bspline.cpp)
target_include_directories(bspline_lib PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>")
ament_target_dependencies(bspline_lib
  fuel_ros2_compat
  rclcpp
  builtin_interfaces
  std_msgs
  geometry_msgs
  plan_env
  visualization_msgs
  sensor_msgs
  cv_bridge
  pcl_conversions)
target_link_libraries(bspline_lib "${cpp_typesupport_target}" ${PCL_LIBRARIES})
add_dependencies(bspline_lib ${PROJECT_NAME})
install(TARGETS bspline_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_export_dependencies(rosidl_default_runtime fuel_ros2_compat rclcpp builtin_interfaces std_msgs geometry_msgs plan_env visualization_msgs sensor_msgs cv_bridge pcl_conversions)
ament_export_libraries(bspline_lib)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
