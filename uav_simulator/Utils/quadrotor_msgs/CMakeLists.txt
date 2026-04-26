cmake_minimum_required(VERSION 3.8)
project(quadrotor_msgs)

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
find_package(Eigen3 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AuxCommand.msg"
  "msg/Corrections.msg"
  "msg/Gains.msg"
  "msg/LQRTrajectory.msg"
  "msg/Odometry.msg"
  "msg/OutputData.msg"
  "msg/PPROutputData.msg"
  "msg/PolynomialTrajectory.msg"
  "msg/PositionCommand.msg"
  "msg/SO3Command.msg"
  "msg/Serial.msg"
  "msg/StatusData.msg"
  "msg/TRPYCommand.msg"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
add_library(decode_msgs src/decode_msgs.cpp)
target_include_directories(decode_msgs PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>")
ament_target_dependencies(decode_msgs
  fuel_ros2_compat
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs)
target_link_libraries(decode_msgs "${cpp_typesupport_target}")
add_dependencies(decode_msgs ${PROJECT_NAME})
add_library(encode_msgs src/encode_msgs.cpp)
target_include_directories(encode_msgs PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>")
ament_target_dependencies(encode_msgs
  fuel_ros2_compat
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs)
target_link_libraries(encode_msgs "${cpp_typesupport_target}")
add_dependencies(encode_msgs ${PROJECT_NAME})
install(TARGETS decode_msgs encode_msgs
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_export_dependencies(rosidl_default_runtime fuel_ros2_compat rclcpp std_msgs geometry_msgs nav_msgs)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
