cmake_minimum_required(VERSION 3.8)
project(map_generator)

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
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree search)
find_package(Eigen3 REQUIRED)

add_executable(random_forest src/random_forest_sensing.cpp)
target_include_directories(random_forest PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(random_forest
  fuel_ros2_compat
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  pcl_conversions)
target_link_libraries(random_forest ${PCL_LIBRARIES})
add_executable(map_recorder src/map_recorder.cpp)
target_include_directories(map_recorder PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(map_recorder
  fuel_ros2_compat
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  pcl_conversions)
target_link_libraries(map_recorder ${PCL_LIBRARIES})
add_executable(map_pub src/map_publisher.cpp)
target_include_directories(map_pub PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(map_pub
  fuel_ros2_compat
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  pcl_conversions)
target_link_libraries(map_pub ${PCL_LIBRARIES})
add_executable(click_map src/click_map.cpp)
target_include_directories(click_map PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(click_map
  fuel_ros2_compat
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  pcl_conversions)
target_link_libraries(click_map ${PCL_LIBRARIES})
install(TARGETS random_forest map_recorder map_pub click_map
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/resource")
  install(DIRECTORY resource/ DESTINATION share/${PROJECT_NAME}/resource)
endif()
ament_export_dependencies(
  fuel_ros2_compat
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  pcl_conversions)
ament_package()
