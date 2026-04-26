cmake_minimum_required(VERSION 3.8)
project(lkh_tsp_solver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type")
add_compile_options(-O3 -Wall)

find_package(ament_cmake REQUIRED)
find_package(fuel_ros2_compat REQUIRED)
find_package(rclcpp REQUIRED)

file(GLOB LKH_C_SOURCES CONFIGURE_DEPENDS src/*.c)

add_library(lkh_tsp_solver ${LKH_C_SOURCES} src/lkh_interface.cpp)
target_compile_options(lkh_tsp_solver PRIVATE $<$<COMPILE_LANGUAGE:C>:-fcommon>)
target_include_directories(lkh_tsp_solver PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(lkh_tsp_solver
  fuel_ros2_compat
  rclcpp)
target_link_libraries(lkh_tsp_solver m)

add_executable(lkh_tsp ${LKH_C_SOURCES})
target_compile_options(lkh_tsp PRIVATE $<$<COMPILE_LANGUAGE:C>:-fcommon>)
target_include_directories(lkh_tsp PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(lkh_tsp
  fuel_ros2_compat
  rclcpp)
target_link_libraries(lkh_tsp m)

install(TARGETS lkh_tsp_solver
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS lkh_tsp
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/resource")
  install(DIRECTORY resource/ DESTINATION share/${PROJECT_NAME}/resource)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(fuel_ros2_compat rclcpp)
ament_package()
