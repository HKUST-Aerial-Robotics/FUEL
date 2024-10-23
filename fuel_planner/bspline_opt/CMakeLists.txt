cmake_minimum_required(VERSION 2.8.3)
project(bspline_opt)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
# 检查系统是否能够找到 nlopt
find_package(NLopt)
if(NOT NLopt_FOUND)
    set(NLOPT_INCLUDE_DIR "/usr/include")  # 默认系统路径
    set(NLOPT_LIBRARY "/usr/lib/x86_64-linux-gnu/libnlopt.so")  # 手动指定库路径
else()
    set(NLOPT_INCLUDE_DIRS ${NLOPT_INCLUDE_DIR})
    set(NLOPT_LIBRARIES ${NLOPT_LIBRARIES})
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  plan_env
  active_perception
  cv_bridge
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES bspline_opt
 CATKIN_DEPENDS plan_env active_perception
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${NLOPT_INCLUDE_DIRS}  # 如果 NLopt 被找到，使用 find_package 的结果
)

add_library( bspline_opt 
    src/bspline_optimizer.cpp 
    )
target_link_libraries( bspline_opt
    ${catkin_LIBRARIES} 
    ${NLOPT_LIBRARIES}  # 链接 nlopt 库
    ${NLOPT_LIBRARY}    # 如果没找到 nlopt，也手动链接
    )