cmake_minimum_required(VERSION 2.8.3)
project(exploration_manager)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
  plan_env
  path_searching
  bspline
  bspline_opt
  traj_utils
  active_perception
  lkh_tsp_solver
  plan_manage
  cv_bridge
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES exploration_manager
 CATKIN_DEPENDS plan_env path_searching bspline bspline_opt active_perception traj_utils lkh_tsp_solver plan_manage
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)


add_executable(exploration_node
  src/exploration_node.cpp 
  src/fast_exploration_fsm.cpp
  src/fast_exploration_manager.cpp
  )
target_link_libraries(exploration_node 
  ${catkin_LIBRARIES}
  # -ldw
  )

# add_library(exploration_manager
#   src/fast_exploration_manager.cpp
# )
# target_link_libraries(exploration_manager
#   ${catkin_LIBRARIES}
# )

add_executable(write_tsp test/write_tsp.cpp)
target_link_libraries(write_tsp
  ${catkin_LIBRARIES} 
)


add_executable(load_obj test/load_obj.cpp)
target_link_libraries(load_obj
  ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)