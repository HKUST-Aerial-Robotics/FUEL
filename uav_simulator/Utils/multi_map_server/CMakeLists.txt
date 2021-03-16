cmake_minimum_required(VERSION 2.8.3)
project(multi_map_server)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pose_utils
  message_generation
  roscpp
  visualization_msgs
  tf
  std_srvs
  laser_geometry
  pose_utils
  quadrotor_msgs
)

add_message_files(
   FILES
   MultiOccupancyGrid.msg
   MultiSparseMap3D.msg
   SparseMap3D.msg
   VerticalOccupancyGridList.msg
   #plan_cmd.msg
)

generate_messages(
   DEPENDENCIES
   geometry_msgs
   nav_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES irobot_msgs
   CATKIN_DEPENDS geometry_msgs nav_msgs quadrotor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
find_package(Armadillo REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(multi_map_visualization src/multi_map_visualization.cc)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(multi_map_visualization multi_map_server_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(multi_map_visualization 
   ${catkin_LIBRARIES}
   ${ARMADILLO_LIBRARIES}
)
