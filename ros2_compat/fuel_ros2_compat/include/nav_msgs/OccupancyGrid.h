#pragma once
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <memory>

namespace nav_msgs {
using OccupancyGrid = msg::OccupancyGrid;
using OccupancyGridPtr = OccupancyGrid::SharedPtr;
using OccupancyGridConstPtr = OccupancyGrid::ConstSharedPtr;
}  // namespace nav_msgs
