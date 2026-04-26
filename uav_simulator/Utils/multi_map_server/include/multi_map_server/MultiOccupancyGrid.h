#pragma once
#include <multi_map_server/msg/multi_occupancy_grid.hpp>
#include <memory>

namespace multi_map_server {
using MultiOccupancyGrid = msg::MultiOccupancyGrid;
using MultiOccupancyGridPtr = MultiOccupancyGrid::SharedPtr;
using MultiOccupancyGridConstPtr = MultiOccupancyGrid::ConstSharedPtr;
}  // namespace multi_map_server
