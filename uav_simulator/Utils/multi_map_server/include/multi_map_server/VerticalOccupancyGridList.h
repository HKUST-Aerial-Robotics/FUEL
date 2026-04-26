#pragma once
#include <multi_map_server/msg/vertical_occupancy_grid_list.hpp>
#include <memory>

namespace multi_map_server {
using VerticalOccupancyGridList = msg::VerticalOccupancyGridList;
using VerticalOccupancyGridListPtr = VerticalOccupancyGridList::SharedPtr;
using VerticalOccupancyGridListConstPtr = VerticalOccupancyGridList::ConstSharedPtr;
}  // namespace multi_map_server
