#pragma once
#include <nav_msgs/msg/map_meta_data.hpp>
#include <memory>

namespace nav_msgs {
using MapMetaData = msg::MapMetaData;
using MapMetaDataPtr = MapMetaData::SharedPtr;
using MapMetaDataConstPtr = MapMetaData::ConstSharedPtr;
}  // namespace nav_msgs
