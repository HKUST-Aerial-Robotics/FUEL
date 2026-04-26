#pragma once
#include <visualization_msgs/msg/marker.hpp>
#include <memory>

namespace visualization_msgs {
using Marker = msg::Marker;
using MarkerPtr = Marker::SharedPtr;
using MarkerConstPtr = Marker::ConstSharedPtr;
}  // namespace visualization_msgs
