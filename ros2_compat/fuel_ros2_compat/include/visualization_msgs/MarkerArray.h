#pragma once
#include <visualization_msgs/msg/marker_array.hpp>
#include <memory>

namespace visualization_msgs {
using MarkerArray = msg::MarkerArray;
using MarkerArrayPtr = MarkerArray::SharedPtr;
using MarkerArrayConstPtr = MarkerArray::ConstSharedPtr;
}  // namespace visualization_msgs
