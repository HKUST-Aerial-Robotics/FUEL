#pragma once
#include <geometry_msgs/msg/pose_array.hpp>
#include <memory>

namespace geometry_msgs {
using PoseArray = msg::PoseArray;
using PoseArrayPtr = PoseArray::SharedPtr;
using PoseArrayConstPtr = PoseArray::ConstSharedPtr;
}  // namespace geometry_msgs
