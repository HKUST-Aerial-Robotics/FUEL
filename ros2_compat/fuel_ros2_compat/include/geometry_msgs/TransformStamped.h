#pragma once
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>

namespace geometry_msgs {
using TransformStamped = msg::TransformStamped;
using TransformStampedPtr = TransformStamped::SharedPtr;
using TransformStampedConstPtr = TransformStamped::ConstSharedPtr;
}  // namespace geometry_msgs
