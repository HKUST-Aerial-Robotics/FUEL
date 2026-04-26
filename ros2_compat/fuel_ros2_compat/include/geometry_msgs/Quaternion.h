#pragma once
#include <geometry_msgs/msg/quaternion.hpp>
#include <memory>

namespace geometry_msgs {
using Quaternion = msg::Quaternion;
using QuaternionPtr = Quaternion::SharedPtr;
using QuaternionConstPtr = Quaternion::ConstSharedPtr;
}  // namespace geometry_msgs
