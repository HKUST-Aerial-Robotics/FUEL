#pragma once
#include <geometry_msgs/msg/vector3.hpp>
#include <memory>

namespace geometry_msgs {
using Vector3 = msg::Vector3;
using Vector3Ptr = Vector3::SharedPtr;
using Vector3ConstPtr = Vector3::ConstSharedPtr;
}  // namespace geometry_msgs
