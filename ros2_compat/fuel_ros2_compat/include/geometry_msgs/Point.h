#pragma once
#include <geometry_msgs/msg/point.hpp>
#include <memory>

namespace geometry_msgs {
using Point = msg::Point;
using PointPtr = Point::SharedPtr;
using PointConstPtr = Point::ConstSharedPtr;
}  // namespace geometry_msgs
