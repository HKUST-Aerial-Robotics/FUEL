#pragma once
#include <nav_msgs/msg/odometry.hpp>
#include <memory>

namespace nav_msgs {
using Odometry = msg::Odometry;
using OdometryPtr = Odometry::SharedPtr;
using OdometryConstPtr = Odometry::ConstSharedPtr;
}  // namespace nav_msgs
