#pragma once
#include <quadrotor_msgs/msg/odometry.hpp>
#include <memory>

namespace quadrotor_msgs {
using Odometry = msg::Odometry;
using OdometryPtr = Odometry::SharedPtr;
using OdometryConstPtr = Odometry::ConstSharedPtr;
}  // namespace quadrotor_msgs
