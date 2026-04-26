#pragma once
#include <quadrotor_msgs/msg/gains.hpp>
#include <memory>

namespace quadrotor_msgs {
using Gains = msg::Gains;
using GainsPtr = Gains::SharedPtr;
using GainsConstPtr = Gains::ConstSharedPtr;
}  // namespace quadrotor_msgs
