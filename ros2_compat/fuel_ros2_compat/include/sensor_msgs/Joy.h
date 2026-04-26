#pragma once
#include <sensor_msgs/msg/joy.hpp>
#include <memory>

namespace sensor_msgs {
using Joy = msg::Joy;
using JoyPtr = Joy::SharedPtr;
using JoyConstPtr = Joy::ConstSharedPtr;
}  // namespace sensor_msgs
