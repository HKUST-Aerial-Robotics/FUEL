#pragma once
#include <quadrotor_msgs/msg/serial.hpp>
#include <memory>

namespace quadrotor_msgs {
using Serial = msg::Serial;
using SerialPtr = Serial::SharedPtr;
using SerialConstPtr = Serial::ConstSharedPtr;
}  // namespace quadrotor_msgs
