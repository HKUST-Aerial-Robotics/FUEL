#pragma once
#include <quadrotor_msgs/msg/so3_command.hpp>
#include <memory>

namespace quadrotor_msgs {
using SO3Command = msg::SO3Command;
using SO3CommandPtr = SO3Command::SharedPtr;
using SO3CommandConstPtr = SO3Command::ConstSharedPtr;
}  // namespace quadrotor_msgs
