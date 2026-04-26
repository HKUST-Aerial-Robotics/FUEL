#pragma once
#include <quadrotor_msgs/msg/position_command.hpp>
#include <memory>

namespace quadrotor_msgs {
using PositionCommand = msg::PositionCommand;
using PositionCommandPtr = PositionCommand::SharedPtr;
using PositionCommandConstPtr = PositionCommand::ConstSharedPtr;
}  // namespace quadrotor_msgs
