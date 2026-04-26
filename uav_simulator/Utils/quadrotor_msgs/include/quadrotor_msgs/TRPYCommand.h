#pragma once
#include <quadrotor_msgs/msg/trpy_command.hpp>
#include <memory>

namespace quadrotor_msgs {
using TRPYCommand = msg::TRPYCommand;
using TRPYCommandPtr = TRPYCommand::SharedPtr;
using TRPYCommandConstPtr = TRPYCommand::ConstSharedPtr;
}  // namespace quadrotor_msgs
