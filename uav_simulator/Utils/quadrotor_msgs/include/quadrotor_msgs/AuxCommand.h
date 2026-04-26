#pragma once
#include <quadrotor_msgs/msg/aux_command.hpp>
#include <memory>

namespace quadrotor_msgs {
using AuxCommand = msg::AuxCommand;
using AuxCommandPtr = AuxCommand::SharedPtr;
using AuxCommandConstPtr = AuxCommand::ConstSharedPtr;
}  // namespace quadrotor_msgs
