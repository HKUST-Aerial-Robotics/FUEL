#pragma once
#include <quadrotor_msgs/msg/corrections.hpp>
#include <memory>

namespace quadrotor_msgs {
using Corrections = msg::Corrections;
using CorrectionsPtr = Corrections::SharedPtr;
using CorrectionsConstPtr = Corrections::ConstSharedPtr;
}  // namespace quadrotor_msgs
