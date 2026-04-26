#pragma once
#include <quadrotor_msgs/msg/status_data.hpp>
#include <memory>

namespace quadrotor_msgs {
using StatusData = msg::StatusData;
using StatusDataPtr = StatusData::SharedPtr;
using StatusDataConstPtr = StatusData::ConstSharedPtr;
}  // namespace quadrotor_msgs
