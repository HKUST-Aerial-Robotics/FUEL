#pragma once
#include <quadrotor_msgs/msg/output_data.hpp>
#include <memory>

namespace quadrotor_msgs {
using OutputData = msg::OutputData;
using OutputDataPtr = OutputData::SharedPtr;
using OutputDataConstPtr = OutputData::ConstSharedPtr;
}  // namespace quadrotor_msgs
