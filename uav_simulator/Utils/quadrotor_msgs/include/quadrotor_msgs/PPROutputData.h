#pragma once
#include <quadrotor_msgs/msg/ppr_output_data.hpp>
#include <memory>

namespace quadrotor_msgs {
using PPROutputData = msg::PPROutputData;
using PPROutputDataPtr = PPROutputData::SharedPtr;
using PPROutputDataConstPtr = PPROutputData::ConstSharedPtr;
}  // namespace quadrotor_msgs
