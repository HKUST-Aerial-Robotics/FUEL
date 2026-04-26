#pragma once
#include <sensor_msgs/msg/imu.hpp>
#include <memory>

namespace sensor_msgs {
using Imu = msg::Imu;
using ImuPtr = Imu::SharedPtr;
using ImuConstPtr = Imu::ConstSharedPtr;
}  // namespace sensor_msgs
