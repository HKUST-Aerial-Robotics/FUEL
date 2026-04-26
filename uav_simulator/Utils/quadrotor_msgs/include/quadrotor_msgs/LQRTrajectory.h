#pragma once
#include <quadrotor_msgs/msg/lqr_trajectory.hpp>
#include <memory>

namespace quadrotor_msgs {
using LQRTrajectory = msg::LQRTrajectory;
using LQRTrajectoryPtr = LQRTrajectory::SharedPtr;
using LQRTrajectoryConstPtr = LQRTrajectory::ConstSharedPtr;
}  // namespace quadrotor_msgs
