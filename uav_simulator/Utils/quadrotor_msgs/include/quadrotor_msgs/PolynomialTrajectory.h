#pragma once
#include <quadrotor_msgs/msg/polynomial_trajectory.hpp>
#include <memory>

namespace quadrotor_msgs {
using PolynomialTrajectory = msg::PolynomialTrajectory;
using PolynomialTrajectoryPtr = PolynomialTrajectory::SharedPtr;
using PolynomialTrajectoryConstPtr = PolynomialTrajectory::ConstSharedPtr;
}  // namespace quadrotor_msgs
