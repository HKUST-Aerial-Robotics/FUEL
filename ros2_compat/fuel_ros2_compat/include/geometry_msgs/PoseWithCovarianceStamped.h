#pragma once
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <memory>

namespace geometry_msgs {
using PoseWithCovarianceStamped = msg::PoseWithCovarianceStamped;
using PoseWithCovarianceStampedPtr = PoseWithCovarianceStamped::SharedPtr;
using PoseWithCovarianceStampedConstPtr = PoseWithCovarianceStamped::ConstSharedPtr;
}  // namespace geometry_msgs
