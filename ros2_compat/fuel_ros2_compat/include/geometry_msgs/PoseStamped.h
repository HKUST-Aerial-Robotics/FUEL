#pragma once
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>

namespace geometry_msgs {
using PoseStamped = msg::PoseStamped;
using PoseStampedPtr = PoseStamped::SharedPtr;
using PoseStampedConstPtr = PoseStamped::ConstSharedPtr;
}  // namespace geometry_msgs
