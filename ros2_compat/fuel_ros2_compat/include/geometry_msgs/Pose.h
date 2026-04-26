#pragma once
#include <geometry_msgs/msg/pose.hpp>
#include <memory>

namespace geometry_msgs {
using Pose = msg::Pose;
using PosePtr = Pose::SharedPtr;
using PoseConstPtr = Pose::ConstSharedPtr;
}  // namespace geometry_msgs
