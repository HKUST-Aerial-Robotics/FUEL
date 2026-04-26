#pragma once
#include <sensor_msgs/msg/camera_info.hpp>
#include <memory>

namespace sensor_msgs {
using CameraInfo = msg::CameraInfo;
using CameraInfoPtr = CameraInfo::SharedPtr;
using CameraInfoConstPtr = CameraInfo::ConstSharedPtr;
}  // namespace sensor_msgs
