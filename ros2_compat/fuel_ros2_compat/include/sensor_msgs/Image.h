#pragma once
#include <sensor_msgs/msg/image.hpp>
#include <memory>

namespace sensor_msgs {
using Image = msg::Image;
using ImagePtr = Image::SharedPtr;
using ImageConstPtr = Image::ConstSharedPtr;
}  // namespace sensor_msgs
