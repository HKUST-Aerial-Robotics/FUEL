#pragma once
#include <std_msgs/msg/color_rgba.hpp>
#include <memory>

namespace std_msgs {
using ColorRGBA = msg::ColorRGBA;
using ColorRGBAPtr = ColorRGBA::SharedPtr;
using ColorRGBAConstPtr = ColorRGBA::ConstSharedPtr;
}  // namespace std_msgs
