#pragma once
#include <nav_msgs/msg/path.hpp>
#include <memory>

namespace nav_msgs {
using Path = msg::Path;
using PathPtr = Path::SharedPtr;
using PathConstPtr = Path::ConstSharedPtr;
}  // namespace nav_msgs
