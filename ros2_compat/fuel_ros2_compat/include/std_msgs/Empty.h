#pragma once
#include <std_msgs/msg/empty.hpp>
#include <memory>

namespace std_msgs {
using Empty = msg::Empty;
using EmptyPtr = Empty::SharedPtr;
using EmptyConstPtr = Empty::ConstSharedPtr;
}  // namespace std_msgs
