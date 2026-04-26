#pragma once
#include <std_msgs/msg/int32_multi_array.hpp>
#include <memory>

namespace std_msgs {
using Int32MultiArray = msg::Int32MultiArray;
using Int32MultiArrayPtr = Int32MultiArray::SharedPtr;
using Int32MultiArrayConstPtr = Int32MultiArray::ConstSharedPtr;
}  // namespace std_msgs
