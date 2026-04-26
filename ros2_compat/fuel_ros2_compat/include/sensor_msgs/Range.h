#pragma once
#include <sensor_msgs/msg/range.hpp>
#include <memory>

namespace sensor_msgs {
using Range = msg::Range;
using RangePtr = Range::SharedPtr;
using RangeConstPtr = Range::ConstSharedPtr;
}  // namespace sensor_msgs
