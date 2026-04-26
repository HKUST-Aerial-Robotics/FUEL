#pragma once
#include <std_msgs/msg/header.hpp>
#include <memory>

namespace std_msgs {
using Header = msg::Header;
using HeaderPtr = Header::SharedPtr;
using HeaderConstPtr = Header::ConstSharedPtr;
}  // namespace std_msgs
