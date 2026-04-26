#pragma once
#include <sensor_msgs/msg/laser_scan.hpp>
#include <memory>

namespace sensor_msgs {
using LaserScan = msg::LaserScan;
using LaserScanPtr = LaserScan::SharedPtr;
using LaserScanConstPtr = LaserScan::ConstSharedPtr;
}  // namespace sensor_msgs
