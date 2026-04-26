#pragma once
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <memory>

namespace sensor_msgs {
using PointCloud2 = msg::PointCloud2;
using PointCloud2Ptr = PointCloud2::SharedPtr;
using PointCloud2ConstPtr = PointCloud2::ConstSharedPtr;
}  // namespace sensor_msgs
