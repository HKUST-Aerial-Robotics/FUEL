#pragma once
#include <sensor_msgs/msg/point_cloud.hpp>
#include <memory>

namespace sensor_msgs {
using PointCloud = msg::PointCloud;
using PointCloudPtr = PointCloud::SharedPtr;
using PointCloudConstPtr = PointCloud::ConstSharedPtr;
}  // namespace sensor_msgs
