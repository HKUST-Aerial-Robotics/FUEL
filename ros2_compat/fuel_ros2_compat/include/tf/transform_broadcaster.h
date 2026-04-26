#pragma once

#include <memory>

#include <tf/tf.h>
#include <tf2_ros/transform_broadcaster.h>

namespace tf {

class TransformBroadcaster {
public:
  TransformBroadcaster()
      : broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(ros::global_node())) {}

  void sendTransform(const StampedTransform& transform) {
    broadcaster_->sendTransform(transform.toMsg());
  }

private:
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};

}  // namespace tf
