#pragma once

#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include <ros/ros.h>

namespace tf {

using Quaternion = tf2::Quaternion;
using Vector3 = tf2::Vector3;
using Transform = tf2::Transform;
using Pose = tf2::Transform;

inline geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw) {
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
}

inline double getYaw(const geometry_msgs::msg::Quaternion& q) { return tf2::getYaw(q); }

template <typename T>
class Stamped {
public:
  Stamped(const T& data, const ros::Time& stamp, const std::string& frame_id)
      : data_(data), stamp_(stamp), frame_id_(frame_id) {}

  const T& getData() const { return data_; }
  const ros::Time& stamp() const { return stamp_; }
  const std::string& frame_id() const { return frame_id_; }

private:
  T data_;
  ros::Time stamp_;
  std::string frame_id_;
};

class StampedTransform {
public:
  StampedTransform(const Transform& transform, const ros::Time& stamp,
                   const std::string& frame_id, const std::string& child_frame_id)
      : transform_(transform), stamp_(stamp), frame_id_(frame_id), child_frame_id_(child_frame_id) {}

  geometry_msgs::msg::TransformStamped toMsg() const {
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = stamp_;
    msg.header.frame_id = frame_id_;
    msg.child_frame_id = child_frame_id_;
    msg.transform = tf2::toMsg(transform_);
    return msg;
  }

private:
  Transform transform_;
  ros::Time stamp_;
  std::string frame_id_;
  std::string child_frame_id_;
};

inline void poseStampedTFToMsg(const Stamped<Pose>& pose, geometry_msgs::msg::PoseStamped& msg) {
  msg.header.stamp = pose.stamp();
  msg.header.frame_id = pose.frame_id();
  const auto& transform = pose.getData();
  msg.pose.position.x = transform.getOrigin().x();
  msg.pose.position.y = transform.getOrigin().y();
  msg.pose.position.z = transform.getOrigin().z();
  msg.pose.orientation = tf2::toMsg(transform.getRotation());
}

}  // namespace tf
