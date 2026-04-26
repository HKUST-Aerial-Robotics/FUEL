#pragma once

#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/corrections.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <quadrotor_msgs/msg/so3_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <so3_control/SO3Control.h>
#include <std_msgs/msg/bool.hpp>

namespace so3_control {

class SO3ControlComponent : public rclcpp::Node {
public:
  explicit SO3ControlComponent(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  template <typename T>
  T getParameterOrDeclare(const std::string& name, const T& default_value) {
    if (!has_parameter(name)) {
      declare_parameter<T>(name, default_value);
    }
    return get_parameter(name).get_value<T>();
  }

  void publishSO3Command();
  void positionCmdCallback(const quadrotor_msgs::msg::PositionCommand::ConstSharedPtr cmd);
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom);
  void enableMotorsCallback(const std_msgs::msg::Bool::ConstSharedPtr msg);
  void correctionsCallback(const quadrotor_msgs::msg::Corrections::ConstSharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr imu);

  SO3Control controller_;
  rclcpp::Publisher<quadrotor_msgs::msg::SO3Command>::SharedPtr so3_command_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr position_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_motors_sub_;
  rclcpp::Subscription<quadrotor_msgs::msg::Corrections>::SharedPtr corrections_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  bool position_cmd_updated_{false};
  bool position_cmd_init_{false};
  std::string frame_id_;

  Eigen::Vector3d des_pos_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d des_vel_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d des_acc_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d kx_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d kv_{Eigen::Vector3d::Zero()};
  double des_yaw_{0.0};
  double des_yaw_dot_{0.0};
  double current_yaw_{0.0};
  bool enable_motors_{true};
  bool use_external_yaw_{false};
  double kR_[3]{1.5, 1.5, 1.0};
  double kOm_[3]{0.13, 0.13, 0.1};
  double corrections_[3]{0.0, 0.0, 0.0};
};

}  // namespace so3_control
