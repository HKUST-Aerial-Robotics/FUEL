#include <so3_control/so3_control_component.h>

#include <tf/transform_datatypes.h>

namespace so3_control {

SO3ControlComponent::SO3ControlComponent(const rclcpp::NodeOptions& options)
    : rclcpp::Node("so3_control", options) {
  const auto quadrotor_name =
      getParameterOrDeclare<std::string>("quadrotor_name", std::string("quadrotor"));
  frame_id_ = "/" + quadrotor_name;

  controller_.setMass(getParameterOrDeclare<double>("mass", 0.74));
  use_external_yaw_ = getParameterOrDeclare<bool>("use_external_yaw", true);

  kR_[0] = getParameterOrDeclare<double>("gains.rot.x", 1.5);
  kR_[1] = getParameterOrDeclare<double>("gains.rot.y", 1.5);
  kR_[2] = getParameterOrDeclare<double>("gains.rot.z", 1.0);
  kOm_[0] = getParameterOrDeclare<double>("gains.ang.x", 0.13);
  kOm_[1] = getParameterOrDeclare<double>("gains.ang.y", 0.13);
  kOm_[2] = getParameterOrDeclare<double>("gains.ang.z", 0.1);

  corrections_[0] = getParameterOrDeclare<double>("corrections.z", 0.0);
  corrections_[1] = getParameterOrDeclare<double>("corrections.r", 0.0);
  corrections_[2] = getParameterOrDeclare<double>("corrections.p", 0.0);

  so3_command_pub_ =
      create_publisher<quadrotor_msgs::msg::SO3Command>("~/so3_cmd", rclcpp::QoS(10));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "~/odom", rclcpp::QoS(10),
      std::bind(&SO3ControlComponent::odomCallback, this, std::placeholders::_1));
  position_cmd_sub_ = create_subscription<quadrotor_msgs::msg::PositionCommand>(
      "~/position_cmd", rclcpp::QoS(10),
      std::bind(&SO3ControlComponent::positionCmdCallback, this, std::placeholders::_1));
  enable_motors_sub_ = create_subscription<std_msgs::msg::Bool>(
      "~/motors", rclcpp::QoS(2),
      std::bind(&SO3ControlComponent::enableMotorsCallback, this, std::placeholders::_1));
  corrections_sub_ = create_subscription<quadrotor_msgs::msg::Corrections>(
      "~/corrections", rclcpp::QoS(10),
      std::bind(&SO3ControlComponent::correctionsCallback, this, std::placeholders::_1));
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "~/imu", rclcpp::QoS(10),
      std::bind(&SO3ControlComponent::imuCallback, this, std::placeholders::_1));
}

void SO3ControlComponent::publishSO3Command() {
  controller_.calculateControl(des_pos_, des_vel_, des_acc_, des_yaw_, des_yaw_dot_, kx_, kv_);

  const Eigen::Vector3d& force = controller_.getComputedForce();
  const Eigen::Quaterniond& orientation = controller_.getComputedOrientation();

  quadrotor_msgs::msg::SO3Command so3_command;
  so3_command.header.stamp = now();
  so3_command.header.frame_id = frame_id_;
  so3_command.force.x = force(0);
  so3_command.force.y = force(1);
  so3_command.force.z = force(2);
  so3_command.orientation.x = orientation.x();
  so3_command.orientation.y = orientation.y();
  so3_command.orientation.z = orientation.z();
  so3_command.orientation.w = orientation.w();
  for (int i = 0; i < 3; ++i) {
    so3_command.kr[i] = kR_[i];
    so3_command.kom[i] = kOm_[i];
  }
  so3_command.aux.current_yaw = current_yaw_;
  so3_command.aux.kf_correction = corrections_[0];
  so3_command.aux.angle_corrections[0] = corrections_[1];
  so3_command.aux.angle_corrections[1] = corrections_[2];
  so3_command.aux.enable_motors = enable_motors_;
  so3_command.aux.use_external_yaw = use_external_yaw_;
  so3_command_pub_->publish(so3_command);
}

void SO3ControlComponent::positionCmdCallback(
    const quadrotor_msgs::msg::PositionCommand::ConstSharedPtr cmd) {
  des_pos_ = Eigen::Vector3d(cmd->position.x, cmd->position.y, cmd->position.z);
  des_vel_ = Eigen::Vector3d(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
  des_acc_ = Eigen::Vector3d(cmd->acceleration.x, cmd->acceleration.y, cmd->acceleration.z);
  kx_ = Eigen::Vector3d(cmd->kx[0], cmd->kx[1], cmd->kx[2]);
  kv_ = Eigen::Vector3d(cmd->kv[0], cmd->kv[1], cmd->kv[2]);

  des_yaw_ = cmd->yaw;
  des_yaw_dot_ = cmd->yaw_dot;
  position_cmd_updated_ = true;
  position_cmd_init_ = true;

  publishSO3Command();
}

void SO3ControlComponent::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom) {
  const Eigen::Vector3d position(odom->pose.pose.position.x, odom->pose.pose.position.y,
                                 odom->pose.pose.position.z);
  const Eigen::Vector3d velocity(odom->twist.twist.linear.x, odom->twist.twist.linear.y,
                                 odom->twist.twist.linear.z);

  current_yaw_ = tf::getYaw(odom->pose.pose.orientation);

  controller_.setPosition(position);
  controller_.setVelocity(velocity);

  if (position_cmd_init_) {
    if (!position_cmd_updated_) {
      publishSO3Command();
    }
    position_cmd_updated_ = false;
  }
}

void SO3ControlComponent::enableMotorsCallback(const std_msgs::msg::Bool::ConstSharedPtr msg) {
  if (msg->data) {
    RCLCPP_INFO(get_logger(), "Enabling motors");
  } else {
    RCLCPP_INFO(get_logger(), "Disabling motors");
  }

  enable_motors_ = msg->data;
}

void SO3ControlComponent::correctionsCallback(
    const quadrotor_msgs::msg::Corrections::ConstSharedPtr msg) {
  corrections_[0] = msg->kf_correction;
  corrections_[1] = msg->angle_corrections[0];
  corrections_[2] = msg->angle_corrections[1];
}

void SO3ControlComponent::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr imu) {
  const Eigen::Vector3d acc(imu->linear_acceleration.x, imu->linear_acceleration.y,
                            imu->linear_acceleration.z);
  controller_.setAcc(acc);
}

}  // namespace so3_control

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(so3_control::SO3ControlComponent)
