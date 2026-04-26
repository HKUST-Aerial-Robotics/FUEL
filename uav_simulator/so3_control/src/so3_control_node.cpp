#include <rclcpp/rclcpp.hpp>
#include <so3_control/so3_control_component.h>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  rclcpp::spin(std::make_shared<so3_control::SO3ControlComponent>(options));
  rclcpp::shutdown();
  return 0;
}
