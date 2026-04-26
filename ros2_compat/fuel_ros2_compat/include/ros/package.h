#pragma once

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace ros {
namespace package {
inline std::string getPath(const std::string& package_name) {
  return ament_index_cpp::get_package_share_directory(package_name);
}
}  // namespace package
}  // namespace ros
