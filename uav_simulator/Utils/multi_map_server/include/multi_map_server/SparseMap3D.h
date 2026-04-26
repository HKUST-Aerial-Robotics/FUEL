#pragma once
#include <multi_map_server/msg/sparse_map3_d.hpp>
#include <memory>

namespace multi_map_server {
using SparseMap3D = msg::SparseMap3D;
using SparseMap3DPtr = SparseMap3D::SharedPtr;
using SparseMap3DConstPtr = SparseMap3D::ConstSharedPtr;
}  // namespace multi_map_server
