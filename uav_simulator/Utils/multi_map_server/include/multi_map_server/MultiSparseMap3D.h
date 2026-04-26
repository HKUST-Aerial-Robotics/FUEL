#pragma once
#include <multi_map_server/msg/multi_sparse_map3_d.hpp>
#include <memory>

namespace multi_map_server {
using MultiSparseMap3D = msg::MultiSparseMap3D;
using MultiSparseMap3DPtr = MultiSparseMap3D::SharedPtr;
using MultiSparseMap3DConstPtr = MultiSparseMap3D::ConstSharedPtr;
}  // namespace multi_map_server
