#pragma once
#include <bspline/msg/bspline.hpp>
#include <memory>

namespace bspline {
using Bspline = msg::Bspline;
using BsplinePtr = Bspline::SharedPtr;
using BsplineConstPtr = Bspline::ConstSharedPtr;
}  // namespace bspline
