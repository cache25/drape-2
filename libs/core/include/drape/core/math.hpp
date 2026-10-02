#pragma once

#include <Eigen/Dense>

namespace drape {

using Vec2d = Eigen::Vector2d;
using Vec3d = Eigen::Vector3d;
using Vec3f = Eigen::Vector3f;
using Mat2d = Eigen::Matrix2d;
using Mat3d = Eigen::Matrix3d;

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kGravity = 9.81;

}  // namespace drape
