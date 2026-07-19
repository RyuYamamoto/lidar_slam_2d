// SPDX-License-Identifier: MIT
// Copyright (c) 2026 RyuYamamoto

#ifndef LIDAR_SLAM__SE2_HPP_
#define LIDAR_SLAM__SE2_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>

namespace lidar_slam
{

// 2D rigid transform helpers on Eigen::Isometry2d.

inline Eigen::Isometry2d make_se2(const double x, const double y, const double yaw)
{
  Eigen::Isometry2d transform = Eigen::Isometry2d::Identity();
  transform.linear() = Eigen::Rotation2Dd(yaw).toRotationMatrix();
  transform.translation() = Eigen::Vector2d(x, y);
  return transform;
}

inline double get_yaw(const Eigen::Isometry2d & transform)
{
  const Eigen::Matrix2d & rotation = transform.linear();
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

inline double get_x(const Eigen::Isometry2d & transform)
{
  return transform.translation().x();
}

inline double get_y(const Eigen::Isometry2d & transform)
{
  return transform.translation().y();
}

// Interpolates two poses (translation lerp + shortest-angle) with ratio in [0, 1].
inline Eigen::Isometry2d interpolate_se2(
  const Eigen::Isometry2d & from, const Eigen::Isometry2d & to, const double ratio)
{
  const Eigen::Vector2d translation = (1.0 - ratio) * from.translation() + ratio * to.translation();

  const double yaw_from = get_yaw(from);
  double delta_yaw = get_yaw(to) - yaw_from;
  delta_yaw = std::atan2(std::sin(delta_yaw), std::cos(delta_yaw));

  return make_se2(translation.x(), translation.y(), yaw_from + ratio * delta_yaw);
}

}  // namespace lidar_slam

#endif  // LIDAR_SLAM__SE2_HPP_
