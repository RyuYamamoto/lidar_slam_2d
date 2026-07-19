// SPDX-License-Identifier: MIT
// Copyright (c) 2026 RyuYamamoto

#ifndef LIDAR_SLAM__KEYFRAME_HPP_
#define LIDAR_SLAM__KEYFRAME_HPP_

#include "se2.hpp"

#include <cstddef>
#include <vector>

namespace lidar_slam
{

// A pose graph node: optimized map-frame pose plus the base-frame scan points
// captured at creation time (which do not depend on the pose).
struct Keyframe
{
  std::size_t id{0};
  double stamp{0.0};                                           // seconds
  Eigen::Isometry2d pose{Eigen::Isometry2d::Identity()};       // map frame, updated by the back-end
  Eigen::Isometry2d odom_pose{Eigen::Isometry2d::Identity()};  // odom frame at creation
  std::vector<Eigen::Vector2d> scan_points;                    // base frame, immutable
  Eigen::Vector2d sensor_origin{Eigen::Vector2d::Zero()};      // base-frame sensor origin
};

}  // namespace lidar_slam

#endif  // LIDAR_SLAM__KEYFRAME_HPP_
