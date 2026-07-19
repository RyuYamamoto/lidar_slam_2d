// SPDX-License-Identifier: MIT
// Copyright (c) 2026 RyuYamamoto

#ifndef LIDAR_SLAM__SCAN_MATCHER_BASE_HPP_
#define LIDAR_SLAM__SCAN_MATCHER_BASE_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <limits>
#include <vector>

namespace lidar_slam
{

struct MatchResult
{
  Eigen::Isometry2d relative_pose{Eigen::Isometry2d::Identity()};  // source pose in target frame
  bool converged{false};
  double fitness{std::numeric_limits<double>::max()};        // mean correspondence residual [m]
  Eigen::Matrix3d information{Eigen::Matrix3d::Identity()};  // Hessian (J^T J), order (x, y, yaw)
};

// Base interface for 2D scan matchers.
class ScanMatcherBase
{
public:
  virtual ~ScanMatcherBase() = default;

  // Aligns source onto target. init_guess and the returned relative_pose both map
  // source into the target frame.
  virtual MatchResult align(
    const std::vector<Eigen::Vector2d> & source, const std::vector<Eigen::Vector2d> & target,
    const Eigen::Isometry2d & init_guess) = 0;
};

}  // namespace lidar_slam

#endif  // LIDAR_SLAM__SCAN_MATCHER_BASE_HPP_
