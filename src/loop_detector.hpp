// SPDX-License-Identifier: MIT
// Copyright (c) 2026 RyuYamamoto

#ifndef LIDAR_SLAM__LOOP_DETECTOR_HPP_
#define LIDAR_SLAM__LOOP_DETECTOR_HPP_

#include "keyframe_manager.hpp"
#include "scan_matcher_base.hpp"

#include <cstddef>
#include <vector>

namespace lidar_slam
{

struct LoopParameters
{
  bool enable{true};
  double search_radius{3.0};      // [m] candidate search radius in the map frame
  std::size_t min_id_gap{30};     // minimum keyframe index gap to qualify as a loop
  std::size_t submap_size{5};     // half-window of the candidate submap
  double fitness_threshold{0.3};  // ICP residual gate to accept a loop [m]
  std::size_t max_candidates{1};  // max loops added per detection call
};

struct LoopConstraint
{
  std::size_t from{0};                                        // candidate (older) keyframe id
  std::size_t to{0};                                          // current keyframe id
  Eigen::Isometry2d relative{Eigen::Isometry2d::Identity()};  // from -> to
  // ICP Hessian (J^T J) at the match, ordered (x, y, yaw). The caller turns this
  // into an anisotropic noise so corridor-degenerate loops are not over-trusted.
  Eigen::Matrix3d hessian{Eigen::Matrix3d::Identity()};
  double fitness{0.0};
};

// Detects loop closures by searching spatially-near past keyframes and verifying
// them with a dedicated ICP matcher. The matcher is owned by the caller.
class LoopDetector
{
public:
  LoopDetector(const LoopParameters & parameters, ScanMatcherBase * matcher);

  std::vector<LoopConstraint> detect(
    const KeyframeManager & keyframe_manager, std::size_t current_id) const;

private:
  LoopParameters parameters_;
  ScanMatcherBase * matcher_;
};

}  // namespace lidar_slam

#endif  // LIDAR_SLAM__LOOP_DETECTOR_HPP_
