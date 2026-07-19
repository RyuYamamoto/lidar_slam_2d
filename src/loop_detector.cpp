// SPDX-License-Identifier: MIT
// Copyright (c) 2026 RyuYamamoto

#include "loop_detector.hpp"

#include "se2.hpp"

#include <algorithm>
#include <utility>

namespace lidar_slam
{

LoopDetector::LoopDetector(const LoopParameters & parameters, ScanMatcherBase * matcher)
: parameters_(parameters), matcher_(matcher)
{
}

std::vector<LoopConstraint> LoopDetector::detect(
  const KeyframeManager & keyframe_manager, const std::size_t current_id) const
{
  std::vector<LoopConstraint> constraints;
  if (!parameters_.enable || matcher_ == nullptr) return constraints;

  const std::vector<Keyframe> & keyframes = keyframe_manager.keyframes();
  if (current_id >= keyframes.size() || current_id < parameters_.min_id_gap) return constraints;

  const Keyframe & current = keyframes[current_id];

  // Candidates: near in space, far in index. Sorted nearest first.
  std::vector<std::pair<double, std::size_t>> candidates;
  const std::size_t last_candidate = current_id - parameters_.min_id_gap;
  for (std::size_t id = 0; id <= last_candidate; ++id) {
    const double distance = (current.pose.translation() - keyframes[id].pose.translation()).norm();
    if (distance < parameters_.search_radius) candidates.emplace_back(distance, id);
  }
  std::sort(candidates.begin(), candidates.end());

  std::size_t accepted = 0;
  for (const auto & candidate_entry : candidates) {
    if (accepted >= parameters_.max_candidates) break;

    const std::size_t candidate_id = candidate_entry.second;
    const Keyframe & candidate = keyframes[candidate_id];
    const std::vector<Eigen::Vector2d> target =
      keyframe_manager.build_submap_around(candidate_id, parameters_.submap_size, candidate.pose);
    const Eigen::Isometry2d init_guess = candidate.pose.inverse() * current.pose;

    const MatchResult match = matcher_->align(current.scan_points, target, init_guess);
    if (!match.converged || match.fitness > parameters_.fitness_threshold) continue;

    LoopConstraint constraint;
    constraint.from = candidate_id;
    constraint.to = current_id;
    constraint.relative = match.relative_pose;  // candidate -> current
    constraint.hessian = match.information;
    constraint.fitness = match.fitness;
    constraints.emplace_back(constraint);
    ++accepted;
  }

  return constraints;
}

}  // namespace lidar_slam
