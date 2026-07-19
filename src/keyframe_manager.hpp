// SPDX-License-Identifier: MIT
// Copyright (c) 2026 RyuYamamoto

#ifndef LIDAR_SLAM__KEYFRAME_MANAGER_HPP_
#define LIDAR_SLAM__KEYFRAME_MANAGER_HPP_

#include "keyframe.hpp"

#include <cstddef>
#include <vector>

namespace lidar_slam
{

// Owns the keyframe sequence (value semantics, single owner) and provides the
// keyframe generation decision plus submap construction helpers.
class KeyframeManager
{
public:
  bool empty() const { return keyframes_.empty(); }
  std::size_t size() const { return keyframes_.size(); }

  const std::vector<Keyframe> & keyframes() const { return keyframes_; }
  const Keyframe & latest() const { return keyframes_.back(); }

  // Appends a new keyframe and returns its assigned id.
  std::size_t add_keyframe(Keyframe keyframe);

  // Decides whether the current odom pose warrants a new keyframe relative to the
  // latest keyframe's odom pose.
  bool should_create_keyframe(
    const Eigen::Isometry2d & odom_pose, double trans_threshold, double rot_threshold) const;

  // Overwrites keyframe poses with the latest optimized estimates (order-aligned).
  void update_poses(const std::vector<Eigen::Isometry2d> & poses);

  // Submap from the last `submap_size` keyframes, expressed in reference_pose frame.
  std::vector<Eigen::Vector2d> build_submap_in_frame(
    const Eigen::Isometry2d & reference_pose, std::size_t submap_size) const;

  // Submap from keyframes in [center_id - window, center_id + window], expressed in
  // reference_pose frame. Used for loop closure candidate matching.
  std::vector<Eigen::Vector2d> build_submap_around(
    std::size_t center_id, std::size_t window, const Eigen::Isometry2d & reference_pose) const;

private:
  std::vector<Keyframe> keyframes_;
};

}  // namespace lidar_slam

#endif  // LIDAR_SLAM__KEYFRAME_MANAGER_HPP_
