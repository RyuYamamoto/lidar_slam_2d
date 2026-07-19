// SPDX-License-Identifier: MIT
// Copyright (c) 2026 RyuYamamoto

#include "keyframe_manager.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace lidar_slam
{

std::size_t KeyframeManager::add_keyframe(Keyframe keyframe)
{
  keyframe.id = keyframes_.size();
  keyframes_.emplace_back(std::move(keyframe));
  return keyframes_.back().id;
}

bool KeyframeManager::should_create_keyframe(
  const Eigen::Isometry2d & odom_pose, const double trans_threshold,
  const double rot_threshold) const
{
  if (keyframes_.empty()) return true;

  const Eigen::Isometry2d delta = latest().odom_pose.inverse() * odom_pose;
  const double translation = delta.translation().norm();
  const double rotation = std::abs(get_yaw(delta));
  return translation > trans_threshold || rotation > rot_threshold;
}

void KeyframeManager::update_poses(const std::vector<Eigen::Isometry2d> & poses)
{
  const std::size_t count = std::min(poses.size(), keyframes_.size());
  for (std::size_t i = 0; i < count; ++i) keyframes_[i].pose = poses[i];
}

std::vector<Eigen::Vector2d> KeyframeManager::build_submap_in_frame(
  const Eigen::Isometry2d & reference_pose, const std::size_t submap_size) const
{
  std::vector<Eigen::Vector2d> submap;
  if (keyframes_.empty()) return submap;

  const std::size_t count = std::min(submap_size, keyframes_.size());
  const std::size_t start = keyframes_.size() - count;
  const Eigen::Isometry2d reference_inverse = reference_pose.inverse();

  for (std::size_t i = start; i < keyframes_.size(); ++i) {
    const Keyframe & keyframe = keyframes_[i];
    // Express this keyframe's base-frame points in the reference frame:
    //   p_ref = reference^{-1} * keyframe.pose * p_base
    const Eigen::Isometry2d transform = reference_inverse * keyframe.pose;
    submap.reserve(submap.size() + keyframe.scan_points.size());
    for (const auto & point : keyframe.scan_points) {
      submap.emplace_back(transform * point);
    }
  }
  return submap;
}

std::vector<Eigen::Vector2d> KeyframeManager::build_submap_around(
  const std::size_t center_id, const std::size_t window,
  const Eigen::Isometry2d & reference_pose) const
{
  std::vector<Eigen::Vector2d> submap;
  if (keyframes_.empty() || center_id >= keyframes_.size()) return submap;

  const std::size_t start = center_id >= window ? center_id - window : 0;
  const std::size_t end = std::min(keyframes_.size() - 1, center_id + window);
  const Eigen::Isometry2d reference_inverse = reference_pose.inverse();

  for (std::size_t i = start; i <= end; ++i) {
    const Keyframe & keyframe = keyframes_[i];
    const Eigen::Isometry2d transform = reference_inverse * keyframe.pose;
    submap.reserve(submap.size() + keyframe.scan_points.size());
    for (const auto & point : keyframe.scan_points) {
      submap.emplace_back(transform * point);
    }
  }
  return submap;
}

}  // namespace lidar_slam
