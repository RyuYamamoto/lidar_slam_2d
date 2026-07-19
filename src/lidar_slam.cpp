// SPDX-License-Identifier: MIT
// Copyright (c) 2026 RyuYamamoto

#include "lidar_slam.hpp"

#include "se2.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <utility>

namespace lidar_slam
{

namespace
{

Eigen::Matrix3d make_information(const double sigma_xy, const double sigma_yaw, const double scale)
{
  const double sxy = sigma_xy * scale;
  const double syaw = sigma_yaw * scale;
  Eigen::Matrix3d information = Eigen::Matrix3d::Zero();
  information(0, 0) = 1.0 / (sxy * sxy);
  information(1, 1) = 1.0 / (sxy * sxy);
  information(2, 2) = 1.0 / (syaw * syaw);
  return information;
}

// Anisotropic information from an ICP Hessian: the best-constrained direction
// reaches the base information, the degenerate one is floored, yaw stays fixed.
Eigen::Matrix3d information_from_hessian(
  const Eigen::Matrix3d & hessian, const double sigma_xy, const double sigma_yaw,
  const double floor_scale)
{
  const double info_max = 1.0 / (sigma_xy * sigma_xy);
  const double floor_sigma = sigma_xy * floor_scale;
  const double info_floor = 1.0 / (floor_sigma * floor_sigma);

  const Eigen::Matrix2d hessian_translation = hessian.topLeftCorner<2, 2>();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(hessian_translation);

  Eigen::Matrix3d information = Eigen::Matrix3d::Zero();
  information(2, 2) = 1.0 / (sigma_yaw * sigma_yaw);

  if (solver.info() != Eigen::Success) {
    information(0, 0) = info_max;
    information(1, 1) = info_max;
    return information;
  }

  const Eigen::Vector2d eigenvalues = solver.eigenvalues();  // ascending
  const Eigen::Matrix2d eigenvectors = solver.eigenvectors();
  const double eigen_max = std::max(eigenvalues(1), 1e-12);

  Eigen::Vector2d scaled;
  for (int i = 0; i < 2; ++i) {
    const double relative = std::max(0.0, eigenvalues(i)) / eigen_max;
    scaled(i) = std::clamp(relative * info_max, info_floor, info_max);
  }

  information.topLeftCorner<2, 2>() = eigenvectors * scaled.asDiagonal() * eigenvectors.transpose();
  return information;
}

}  // namespace

LidarSlam::LidarSlam(
  const CoreParameters & parameters, const IcpParameters & icp_parameters,
  const IcpParameters & loop_icp_parameters, const LoopParameters & loop_parameters,
  const MapParameters & map_parameters)
: parameters_(parameters),
  scan_matcher_(std::make_unique<IcpScanMatcher>(icp_parameters)),
  loop_scan_matcher_(std::make_unique<IcpScanMatcher>(loop_icp_parameters)),
  loop_detector_(std::make_unique<LoopDetector>(loop_parameters, loop_scan_matcher_.get())),
  mapper_(std::make_unique<OccupancyGridMapper>(map_parameters))
{
}

ScanResult LidarSlam::add_scan(
  const double stamp, std::vector<Eigen::Vector2d> base_points, const Eigen::Isometry2d & odom_pose,
  const Eigen::Vector2d & sensor_origin)
{
  ScanResult result;

  // First keyframe: anchor the map frame at the current odom pose (map == odom).
  if (keyframe_manager_.empty()) {
    Keyframe keyframe;
    keyframe.stamp = stamp;
    keyframe.pose = odom_pose;
    keyframe.odom_pose = odom_pose;
    keyframe.scan_points = std::move(base_points);
    keyframe.sensor_origin = sensor_origin;
    const std::size_t id = keyframe_manager_.add_keyframe(std::move(keyframe));
    pose_graph_.add_prior(
      id, odom_pose,
      make_information(parameters_.prior_sigma_xy, parameters_.prior_sigma_yaw, 1.0));
    pose_graph_.optimize();
    keyframe_manager_.update_poses(pose_graph_.get_all_poses());
    result.keyframe_added = true;
    return result;
  }

  if (!keyframe_manager_.should_create_keyframe(
        odom_pose, parameters_.trans_threshold, parameters_.rot_threshold)) {
    return result;
  }

  const Keyframe & last = keyframe_manager_.latest();
  const std::size_t last_id = last.id;
  const Eigen::Isometry2d delta_odom = last.odom_pose.inverse() * odom_pose;

  const std::vector<Eigen::Vector2d> target =
    keyframe_manager_.build_submap_in_frame(last.pose, parameters_.submap_size);
  const MatchResult match = scan_matcher_->align(base_points, target, delta_odom);

  // ICP factor uses anisotropic information from its Hessian (weak along a
  // degenerate corridor axis); a layered odom factor constrains that axis.
  Eigen::Isometry2d relative;
  Eigen::Matrix3d information;
  bool add_odom_factor = true;
  if (match.converged) {
    relative = match.relative_pose;
    information =
      parameters_.use_icp_hessian_info
        ? information_from_hessian(
            match.information, parameters_.between_sigma_xy, parameters_.between_sigma_yaw,
            parameters_.odom_fallback_sigma_scale)
        : make_information(parameters_.between_sigma_xy, parameters_.between_sigma_yaw, 1.0);
  } else {
    relative = delta_odom;
    information = make_information(
      parameters_.between_sigma_xy, parameters_.between_sigma_yaw,
      parameters_.odom_fallback_sigma_scale);
    add_odom_factor = false;
    result.icp_failed = true;
  }

  Keyframe keyframe;
  keyframe.stamp = stamp;
  keyframe.pose = last.pose * relative;
  keyframe.odom_pose = odom_pose;
  keyframe.scan_points = std::move(base_points);
  keyframe.sensor_origin = sensor_origin;
  const std::size_t new_id = keyframe_manager_.add_keyframe(std::move(keyframe));

  pose_graph_.add_between(last_id, new_id, relative, information);
  if (add_odom_factor) {
    pose_graph_.add_relative_factor(
      last_id, new_id, delta_odom,
      make_information(parameters_.odom_between_sigma_xy, parameters_.odom_between_sigma_yaw, 1.0));
  }
  pose_graph_.optimize();
  keyframe_manager_.update_poses(pose_graph_.get_all_poses());
  result.keyframe_added = true;

  const std::vector<LoopConstraint> loops = loop_detector_->detect(keyframe_manager_, new_id);
  if (!loops.empty()) {
    for (const auto & loop : loops) {
      const Eigen::Matrix3d loop_information =
        parameters_.use_icp_hessian_info
          ? information_from_hessian(
              loop.hessian, parameters_.loop_sigma_xy, parameters_.loop_sigma_yaw,
              parameters_.odom_fallback_sigma_scale)
          : make_information(parameters_.loop_sigma_xy, parameters_.loop_sigma_yaw, 1.0);
      pose_graph_.add_loop_closure(
        loop.from, loop.to, loop.relative, loop_information, parameters_.loop_use_robust,
        parameters_.loop_huber_delta);
      result.loops.push_back({loop.from, loop.to, loop.fitness});
    }
    for (int i = 0; i < std::max(1, parameters_.loop_extra_update_iterations); ++i) {
      pose_graph_.optimize();
    }
    keyframe_manager_.update_poses(pose_graph_.get_all_poses());
  }

  return result;
}

Eigen::Isometry2d LidarSlam::map_to_odom(const Eigen::Isometry2d & odom_now) const
{
  if (keyframe_manager_.empty()) return Eigen::Isometry2d::Identity();
  const Keyframe & last = keyframe_manager_.latest();
  const Eigen::Isometry2d map_base_now = last.pose * (last.odom_pose.inverse() * odom_now);
  return map_base_now * odom_now.inverse();
}

GridMap LidarSlam::build_map() const
{
  return mapper_->build_map(keyframe_manager_.keyframes());
}

std::vector<Eigen::Isometry2d> LidarSlam::keyframe_poses() const
{
  std::vector<Eigen::Isometry2d> poses;
  poses.reserve(keyframe_manager_.size());
  for (const auto & keyframe : keyframe_manager_.keyframes()) poses.emplace_back(keyframe.pose);
  return poses;
}

}  // namespace lidar_slam
