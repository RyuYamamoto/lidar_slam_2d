// SPDX-License-Identifier: MIT
// Copyright (c) 2026 RyuYamamoto

#ifndef LIDAR_SLAM__POSE_GRAPH_HPP_
#define LIDAR_SLAM__POSE_GRAPH_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <cstddef>
#include <vector>

namespace lidar_slam
{

// Thin wrapper around GTSAM iSAM2. GTSAM types are confined to this class so the
// rest of the pipeline only sees Eigen::Isometry2d.
class PoseGraph
{
public:
  PoseGraph();

  // Adds a Pose2 prior and the initial value for the given node.
  void add_prior(std::size_t id, const Eigen::Isometry2d & pose, const Eigen::Matrix3d & info);

  // Adds a BetweenFactor(Pose2) between two nodes. The initial value of `to` is
  // seeded from the current estimate of `from` composed with `relative`.
  void add_between(
    std::size_t from, std::size_t to, const Eigen::Isometry2d & relative,
    const Eigen::Matrix3d & info);

  // Adds a BetweenFactor between two existing nodes (no new value inserted). Used
  // to layer the odometry constraint on top of the ICP one.
  void add_relative_factor(
    std::size_t from, std::size_t to, const Eigen::Isometry2d & relative,
    const Eigen::Matrix3d & info);

  // Adds a loop closure BetweenFactor between two existing nodes, optionally with a
  // Huber robust kernel against false positives.
  void add_loop_closure(
    std::size_t from, std::size_t to, const Eigen::Isometry2d & relative,
    const Eigen::Matrix3d & info, bool use_robust, double huber_delta);

  // Runs one incremental iSAM2 update and refreshes the cached estimate.
  void optimize();

  Eigen::Isometry2d get_pose(std::size_t id) const;
  std::vector<Eigen::Isometry2d> get_all_poses() const;

private:
  gtsam::ISAM2 isam2_;
  gtsam::NonlinearFactorGraph new_factors_;
  gtsam::Values new_values_;
  gtsam::Values estimate_;
  std::vector<std::size_t> node_ids_;
};

}  // namespace lidar_slam

#endif  // LIDAR_SLAM__POSE_GRAPH_HPP_
