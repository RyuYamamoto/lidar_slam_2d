// SPDX-License-Identifier: MIT
// Copyright (c) 2026 RyuYamamoto

#include "pose_graph.hpp"

#include "se2.hpp"

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

namespace lidar_slam
{

namespace
{

gtsam::Key pose_key(const std::size_t id)
{
  return gtsam::Symbol('x', id);
}

gtsam::Pose2 to_pose2(const Eigen::Isometry2d & pose)
{
  return gtsam::Pose2(get_x(pose), get_y(pose), get_yaw(pose));
}

Eigen::Isometry2d from_pose2(const gtsam::Pose2 & pose)
{
  return make_se2(pose.x(), pose.y(), pose.theta());
}

}  // namespace

PoseGraph::PoseGraph()
{
  gtsam::ISAM2Params params;
  params.relinearizeThreshold = 0.01;
  params.relinearizeSkip = 1;
  isam2_ = gtsam::ISAM2(params);
}

void PoseGraph::add_prior(
  const std::size_t id, const Eigen::Isometry2d & pose, const Eigen::Matrix3d & info)
{
  const auto noise = gtsam::noiseModel::Gaussian::Information(info);
  new_factors_.emplace_shared<gtsam::PriorFactor<gtsam::Pose2>>(
    pose_key(id), to_pose2(pose), noise);
  new_values_.insert(pose_key(id), to_pose2(pose));
  node_ids_.emplace_back(id);
}

void PoseGraph::add_between(
  const std::size_t from, const std::size_t to, const Eigen::Isometry2d & relative,
  const Eigen::Matrix3d & info)
{
  const auto noise = gtsam::noiseModel::Gaussian::Information(info);
  new_factors_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(
    pose_key(from), pose_key(to), to_pose2(relative), noise);

  // Seed the initial value of the new node from the latest estimate of `from`.
  const Eigen::Isometry2d from_pose = get_pose(from);
  new_values_.insert(pose_key(to), to_pose2(from_pose * relative));
  node_ids_.emplace_back(to);
}

void PoseGraph::add_relative_factor(
  const std::size_t from, const std::size_t to, const Eigen::Isometry2d & relative,
  const Eigen::Matrix3d & info)
{
  const auto noise = gtsam::noiseModel::Gaussian::Information(info);
  new_factors_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(
    pose_key(from), pose_key(to), to_pose2(relative), noise);
}

void PoseGraph::add_loop_closure(
  const std::size_t from, const std::size_t to, const Eigen::Isometry2d & relative,
  const Eigen::Matrix3d & info, const bool use_robust, const double huber_delta)
{
  gtsam::SharedNoiseModel noise = gtsam::noiseModel::Gaussian::Information(info);
  if (use_robust) {
    noise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(huber_delta), noise);
  }
  // Both nodes already exist, so only the factor is added (no value insertion).
  new_factors_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(
    pose_key(from), pose_key(to), to_pose2(relative), noise);
}

void PoseGraph::optimize()
{
  isam2_.update(new_factors_, new_values_);
  new_factors_.resize(0);
  new_values_.clear();
  estimate_ = isam2_.calculateEstimate();
}

Eigen::Isometry2d PoseGraph::get_pose(const std::size_t id) const
{
  // Prefer the optimized estimate; fall back to the pending initial value for a
  // node that has not been through an update yet.
  if (estimate_.exists(pose_key(id))) {
    return from_pose2(estimate_.at<gtsam::Pose2>(pose_key(id)));
  }
  if (new_values_.exists(pose_key(id))) {
    return from_pose2(new_values_.at<gtsam::Pose2>(pose_key(id)));
  }
  return Eigen::Isometry2d::Identity();
}

std::vector<Eigen::Isometry2d> PoseGraph::get_all_poses() const
{
  std::vector<Eigen::Isometry2d> poses;
  poses.reserve(node_ids_.size());
  for (const std::size_t id : node_ids_) poses.emplace_back(get_pose(id));
  return poses;
}

}  // namespace lidar_slam
