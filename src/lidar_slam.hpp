// SPDX-License-Identifier: MIT
// Copyright (c) 2026 RyuYamamoto

#ifndef LIDAR_SLAM__LIDAR_SLAM_HPP_
#define LIDAR_SLAM__LIDAR_SLAM_HPP_

#include "icp_scan_matcher.hpp"
#include "keyframe_manager.hpp"
#include "loop_detector.hpp"
#include "occupancy_grid_mapper.hpp"
#include "pose_graph.hpp"
#include "scan_matcher_base.hpp"

#include <cstddef>
#include <memory>
#include <vector>

namespace lidar_slam
{

struct CoreParameters
{
  double trans_threshold{0.5};
  double rot_threshold{0.3};
  std::size_t submap_size{5};

  double prior_sigma_xy{0.01};
  double prior_sigma_yaw{0.01};
  double between_sigma_xy{0.05};
  double between_sigma_yaw{0.03};
  double odom_fallback_sigma_scale{5.0};
  bool use_icp_hessian_info{true};
  double odom_between_sigma_xy{0.1};
  double odom_between_sigma_yaw{0.05};

  double loop_sigma_xy{0.1};
  double loop_sigma_yaw{0.05};
  bool loop_use_robust{true};
  double loop_huber_delta{0.1};
  int loop_extra_update_iterations{2};
};

struct LoopEvent
{
  std::size_t from{0};
  std::size_t to{0};
  double fitness{0.0};
};

struct ScanResult
{
  bool keyframe_added{false};
  bool icp_failed{false};
  std::vector<LoopEvent> loops;
};

// ROS-independent 2D graph SLAM pipeline (ICP front-end + GTSAM iSAM2 back-end +
// loop closure + occupancy mapping). Works entirely in Eigen::Isometry2d; it has no
// notion of ROS messages, TF, frame names or time sources. The ROS node adapts
// inbound/outbound types around it.
class LidarSlam
{
public:
  LidarSlam(
    const CoreParameters & parameters, const IcpParameters & icp_parameters,
    const IcpParameters & loop_icp_parameters, const LoopParameters & loop_parameters,
    const MapParameters & map_parameters);

  // Processes one scan (already converted to base-frame points). odom_pose is the
  // odom->base motion prior at the scan time; sensor_origin is the sensor position
  // in the base frame. Returns whether a keyframe was added plus any loop events.
  ScanResult add_scan(
    double stamp, std::vector<Eigen::Vector2d> base_points, const Eigen::Isometry2d & odom_pose,
    const Eigen::Vector2d & sensor_origin);

  bool has_keyframes() const { return !keyframe_manager_.empty(); }

  // map->odom for the current odom->base, extrapolated from the last keyframe.
  Eigen::Isometry2d map_to_odom(const Eigen::Isometry2d & odom_now) const;

  GridMap build_map() const;

  std::vector<Eigen::Isometry2d> keyframe_poses() const;

private:
  CoreParameters parameters_;
  std::unique_ptr<ScanMatcherBase> scan_matcher_;
  std::unique_ptr<ScanMatcherBase> loop_scan_matcher_;
  std::unique_ptr<LoopDetector> loop_detector_;
  KeyframeManager keyframe_manager_;
  PoseGraph pose_graph_;
  std::unique_ptr<OccupancyGridMapper> mapper_;
};

}  // namespace lidar_slam

#endif  // LIDAR_SLAM__LIDAR_SLAM_HPP_
