// SPDX-License-Identifier: MIT
// Copyright (c) 2026 RyuYamamoto

#ifndef LIDAR_SLAM__ICP_SCAN_MATCHER_HPP_
#define LIDAR_SLAM__ICP_SCAN_MATCHER_HPP_

#include "scan_matcher_base.hpp"

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace lidar_slam
{

enum class IcpVariant { PointToPoint, PointToLine };

struct IcpParameters
{
  IcpVariant variant{IcpVariant::PointToPoint};
  int max_iterations{30};
  double max_correspondence_dist{1.0};
  double convergence_eps{1.0e-4};
  double fitness_threshold{0.5};
  int normal_neighbor_num{5};  // neighbors used to estimate a point-to-line normal
};

// Point-to-point / point-to-line ICP with a grid-based nearest neighbor search.
class IcpScanMatcher : public ScanMatcherBase
{
public:
  explicit IcpScanMatcher(const IcpParameters & parameters);

  MatchResult align(
    const std::vector<Eigen::Vector2d> & source, const std::vector<Eigen::Vector2d> & target,
    const Eigen::Isometry2d & init_guess) override;

  static IcpVariant variant_from_string(const std::string & name);

private:
  // Uniform grid spatial index over the target points for nearest neighbor queries.
  class GridIndex
  {
  public:
    GridIndex(const std::vector<Eigen::Vector2d> & points, double cell_size);

    // Index of the nearest target point within max_dist, or -1 if none.
    int64_t nearest(const Eigen::Vector2d & query, double max_dist) const;

  private:
    int64_t hash(int cx, int cy) const;

    const std::vector<Eigen::Vector2d> & points_;
    double cell_size_;
    std::unordered_map<int64_t, std::vector<int64_t>> cells_;
  };

  Eigen::Vector2d estimate_normal(
    const std::vector<Eigen::Vector2d> & target, const Eigen::Vector2d & point) const;

  // Gauss-Newton Hessian (J^T J) over correspondences, ordered (x, y, yaw).
  Eigen::Matrix3d compute_hessian(
    const std::vector<Eigen::Vector2d> & source, const std::vector<Eigen::Vector2d> & target,
    const Eigen::Isometry2d & transform) const;

  IcpParameters parameters_;
};

}  // namespace lidar_slam

#endif  // LIDAR_SLAM__ICP_SCAN_MATCHER_HPP_
