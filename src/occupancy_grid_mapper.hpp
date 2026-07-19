// SPDX-License-Identifier: MIT
// Copyright (c) 2026 RyuYamamoto

#ifndef LIDAR_SLAM__OCCUPANCY_GRID_MAPPER_HPP_
#define LIDAR_SLAM__OCCUPANCY_GRID_MAPPER_HPP_

#include "keyframe.hpp"

#include <cstdint>
#include <vector>

namespace lidar_slam
{

struct MapParameters
{
  double resolution{0.05};
  bool auto_resize{true};
  int width{200};
  int height{200};
  double origin_x{-5.0};
  double origin_y{-5.0};
  double margin{2.0};
  // Probability thresholds for quantizing the log-odds grid to [0, 100] / -1.
  double occupied_threshold{0.65};
  double free_threshold{0.25};
  // Per-hit / per-miss log-odds increments (inverse sensor model).
  double hit_prob{0.7};
  double miss_prob{0.4};
  double log_odds_min{-2.0};
  double log_odds_max{3.5};
  int min_hits{2};  // min endpoints on a cell for it to be occupied (suppresses smears)
};

// ROS-independent occupancy grid: row-major data, -1 unknown / 0 free / 100 occupied.
struct GridMap
{
  double resolution{0.05};
  int width{0};
  int height{0};
  double origin_x{0.0};
  double origin_y{0.0};
  std::vector<int8_t> data;
};

// Builds a GridMap from optimized keyframes using log-odds ray casting.
// Re-rasterized from scratch each call, since keyframe poses move after optimization.
class OccupancyGridMapper
{
public:
  explicit OccupancyGridMapper(const MapParameters & parameters);

  GridMap build_map(const std::vector<Keyframe> & keyframes) const;

private:
  MapParameters parameters_;
};

}  // namespace lidar_slam

#endif  // LIDAR_SLAM__OCCUPANCY_GRID_MAPPER_HPP_
