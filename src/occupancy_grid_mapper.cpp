// SPDX-License-Identifier: MIT
// Copyright (c) 2026 RyuYamamoto

#include "occupancy_grid_mapper.hpp"

#include "se2.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace lidar_slam
{

namespace
{

double prob_to_log_odds(const double prob)
{
  return std::log(prob / (1.0 - prob));
}

double log_odds_to_prob(const double log_odds)
{
  return 1.0 - 1.0 / (1.0 + std::exp(log_odds));
}

}  // namespace

OccupancyGridMapper::OccupancyGridMapper(const MapParameters & parameters) : parameters_(parameters)
{
}

GridMap OccupancyGridMapper::build_map(const std::vector<Keyframe> & keyframes) const
{
  GridMap grid;
  grid.resolution = parameters_.resolution;

  const double resolution = parameters_.resolution;

  double origin_x = parameters_.origin_x;
  double origin_y = parameters_.origin_y;
  int width = parameters_.width;
  int height = parameters_.height;

  if (parameters_.auto_resize) {
    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();
    bool has_point = false;

    for (const auto & keyframe : keyframes) {
      const Eigen::Vector2d sensor = keyframe.pose * keyframe.sensor_origin;
      min_x = std::min(min_x, sensor.x());
      min_y = std::min(min_y, sensor.y());
      max_x = std::max(max_x, sensor.x());
      max_y = std::max(max_y, sensor.y());
      for (const auto & point : keyframe.scan_points) {
        const Eigen::Vector2d world = keyframe.pose * point;
        min_x = std::min(min_x, world.x());
        min_y = std::min(min_y, world.y());
        max_x = std::max(max_x, world.x());
        max_y = std::max(max_y, world.y());
        has_point = true;
      }
    }

    if (!has_point) return grid;

    origin_x = min_x - parameters_.margin;
    origin_y = min_y - parameters_.margin;
    width = static_cast<int>(std::ceil((max_x + parameters_.margin - origin_x) / resolution));
    height = static_cast<int>(std::ceil((max_y + parameters_.margin - origin_y) / resolution));
  }

  width = std::max(1, width);
  height = std::max(1, height);

  grid.width = width;
  grid.height = height;
  grid.origin_x = origin_x;
  grid.origin_y = origin_y;

  const std::size_t cell_num = static_cast<std::size_t>(width) * height;
  std::vector<double> log_odds(cell_num, 0.0);
  std::vector<uint8_t> visited(cell_num, 0);
  std::vector<uint16_t> hit_count(cell_num, 0);

  const double l_hit = prob_to_log_odds(parameters_.hit_prob);
  const double l_miss = prob_to_log_odds(parameters_.miss_prob);
  const uint16_t min_hits = static_cast<uint16_t>(std::max(1, parameters_.min_hits));

  auto to_cell = [&](const Eigen::Vector2d & world, int & cx, int & cy) {
    cx = static_cast<int>(std::floor((world.x() - origin_x) / resolution));
    cy = static_cast<int>(std::floor((world.y() - origin_y) / resolution));
  };
  auto in_bounds = [&](const int cx, const int cy) {
    return cx >= 0 && cx < width && cy >= 0 && cy < height;
  };
  auto update_cell = [&](const int cx, const int cy, const double delta, const bool is_hit) {
    if (!in_bounds(cx, cy)) return;
    const std::size_t idx = static_cast<std::size_t>(cy) * width + cx;
    log_odds[idx] =
      std::clamp(log_odds[idx] + delta, parameters_.log_odds_min, parameters_.log_odds_max);
    visited[idx] = 1;
    if (is_hit && hit_count[idx] < std::numeric_limits<uint16_t>::max()) ++hit_count[idx];
  };

  for (const auto & keyframe : keyframes) {
    const Eigen::Vector2d sensor_world = keyframe.pose * keyframe.sensor_origin;
    int sx = 0;
    int sy = 0;
    to_cell(sensor_world, sx, sy);

    for (const auto & point : keyframe.scan_points) {
      const Eigen::Vector2d world = keyframe.pose * point;
      int ex = 0;
      int ey = 0;
      to_cell(world, ex, ey);

      // Bresenham from sensor cell to endpoint: free along the way, hit at the end.
      int x0 = sx;
      int y0 = sy;
      const int dx = std::abs(ex - x0);
      const int dy = std::abs(ey - y0);
      const int step_x = (x0 < ex) ? 1 : -1;
      const int step_y = (y0 < ey) ? 1 : -1;
      int error = dx - dy;

      while (x0 != ex || y0 != ey) {
        update_cell(x0, y0, l_miss, false);
        const int error2 = 2 * error;
        if (error2 > -dy) {
          error -= dy;
          x0 += step_x;
        }
        if (error2 < dx) {
          error += dx;
          y0 += step_y;
        }
      }
      update_cell(ex, ey, l_hit, true);
    }
  }

  grid.data.assign(cell_num, -1);
  for (std::size_t i = 0; i < cell_num; ++i) {
    if (!visited[i]) continue;
    const double prob = log_odds_to_prob(log_odds[i]);
    if (prob > parameters_.occupied_threshold && hit_count[i] >= min_hits) {
      grid.data[i] = 100;
    } else if (prob < parameters_.free_threshold) {
      grid.data[i] = 0;
    }
  }

  return grid;
}

}  // namespace lidar_slam
