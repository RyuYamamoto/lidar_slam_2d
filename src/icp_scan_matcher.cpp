// SPDX-License-Identifier: MIT
// Copyright (c) 2026 RyuYamamoto

#include "icp_scan_matcher.hpp"

#include "se2.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <limits>

namespace lidar_slam
{

IcpScanMatcher::GridIndex::GridIndex(
  const std::vector<Eigen::Vector2d> & points, const double cell_size)
: points_(points), cell_size_(cell_size)
{
  for (int64_t i = 0; i < static_cast<int64_t>(points_.size()); ++i) {
    const int cx = static_cast<int>(std::floor(points_[i].x() / cell_size_));
    const int cy = static_cast<int>(std::floor(points_[i].y() / cell_size_));
    cells_[hash(cx, cy)].emplace_back(i);
  }
}

int64_t IcpScanMatcher::GridIndex::hash(const int cx, const int cy) const
{
  // Pack two 32-bit cell coordinates into a single 64-bit key.
  return (static_cast<int64_t>(cx) << 32) ^ (static_cast<int64_t>(cy) & 0xffffffffLL);
}

int64_t IcpScanMatcher::GridIndex::nearest(
  const Eigen::Vector2d & query, const double max_dist) const
{
  const int cx = static_cast<int>(std::floor(query.x() / cell_size_));
  const int cy = static_cast<int>(std::floor(query.y() / cell_size_));
  const int radius = std::max(1, static_cast<int>(std::ceil(max_dist / cell_size_)));

  int64_t best_index = -1;
  double best_dist_sq = max_dist * max_dist;

  for (int dy = -radius; dy <= radius; ++dy) {
    for (int dx = -radius; dx <= radius; ++dx) {
      const auto it = cells_.find(hash(cx + dx, cy + dy));
      if (it == cells_.end()) continue;
      for (const int64_t index : it->second) {
        const double dist_sq = (points_[index] - query).squaredNorm();
        if (dist_sq < best_dist_sq) {
          best_dist_sq = dist_sq;
          best_index = index;
        }
      }
    }
  }
  return best_index;
}

IcpScanMatcher::IcpScanMatcher(const IcpParameters & parameters) : parameters_(parameters)
{
}

IcpVariant IcpScanMatcher::variant_from_string(const std::string & name)
{
  if (name == "point_to_line") return IcpVariant::PointToLine;
  return IcpVariant::PointToPoint;
}

Eigen::Vector2d IcpScanMatcher::estimate_normal(
  const std::vector<Eigen::Vector2d> & target, const Eigen::Vector2d & point) const
{
  // Collect nearby target points and take the smallest-eigenvector of their
  // covariance as the local surface normal.
  const double search_radius = parameters_.max_correspondence_dist;

  std::vector<Eigen::Vector2d> neighbors;
  for (const auto & candidate : target) {
    if ((candidate - point).squaredNorm() <= search_radius * search_radius) {
      neighbors.emplace_back(candidate);
      if (static_cast<int>(neighbors.size()) >= 3 * parameters_.normal_neighbor_num) break;
    }
  }

  if (neighbors.size() < 2) return Eigen::Vector2d::Zero();

  Eigen::Vector2d mean = Eigen::Vector2d::Zero();
  for (const auto & n : neighbors) mean += n;
  mean /= static_cast<double>(neighbors.size());

  Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
  for (const auto & n : neighbors) {
    const Eigen::Vector2d diff = n - mean;
    covariance += diff * diff.transpose();
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
  if (solver.info() != Eigen::Success) return Eigen::Vector2d::Zero();
  // Eigenvalues are sorted ascending; the normal corresponds to the smallest one.
  Eigen::Vector2d normal = solver.eigenvectors().col(0);
  const double norm = normal.norm();
  if (norm < 1e-9) return Eigen::Vector2d::Zero();
  return normal / norm;
}

MatchResult IcpScanMatcher::align(
  const std::vector<Eigen::Vector2d> & source, const std::vector<Eigen::Vector2d> & target,
  const Eigen::Isometry2d & init_guess)
{
  MatchResult result;
  result.relative_pose = init_guess;

  if (source.empty() || target.size() < 2) {
    result.converged = false;
    return result;
  }

  const GridIndex index(target, std::max(0.05, parameters_.max_correspondence_dist));
  const double gate = parameters_.max_correspondence_dist;
  const double gate_sq = gate * gate;

  Eigen::Isometry2d transform = init_guess;
  double last_mean_error = std::numeric_limits<double>::max();
  int inlier_count = 0;

  for (int iteration = 0; iteration < parameters_.max_iterations; ++iteration) {
    std::vector<Eigen::Vector2d> src_matched;
    std::vector<Eigen::Vector2d> tgt_matched;
    std::vector<Eigen::Vector2d> tgt_normals;
    src_matched.reserve(source.size());
    tgt_matched.reserve(source.size());

    double error_sum = 0.0;
    for (const auto & point : source) {
      const Eigen::Vector2d transformed = transform * point;
      const int64_t nn = index.nearest(transformed, gate);
      if (nn < 0) continue;
      const Eigen::Vector2d & target_point = target[nn];
      if ((target_point - transformed).squaredNorm() > gate_sq) continue;

      src_matched.emplace_back(point);
      tgt_matched.emplace_back(target_point);
      if (parameters_.variant == IcpVariant::PointToLine) {
        tgt_normals.emplace_back(estimate_normal(target, target_point));
      }
      error_sum += (target_point - transformed).norm();
    }

    inlier_count = static_cast<int>(src_matched.size());
    if (inlier_count < 3) {
      result.converged = false;
      result.fitness = std::numeric_limits<double>::max();
      result.relative_pose = transform;
      return result;
    }

    const double mean_error = error_sum / static_cast<double>(inlier_count);

    Eigen::Isometry2d delta = Eigen::Isometry2d::Identity();
    if (parameters_.variant == IcpVariant::PointToLine) {
      // Gauss-Newton step on the (dx, dy, dtheta) increment.
      Eigen::Matrix3d hessian = Eigen::Matrix3d::Zero();
      Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
      for (std::size_t i = 0; i < src_matched.size(); ++i) {
        const Eigen::Vector2d normal = tgt_normals[i];
        if (normal.squaredNorm() < 1e-12) continue;
        const Eigen::Vector2d p = transform * src_matched[i];
        const Eigen::Vector2d q = tgt_matched[i];
        const double residual = normal.dot(p - q);
        // Jacobian of the residual w.r.t. (dx, dy, dtheta) at the current estimate.
        Eigen::Vector3d jacobian;
        jacobian(0) = normal.x();
        jacobian(1) = normal.y();
        jacobian(2) = normal.dot(Eigen::Vector2d(-p.y(), p.x()));
        hessian += jacobian * jacobian.transpose();
        gradient += jacobian * residual;
      }
      hessian += 1e-9 * Eigen::Matrix3d::Identity();
      const Eigen::Vector3d increment = -hessian.ldlt().solve(gradient);
      delta = make_se2(increment(0), increment(1), increment(2));
    } else {
      // point-to-point: closed form (Umeyama) rotation + translation.
      Eigen::Vector2d src_mean = Eigen::Vector2d::Zero();
      Eigen::Vector2d tgt_mean = Eigen::Vector2d::Zero();
      for (std::size_t i = 0; i < src_matched.size(); ++i) {
        src_mean += transform * src_matched[i];
        tgt_mean += tgt_matched[i];
      }
      src_mean /= static_cast<double>(src_matched.size());
      tgt_mean /= static_cast<double>(tgt_matched.size());

      Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
      for (std::size_t i = 0; i < src_matched.size(); ++i) {
        const Eigen::Vector2d s = (transform * src_matched[i]) - src_mean;
        const Eigen::Vector2d t = tgt_matched[i] - tgt_mean;
        covariance += t * s.transpose();
      }

      Eigen::JacobiSVD<Eigen::Matrix2d> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
      Eigen::Matrix2d rotation = svd.matrixU() * svd.matrixV().transpose();
      if (rotation.determinant() < 0.0) {
        Eigen::Matrix2d correction = Eigen::Matrix2d::Identity();
        correction(1, 1) = -1.0;
        rotation = svd.matrixU() * correction * svd.matrixV().transpose();
      }
      const Eigen::Vector2d translation = tgt_mean - rotation * src_mean;
      delta.linear() = rotation;
      delta.translation() = translation;
    }

    transform = delta * transform;

    if (std::abs(last_mean_error - mean_error) < parameters_.convergence_eps) {
      last_mean_error = mean_error;
      result.converged = true;
      break;
    }
    last_mean_error = mean_error;
    if (iteration + 1 == parameters_.max_iterations) result.converged = true;
  }

  result.relative_pose = transform;
  result.fitness = last_mean_error;
  if (result.fitness > parameters_.fitness_threshold) result.converged = false;

  // Hessian at convergence; rank-deficient along degenerate directions.
  result.information = compute_hessian(source, target, transform);

  return result;
}

Eigen::Matrix3d IcpScanMatcher::compute_hessian(
  const std::vector<Eigen::Vector2d> & source, const std::vector<Eigen::Vector2d> & target,
  const Eigen::Isometry2d & transform) const
{
  const GridIndex index(target, std::max(0.05, parameters_.max_correspondence_dist));
  const double gate_sq = parameters_.max_correspondence_dist * parameters_.max_correspondence_dist;

  Eigen::Matrix3d hessian = Eigen::Matrix3d::Zero();
  for (const auto & point : source) {
    const Eigen::Vector2d p = transform * point;
    const int64_t nn = index.nearest(p, parameters_.max_correspondence_dist);
    if (nn < 0) continue;
    const Eigen::Vector2d & q = target[nn];
    if ((q - p).squaredNorm() > gate_sq) continue;

    if (parameters_.variant == IcpVariant::PointToLine) {
      const Eigen::Vector2d normal = estimate_normal(target, q);
      if (normal.squaredNorm() < 1e-12) continue;
      // Residual r = n^T (p - q); J = [n_x, n_y, n^T (-p_y, p_x)].
      Eigen::Vector3d jacobian;
      jacobian(0) = normal.x();
      jacobian(1) = normal.y();
      jacobian(2) = normal.dot(Eigen::Vector2d(-p.y(), p.x()));
      hessian += jacobian * jacobian.transpose();
    } else {
      // point-to-point residual r = p - q (2D); J = [[1,0,-p_y],[0,1,p_x]].
      Eigen::Matrix<double, 2, 3> jacobian;
      jacobian << 1.0, 0.0, -p.y(), 0.0, 1.0, p.x();
      hessian += jacobian.transpose() * jacobian;
    }
  }
  return hessian;
}

}  // namespace lidar_slam
