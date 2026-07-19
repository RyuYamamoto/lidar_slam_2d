// SPDX-License-Identifier: MIT
// Copyright (c) 2026 RyuYamamoto

#ifndef LIDAR_SLAM__LIDAR_SLAM_NODE_HPP_
#define LIDAR_SLAM__LIDAR_SLAM_NODE_HPP_

#include "lidar_slam.hpp"

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace lidar_slam
{

// ROS adapter around LidarSlam: handles subscriptions, TF, publishing, and the
// conversion between ROS message types and the core's plain Eigen types.
class LidarSlamNode : public rclcpp::Node
{
public:
  explicit LidarSlamNode(const rclcpp::NodeOptions & node_options);

private:
  void callback_scan(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
  void callback_odom(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void publish_tf();
  void publish_map();
  void publish_visualization();

  std::vector<Eigen::Vector2d> scan_to_base_points(const sensor_msgs::msg::LaserScan & scan) const;
  std::optional<Eigen::Isometry2d> get_odom_pose(const rclcpp::Time & stamp);
  bool ensure_sensor_transform(const std::string & sensor_frame);

  // --- frames / topics / scan filtering ---
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string scan_topic_;
  std::string odom_topic_;
  bool use_odom_topic_{true};
  double min_range_;
  double max_range_;
  int point_skip_;

  // --- ROS interfaces ---
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr graph_publisher_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  rclcpp::TimerBase::SharedPtr map_timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::unique_ptr<LidarSlam> core_;

  // --- runtime state ---
  std::mutex mutex_;
  std::deque<std::pair<rclcpp::Time, Eigen::Isometry2d>> odom_buffer_;
  std::optional<Eigen::Isometry2d> sensor_transform_;  // base<-sensor
  std::optional<Eigen::Vector2d> sensor_origin_;       // base-frame sensor origin
  std::optional<Eigen::Isometry2d> latest_odom_pose_;
};

}  // namespace lidar_slam

#endif  // LIDAR_SLAM__LIDAR_SLAM_NODE_HPP_
