// SPDX-License-Identifier: MIT
// Copyright (c) 2026 RyuYamamoto

#include "lidar_slam_node.hpp"

#include "se2.hpp"

#include <tf2/utils.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

namespace lidar_slam
{

namespace
{

Eigen::Isometry2d pose_to_se2(const geometry_msgs::msg::Pose & pose)
{
  return make_se2(pose.position.x, pose.position.y, tf2::getYaw(pose.orientation));
}

Eigen::Isometry2d transform_to_se2(const geometry_msgs::msg::Transform & transform)
{
  return make_se2(
    transform.translation.x, transform.translation.y, tf2::getYaw(transform.rotation));
}

geometry_msgs::msg::Transform se2_to_transform(const Eigen::Isometry2d & pose)
{
  geometry_msgs::msg::Transform transform;
  transform.translation.x = get_x(pose);
  transform.translation.y = get_y(pose);
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, get_yaw(pose));
  transform.rotation = tf2::toMsg(quaternion);
  return transform;
}

geometry_msgs::msg::Quaternion yaw_to_quaternion(const double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(quaternion);
}

nav_msgs::msg::OccupancyGrid grid_to_msg(const GridMap & grid)
{
  nav_msgs::msg::OccupancyGrid msg;
  msg.info.resolution = grid.resolution;
  msg.info.width = static_cast<uint32_t>(std::max(0, grid.width));
  msg.info.height = static_cast<uint32_t>(std::max(0, grid.height));
  msg.info.origin.position.x = grid.origin_x;
  msg.info.origin.position.y = grid.origin_y;
  msg.info.origin.orientation.w = 1.0;
  msg.data.assign(grid.data.begin(), grid.data.end());
  return msg;
}

}  // namespace

LidarSlamNode::LidarSlamNode(const rclcpp::NodeOptions & node_options)
: Node("lidar_slam", node_options)
{
  map_frame_ = this->declare_parameter<std::string>("frame.map_frame", "map");
  odom_frame_ = this->declare_parameter<std::string>("frame.odom_frame", "odom");
  base_frame_ = this->declare_parameter<std::string>("frame.base_frame", "base_footprint");

  scan_topic_ = this->declare_parameter<std::string>("topic.scan_topic", "scan");
  use_odom_topic_ = this->declare_parameter<bool>("topic.use_odom_topic", true);
  odom_topic_ = this->declare_parameter<std::string>("topic.odom_topic", "odom");

  min_range_ = this->declare_parameter<double>("scan.min_range", 0.1);
  max_range_ = this->declare_parameter<double>("scan.max_range", 30.0);
  point_skip_ = this->declare_parameter<int>("scan.point_skip", 1);

  CoreParameters core_parameters;
  core_parameters.trans_threshold =
    this->declare_parameter<double>("keyframe.trans_threshold", 0.5);
  core_parameters.rot_threshold = this->declare_parameter<double>("keyframe.rot_threshold", 0.3);
  core_parameters.submap_size =
    static_cast<std::size_t>(this->declare_parameter<int>("icp.submap_size", 5));
  core_parameters.prior_sigma_xy = this->declare_parameter<double>("graph.prior_sigma_xy", 0.01);
  core_parameters.prior_sigma_yaw = this->declare_parameter<double>("graph.prior_sigma_yaw", 0.01);
  core_parameters.between_sigma_xy =
    this->declare_parameter<double>("graph.between_sigma_xy", 0.05);
  core_parameters.between_sigma_yaw =
    this->declare_parameter<double>("graph.between_sigma_yaw", 0.03);
  core_parameters.odom_fallback_sigma_scale =
    this->declare_parameter<double>("graph.odom_fallback_sigma_scale", 5.0);
  core_parameters.use_icp_hessian_info =
    this->declare_parameter<bool>("graph.use_icp_hessian_info", true);
  core_parameters.odom_between_sigma_xy =
    this->declare_parameter<double>("graph.odom_between_sigma_xy", 0.1);
  core_parameters.odom_between_sigma_yaw =
    this->declare_parameter<double>("graph.odom_between_sigma_yaw", 0.05);
  core_parameters.loop_sigma_xy = this->declare_parameter<double>("loop.sigma_xy", 0.1);
  core_parameters.loop_sigma_yaw = this->declare_parameter<double>("loop.sigma_yaw", 0.05);
  core_parameters.loop_use_robust = this->declare_parameter<bool>("loop.use_robust", true);
  core_parameters.loop_huber_delta = this->declare_parameter<double>("loop.huber_delta", 0.1);
  core_parameters.loop_extra_update_iterations =
    this->declare_parameter<int>("loop.extra_update_iterations", 2);

  IcpParameters icp_parameters;
  icp_parameters.variant = IcpScanMatcher::variant_from_string(
    this->declare_parameter<std::string>("icp.variant", "point_to_line"));
  icp_parameters.max_iterations = this->declare_parameter<int>("icp.max_iterations", 30);
  icp_parameters.max_correspondence_dist =
    this->declare_parameter<double>("icp.max_correspondence_dist", 1.0);
  icp_parameters.convergence_eps = this->declare_parameter<double>("icp.convergence_eps", 1.0e-4);
  icp_parameters.fitness_threshold = this->declare_parameter<double>("icp.fitness_threshold", 0.5);
  icp_parameters.normal_neighbor_num = this->declare_parameter<int>("icp.normal_neighbor_num", 5);

  LoopParameters loop_parameters;
  loop_parameters.enable = this->declare_parameter<bool>("loop.enable", true);
  loop_parameters.search_radius = this->declare_parameter<double>("loop.search_radius", 3.0);
  loop_parameters.min_id_gap =
    static_cast<std::size_t>(this->declare_parameter<int>("loop.min_id_gap", 30));
  loop_parameters.submap_size =
    static_cast<std::size_t>(this->declare_parameter<int>("loop.submap_size", 5));
  loop_parameters.fitness_threshold =
    this->declare_parameter<double>("loop.fitness_threshold", 0.3);
  loop_parameters.max_candidates =
    static_cast<std::size_t>(this->declare_parameter<int>("loop.max_candidates", 1));

  IcpParameters loop_icp_parameters;
  loop_icp_parameters.variant = IcpVariant::PointToPoint;
  loop_icp_parameters.max_iterations = this->declare_parameter<int>("loop.icp_max_iterations", 40);
  loop_icp_parameters.max_correspondence_dist =
    this->declare_parameter<double>("loop.icp_max_correspondence_dist", 2.0);
  loop_icp_parameters.convergence_eps = icp_parameters.convergence_eps;
  loop_icp_parameters.fitness_threshold = loop_parameters.fitness_threshold;

  MapParameters map_parameters;
  map_parameters.resolution = this->declare_parameter<double>("map.resolution", 0.05);
  map_parameters.auto_resize = this->declare_parameter<bool>("map.auto_resize", true);
  map_parameters.width = this->declare_parameter<int>("map.width", 200);
  map_parameters.height = this->declare_parameter<int>("map.height", 200);
  map_parameters.origin_x = this->declare_parameter<double>("map.origin_x", -5.0);
  map_parameters.origin_y = this->declare_parameter<double>("map.origin_y", -5.0);
  map_parameters.margin = this->declare_parameter<double>("map.margin", 2.0);
  map_parameters.occupied_threshold =
    this->declare_parameter<double>("map.occupied_threshold", 0.65);
  map_parameters.free_threshold = this->declare_parameter<double>("map.free_threshold", 0.25);
  map_parameters.hit_prob = this->declare_parameter<double>("map.hit_prob", 0.7);
  map_parameters.miss_prob = this->declare_parameter<double>("map.miss_prob", 0.4);
  map_parameters.min_hits = this->declare_parameter<int>("map.min_hits", 2);

  const double map_publish_rate = this->declare_parameter<double>("map.publish_rate", 1.0);
  const double tf_publish_rate = this->declare_parameter<double>("tf.publish_rate", 30.0);

  core_ = std::make_unique<LidarSlam>(
    core_parameters, icp_parameters, loop_icp_parameters, loop_parameters, map_parameters);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, rclcpp::SensorDataQoS(),
    std::bind(&LidarSlamNode::callback_scan, this, std::placeholders::_1));
  if (use_odom_topic_) {
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(50),
      std::bind(&LidarSlamNode::callback_odom, this, std::placeholders::_1));
  }

  map_publisher_ =
    this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(1).transient_local());
  path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("keyframe_path", 1);
  graph_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("pose_graph", 1);

  tf_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int64_t>(1000.0 / tf_publish_rate)),
    std::bind(&LidarSlamNode::publish_tf, this));
  map_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int64_t>(1000.0 / map_publish_rate)),
    std::bind(&LidarSlamNode::publish_map, this));

  RCLCPP_INFO(get_logger(), "lidar_slam node started.");
}

void LidarSlamNode::callback_odom(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const Eigen::Isometry2d pose = pose_to_se2(msg->pose.pose);
  odom_buffer_.emplace_back(rclcpp::Time(msg->header.stamp), pose);
  latest_odom_pose_ = pose;
  while (odom_buffer_.size() > 500) odom_buffer_.pop_front();
}

std::optional<Eigen::Isometry2d> LidarSlamNode::get_odom_pose(const rclcpp::Time & stamp)
{
  if (use_odom_topic_) {
    if (odom_buffer_.empty()) return std::nullopt;
    if (stamp <= odom_buffer_.front().first) return odom_buffer_.front().second;
    if (stamp >= odom_buffer_.back().first) return odom_buffer_.back().second;
    for (std::size_t i = 0; i + 1 < odom_buffer_.size(); ++i) {
      const auto & prev = odom_buffer_[i];
      const auto & next = odom_buffer_[i + 1];
      if (prev.first <= stamp && stamp < next.first) {
        const double span = (next.first - prev.first).seconds();
        const double ratio = span > 1e-9 ? (stamp - prev.first).seconds() / span : 0.0;
        return interpolate_se2(prev.second, next.second, ratio);
      }
    }
    return odom_buffer_.back().second;
  }

  try {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform =
        tf_buffer_->lookupTransform(odom_frame_, base_frame_, stamp, tf2::durationFromSec(0.1));
    } catch (const tf2::ExtrapolationException &) {
      transform = tf_buffer_->lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);
    }
    const Eigen::Isometry2d pose = transform_to_se2(transform.transform);
    latest_odom_pose_ = pose;
    return pose;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "odom TF lookup failed: %s", ex.what());
    return std::nullopt;
  }
}

bool LidarSlamNode::ensure_sensor_transform(const std::string & sensor_frame)
{
  if (sensor_transform_.has_value()) return true;
  try {
    const auto transform = tf_buffer_->lookupTransform(
      base_frame_, sensor_frame, tf2::TimePointZero, tf2::durationFromSec(0.2));
    const Eigen::Isometry2d se2 = transform_to_se2(transform.transform);
    sensor_transform_ = se2;
    sensor_origin_ = se2.translation();
    RCLCPP_INFO(
      get_logger(), "resolved %s <- %s transform.", base_frame_.c_str(), sensor_frame.c_str());
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "sensor TF lookup failed: %s", ex.what());
    return false;
  }
}

std::vector<Eigen::Vector2d> LidarSlamNode::scan_to_base_points(
  const sensor_msgs::msg::LaserScan & scan) const
{
  std::vector<Eigen::Vector2d> points;
  const int skip = std::max(1, point_skip_);
  points.reserve(scan.ranges.size() / skip + 1);

  for (std::size_t i = 0; i < scan.ranges.size(); i += skip) {
    const float range = scan.ranges[i];
    if (!std::isfinite(range) || range < min_range_ || range > max_range_) continue;
    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    const Eigen::Vector2d point_sensor(range * std::cos(angle), range * std::sin(angle));
    points.emplace_back(sensor_transform_.value() * point_sensor);
  }
  return points;
}

void LidarSlamNode::callback_scan(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!ensure_sensor_transform(msg->header.frame_id)) return;

  const rclcpp::Time stamp(msg->header.stamp);
  const auto odom_pose = get_odom_pose(stamp);
  if (!odom_pose.has_value()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "no odometry available yet.");
    return;
  }

  std::vector<Eigen::Vector2d> scan_points = scan_to_base_points(*msg);
  if (scan_points.size() < 10) return;

  const ScanResult result = core_->add_scan(
    stamp.seconds(), std::move(scan_points), odom_pose.value(), sensor_origin_.value());

  if (result.icp_failed) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "ICP did not converge; using odometry fallback.");
  }
  for (const auto & loop : result.loops) {
    RCLCPP_INFO(
      get_logger(), "loop closure: %zu -> %zu (fitness=%.3f)", loop.from, loop.to, loop.fitness);
  }
}

void LidarSlamNode::publish_tf()
{
  std::lock_guard<std::mutex> lock(mutex_);

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = this->now();
  tf.header.frame_id = map_frame_;
  tf.child_frame_id = odom_frame_;

  if (!core_->has_keyframes() || !latest_odom_pose_.has_value()) {
    tf.transform = se2_to_transform(Eigen::Isometry2d::Identity());
  } else {
    tf.transform = se2_to_transform(core_->map_to_odom(latest_odom_pose_.value()));
  }
  tf_broadcaster_->sendTransform(tf);
}

void LidarSlamNode::publish_map()
{
  GridMap grid;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!core_->has_keyframes()) return;
    grid = core_->build_map();
  }

  nav_msgs::msg::OccupancyGrid msg = grid_to_msg(grid);
  msg.header.frame_id = map_frame_;
  msg.header.stamp = this->now();
  map_publisher_->publish(msg);

  publish_visualization();
}

void LidarSlamNode::publish_visualization()
{
  std::vector<Eigen::Isometry2d> poses;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    poses = core_->keyframe_poses();
  }
  if (poses.empty()) return;

  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = map_frame_;

  visualization_msgs::msg::Marker nodes;
  nodes.header = path.header;
  nodes.ns = "nodes";
  nodes.id = 0;
  nodes.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  nodes.action = visualization_msgs::msg::Marker::ADD;
  nodes.scale.x = nodes.scale.y = nodes.scale.z = 0.15;
  nodes.color.g = 1.0;
  nodes.color.a = 1.0;
  nodes.pose.orientation.w = 1.0;

  visualization_msgs::msg::Marker edges;
  edges.header = path.header;
  edges.ns = "edges";
  edges.id = 1;
  edges.type = visualization_msgs::msg::Marker::LINE_STRIP;
  edges.action = visualization_msgs::msg::Marker::ADD;
  edges.scale.x = 0.05;
  edges.color.b = 1.0;
  edges.color.a = 1.0;
  edges.pose.orientation.w = 1.0;

  for (const auto & pose : poses) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = path.header;
    pose_stamped.pose.position.x = get_x(pose);
    pose_stamped.pose.position.y = get_y(pose);
    pose_stamped.pose.orientation = yaw_to_quaternion(get_yaw(pose));
    path.poses.emplace_back(pose_stamped);

    geometry_msgs::msg::Point point;
    point.x = get_x(pose);
    point.y = get_y(pose);
    nodes.points.emplace_back(point);
    edges.points.emplace_back(point);
  }

  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.emplace_back(nodes);
  marker_array.markers.emplace_back(edges);

  path_publisher_->publish(path);
  graph_publisher_->publish(marker_array);
}

}  // namespace lidar_slam

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lidar_slam::LidarSlamNode)
