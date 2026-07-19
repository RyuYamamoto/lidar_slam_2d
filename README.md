# lidar_slam_2d

2D LiDAR graph-based SLAM (loosely coupled) for ROS 2, with a ROS-independent core.

- Front-end: self-implemented ICP (point-to-point / point-to-line) with a grid-based
  nearest neighbor search.
- Back-end: GTSAM iSAM2 pose graph (odometry + ICP `BetweenFactor`s), degeneracy-aware
  anisotropic information from the ICP Hessian, and loop closure with a Huber kernel.
- Mapping: log-odds occupancy grid rebuilt from the optimized keyframes.

Only Eigen and GTSAM are required by the core; the ROS node is a thin adapter.

## Build

```bash
# in a colcon workspace (ROS 2 Humble/Jazzy), with GTSAM (libgtsam-dev / ros-<distro>-gtsam)
colcon build --packages-select lidar_slam_2d
```

## Run

```bash
# node + rviz, and optionally replay a bag (with the required /tf_static QoS override)
ros2 launch lidar_slam_2d lidar_slam_bringup.launch.py \
  use_bag:=true bag_path:=/path/to/bag bag_rate:=1.0
```

Publishes `map`->`odom` TF and `/map` (`nav_msgs/OccupancyGrid`). Topic names, frame names
and thresholds are parameters; see `config/lidar_slam_params.yaml`.
