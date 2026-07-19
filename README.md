# lidar_slam_2d

2D LiDAR graph-based SLAM (loosely coupled) for ROS 2, with a ROS-independent core.

- Front-end: self-implemented ICP (point-to-point / point-to-line) with a grid-based
  nearest neighbor search.
- Back-end: GTSAM iSAM2 pose graph (odometry + ICP `BetweenFactor`s), degeneracy-aware
  anisotropic information from the ICP Hessian, and loop closure with a Huber kernel.
- Mapping: log-odds occupancy grid rebuilt from the optimized keyframes.

Only Eigen and GTSAM are required by the core; the ROS node is a thin adapter.

## Layout

```
src/
  lidar_slam.{hpp,cpp}         # NavyuSlam-free core pipeline (LidarSlam), ROS-independent
  lidar_slam_node.{hpp,cpp}    # ROS node (LidarSlamNode): subscriptions, TF, type adaptation
  icp_scan_matcher.*           # ICP front-end
  pose_graph.*                 # GTSAM iSAM2 wrapper
  keyframe_manager.*           # keyframe container / submap construction
  loop_detector.*              # loop closure candidate search + verification
  occupancy_grid_mapper.*      # log-odds occupancy grid (returns a plain GridMap)
  keyframe.hpp / se2.hpp / scan_matcher_base.hpp
config/  launch/  rviz/
```

Headers live next to the sources under `src/`.

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

## License

MIT
