cmake_minimum_required(VERSION 3.5)
project(lidar_slam_2d)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# GTSAM / Eigen are not resolved through ament, so find them explicitly.
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)

# Core SLAM library: ROS-independent, depends only on Eigen and GTSAM. It can be
# built and reused outside a ROS environment. Headers live alongside the sources.
add_library(lidar_slam SHARED
  src/lidar_slam.cpp
  src/icp_scan_matcher.cpp
  src/pose_graph.cpp
  src/keyframe_manager.cpp
  src/loop_detector.cpp
  src/occupancy_grid_mapper.cpp
)
target_include_directories(lidar_slam PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(lidar_slam PUBLIC Eigen3::Eigen gtsam)

# ROS node: thin adapter that wraps the core library.
ament_auto_add_library(lidar_slam_node_component SHARED
  src/lidar_slam_node.cpp
)
target_include_directories(lidar_slam_node_component PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(lidar_slam_node_component lidar_slam gtsam)

rclcpp_components_register_node(lidar_slam_node_component
  PLUGIN "lidar_slam::LidarSlamNode"
  EXECUTABLE lidar_slam_node
)

install(
  TARGETS lidar_slam
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
  rviz
)
