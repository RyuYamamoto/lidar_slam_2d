# SPDX-License-Identifier: MIT
# Copyright (c) 2026 RyuYamamoto

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    use_rviz = LaunchConfiguration("use_rviz", default="true")
    use_bag = LaunchConfiguration("use_bag", default="false")
    bag_path = LaunchConfiguration("bag_path", default="")
    bag_rate = LaunchConfiguration("bag_rate", default="1.0")

    slam_config_path = PathJoinSubstitution(
        [FindPackageShare("lidar_slam_2d"), "config", "lidar_slam_params.yaml"]
    )
    rviz_config = PathJoinSubstitution([FindPackageShare("lidar_slam_2d"), "rviz", "lidar_slam.rviz"])
    qos_override = PathJoinSubstitution(
        [FindPackageShare("lidar_slam_2d"), "config", "rosbag_qos_override.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_bag", default_value="false"),
            DeclareLaunchArgument("bag_path", default_value=""),
            DeclareLaunchArgument("bag_rate", default_value="1.0"),
            Node(
                package="lidar_slam_2d",
                executable="lidar_slam_node",
                name="lidar_slam",
                output="screen",
                parameters=[slam_config_path, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            # Optional rosbag replay. /tf_static needs a transient_local QoS override
            # so the static base<-sensor transform reaches the SLAM node.
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "play",
                    bag_path,
                    "--clock",
                    "--rate",
                    bag_rate,
                    "--qos-profile-overrides-path",
                    qos_override,
                ],
                condition=IfCondition(use_bag),
                output="screen",
            ),
        ]
    )
