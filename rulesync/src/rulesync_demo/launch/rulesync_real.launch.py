"""Real-robot launch: same RuleSync nodes, parameterized topics/frames.

The user launches this when the ROS car is available. No motion is commanded
by this file; it only starts manager + bridge. Motion requires the separate
demo client with explicit user approval.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("odom_topic", default_value="/odom"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("nav_namespace", default_value="/"),
        Node(package="rulesync_manager", executable="manager",
             name="rulesync_manager", output="screen"),
        Node(package="rulesync_nav2_bridge", executable="bridge",
             name="rulesync_nav2_bridge",
             parameters=[{"map_frame": LaunchConfiguration("map_frame"),
                          "odom_topic": LaunchConfiguration("odom_topic")}],
             output="screen"),
    ])
