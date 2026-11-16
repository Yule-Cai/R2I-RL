"""Standalone fail-closed RuleGuard launch.

The controller must consume executable_topic.  The configured pose_topic must
publish actual localized PoseStamped samples in rule_frame.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("rule_frame", default_value="map"),
        DeclareLaunchArgument("pose_topic", default_value="/ruleguard/pose"),
        DeclareLaunchArgument(
            "authorization_topic",
            default_value="/rulesync/execution_authorization"),
        DeclareLaunchArgument(
            "executable_topic",
            default_value="/rulepatch/executable_path"),
        Node(
            package="rulesync_guard",
            executable="ruleguard",
            name="rulesync_guard",
            parameters=[{
                "rule_frame": LaunchConfiguration("rule_frame"),
                "pose_topic": LaunchConfiguration("pose_topic"),
                "authorization_topic": LaunchConfiguration(
                    "authorization_topic"),
                "executable_topic": LaunchConfiguration(
                    "executable_topic"),
            }],
            output="screen",
        ),
    ])
