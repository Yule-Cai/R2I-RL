"""TB3-sim launch is provided by nav2_minimal_tb3_sim / turtlebot3 packages.

RuleSync sim launch: starts manager + bridge with parameters; the Nav2+sim
stack itself is launched separately (see demo script) so versions can vary.
VERIFY-LYRICAL: topic/remap names after install.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_description():
    return LaunchDescription([
        Node(package="rulesync_manager", executable="manager",
             name="rulesync_manager",
             parameters=[{"participants": ["nav2_bridge"],
                          "txn_log_path": "/tmp/rulesync_txn.log"}],
             output="screen"),
        Node(package="rulesync_nav2_bridge", executable="bridge",
             name="rulesync_nav2_bridge",
             parameters=[{"odom_topic": "/odom",
                          "goal_topic": "/goal_pose"}],
             output="screen"),
    ])


def generate_launch_description():
    return generate_description()
