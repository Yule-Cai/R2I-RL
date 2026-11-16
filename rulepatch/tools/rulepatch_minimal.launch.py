"""Minimal RulePatchNet bringup (absolute-path launch, no install needed).

Nodes: map_server, planner_server (headless Nav2 planning, manually
activated), rulepatch inference + pp_driver. Sim/spawn/bridges/RSP,
rulesync manager+bridge, and static TF are launched separately (see
demo_rulepatch.sh) so each piece can be verified independently.
Usage:
  ros2 launch <abs-path>/rulepatch_minimal.launch.py \
    --ros-args -p map_yaml:=... -p params_file:=... -p weights:=...
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_yaml = LaunchConfiguration("map_yaml")
    params_file = LaunchConfiguration("params_file")
    weights = LaunchConfiguration("weights")
    ws = "/home/yule/rulesync_ws/install/rulepatch_ros/bin"
    # NOTE: ExecuteProcess + absolute bin paths: ros2 run / Node executable
    # lookup misses ament_python console scripts installed under bin/.
    return LaunchDescription([
        DeclareLaunchArgument(
            "map_yaml",
            default_value="/opt/ros/lyrical/share/nav2_bringup/maps/tb3_sandbox.yaml"),
        DeclareLaunchArgument("params_file", default_value=""),
        DeclareLaunchArgument("weights", default_value=""),
        SetEnvironmentVariable(
            "PYTHONPATH",
            "/home/yule/R2I-RL-checkout/rulepatch:" + os.environ.get(
                "PYTHONPATH", "")),
        Node(package="nav2_map_server", executable="map_server",
             name="map_server", output="screen",
             parameters=[{"use_sim_time": True},
                         {"yaml_filename": map_yaml},
                         {"topic_name": "map"},
                         {"frame_id": "map"}]),
        Node(package="nav2_planner", executable="planner_server",
             name="planner_server", output="screen",
             parameters=[{"use_sim_time": True}, params_file]),
        ExecuteProcess(cmd=["python3", "-c", "from rulepatch_ros.inference_node import main; main()", "--ros-args", "-p", "weights_npz:=/home/yule/rulepatch/runs/m3/best_fused.npz"],
                       output="screen"),
        ExecuteProcess(cmd=["python3", "-c", "from rulepatch_ros.pp_driver import main; main()"], output="screen"),
    ])
