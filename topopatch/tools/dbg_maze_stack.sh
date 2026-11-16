#!/bin/bash
# Verified-clean maze headless stack + single manual plan (debug).
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export ROS_DISABLE_ROS2_DAEMON=1
export PYTHONDONTWRITEBYTECODE=1
set +u
source /opt/ros/lyrical/setup.bash
set -u
export PYTHONPATH=/home/yule/R2I-RL-checkout:${PYTHONPATH:-}
cd ~/R2I-RL-checkout
setsid nohup ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom \
  --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 > /tmp/dbg2_tf.log 2>&1 < /dev/null &
setsid nohup ros2 run tf2_ros static_transform_publisher --frame-id odom --child-frame-id base_link \
  --x -5.9 --y -5.8 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 >> /tmp/dbg2_tf.log 2>&1 < /dev/null &
setsid nohup ros2 run nav2_map_server map_server --ros-args \
  --params-file /tmp/paper_planner_topo_ext_maze-B-s202.yaml > /tmp/dbg2_map.log 2>&1 < /dev/null &
setsid nohup ros2 run nav2_planner planner_server --ros-args \
  --params-file /tmp/paper_planner_topo_ext_maze-B-s202.yaml > /tmp/dbg2_planner.log 2>&1 < /dev/null &
sleep 8
timeout 100 python3 rulepatch/tools/activate_nav2.py --timeout 90 \
  --nodes map_server,planner_server 2>&1 | tail -1
echo STACK_UP
