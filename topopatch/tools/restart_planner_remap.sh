#!/bin/bash
# Restart planner_server remapped for F1 (mask-delayed topic).
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export ROS_DISABLE_ROS2_DAEMON=1
export PYTHONDONTWRITEBYTECODE=1
set +u
source /opt/ros/lyrical/setup.bash
source ~/rulesync_ws/install/setup.bash
set -u
pkill -9 -f '[p]lanner_server' 2>/dev/null || true
sleep 2
cd ~/R2I-RL-checkout
setsid nohup ros2 run nav2_planner planner_server --ros-args \
  -p use_sim_time:=False --params-file /tmp/paper_planner_sb.yaml \
  -r /rulesync/keepout_mask:=/mask_delayed \
  > /tmp/persist_f1_planner.log 2>&1 < /dev/null &
sleep 8
timeout 90 python3 rulepatch/tools/activate_nav2.py --timeout 80 \
  --nodes planner_server 2>&1 | tail -1
