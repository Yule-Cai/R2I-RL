#!/bin/bash
# Restart topo_inference (0.5s poll) on the live bench stack.
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export ROS_DISABLE_ROS2_DAEMON=1
export PYTHONDONTWRITEBYTECODE=1
set +u
source /opt/ros/lyrical/setup.bash
source ~/rulesync_ws/install/setup.bash
set -u
export PYTHONPATH=/home/yule/R2I-RL-checkout:/home/yule/R2I-RL-checkout/rulepatch:/home/yule/R2I-RL-checkout/topopatch_ros:${PYTHONPATH:-}
pkill -9 -f '[t]opo_inference' 2>/dev/null || true
sleep 2
cd ~/R2I-RL-checkout
setsid nohup python3 -c "from topopatch_ros.topo_inference_node import main; main()" \
  --ros-args -p weights_npz:=/home/yule/rulepatch/topo_scope/best_fused.npz \
  -p tau:=1.8216713190078737 -p use_scope:=True \
  > /tmp/persist_topo2.log 2>&1 < /dev/null &
sleep 8
tail -1 /tmp/persist_topo2.log
