#!/bin/bash
# Start uav_bridge for the smoke world (boxes file + spawn args).
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export ROS_DISABLE_ROS2_DAEMON=1
export PYTHONPATH=/home/yule/R2I-RL-checkout:/home/yule/R2I-RL-checkout/topopatch_ros:${PYTHONPATH:-}
export PYTHONDONTWRITEBYTECODE=1
set +u
source /opt/ros/lyrical/setup.bash
source /home/yule/rulesync_ws/install/setup.bash
set -u
BOXES_FILE=${1:-/tmp/boxes_smoke.json}
SPAWN=${2:-[1.0,10.0,1.0]}
LOG=${3:-/home/yule/rulepatch/3d/logs/ubridge_smoke2.log}
# Singleton: a stale bridge from another world answers the SAME vote
# service with the WRONG occupancy (verified: barrier-boxes bridge
# rejected open-world commits as "in collision"). Kill first.
for pat in '[u]av_bridge'; do pkill -9 -f "$pat" 2>/dev/null || true; done
sleep 3
BOXES=$(cat $BOXES_FILE)
SX=$(echo $SPAWN | tr -d '[]' | cut -d, -f1)
SY=$(echo $SPAWN | tr -d '[]' | cut -d, -f2)
SZ=$(echo $SPAWN | tr -d '[]' | cut -d, -f3)
setsid nohup python3 -m topopatch_ros.uav_bridge_node --ros-args \
  -p world_boxes:="'$BOXES'" -p spawn_xyz:="[$SX,$SY,$SZ]" \
  > $LOG 2>&1 < /dev/null &
sleep 8
tail -2 $LOG
