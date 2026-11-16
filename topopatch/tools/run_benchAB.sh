#!/bin/bash
# A/B transaction benchmark runner (onexplayer side).
export PYTHONDONTWRITEBYTECODE=1
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export ROS_DISABLE_ROS2_DAEMON=1
set +u
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
source ~/rulesync_ws/install/setup.bash
set -u
export PYTHONPATH=/home/yule/R2I-RL-checkout:/home/yule/R2I-RL-checkout/rulepatch:/home/yule/R2I-RL-checkout/topopatch_ros:${PYTHONPATH:-}
export AMENT_PREFIX_PATH=/opt/ros/lyrical:/home/yule/rulesync_ws/install
export ROS_DISTRO=lyrical
cd ~/R2I-RL-checkout
python3 -m topopatch_ros.bench_txn --csv ~/rulepatch/paper/benchtxn_persistent.csv --mode persistent --warmup 20 --n 200 --epoch0 2001
python3 -m topopatch_ros.bench_txn --csv ~/rulepatch/paper/benchtxn_legacy.csv --mode legacy --warmup 20 --n 200 --epoch0 5001
echo BENCH_AB_DONE
