#!/bin/bash
# RuleSync Milestone-1 simulation demo driver (except GUI interaction).
# Prerequisites (run once): Nav2 + TB3 sim installed, workspace built + sourced.
# Usage: bash scripts/demo_runtime_rules.sh [--goal-x 4.0 --goal-y 1.0]
set -euo pipefail
WS=~/rulesync_ws
GOAL_X=4.0
GOAL_Y=1.0
for arg in "$@"; do
  case "$arg" in
    --goal-x=*) GOAL_X="${arg#*=}" ;;
    --goal-y=*) GOAL_Y="${arg#*=}" ;;
  esac
done
source /opt/ros/lyrical/setup.bash
source "$WS/install/setup.bash"

echo "[demo] starting TB3 sim + Nav2 (background, logs to /tmp/rulesync_sim.log)..."
# VERIFY-LYRICAL: exact bringup launch file/package names after install.
bash -c 'ros2 launch rulesync_demo rulesync_tb3_sim.launch.py headless:=True use_rviz:=False slam:=False params_file:=$HOME/rulesync_ws/nav2_params_rulesync.yaml' \
  > /tmp/rulesync_sim.log 2>&1 &
SIM_PID=$!
trap "kill $SIM_PID 2>/dev/null || true" EXIT
echo "[demo] waiting for Nav2 action server..."
for i in $(seq 1 60); do
  if ros2 action list 2>/dev/null | grep -q navigate_to_pose; then break; fi
  sleep 5
done
ros2 action list | grep navigate_to_pose

echo "[demo] starting RuleSync nodes..."
ros2 launch rulesync_demo rulesync_sim.launch.py > /tmp/rulesync_nodes.log 2>&1 &
NODES_PID=$!
trap "kill $SIM_PID $NODES_PID 2>/dev/null || true" EXIT
sleep 5

echo "[demo] recording rosbag2..."
rm -rf /tmp/rulesync_demo_bag
ros2 bag record -o /tmp/rulesync_demo_bag /cmd_vel /odom /scan /plan \
  /rulesync/keepout_mask /rulesync/rule_markers > /tmp/rulesync_bag.log 2>&1 &
BAG_PID=$!
trap "kill $SIM_PID $NODES_PID $BAG_PID 2>/dev/null || true" EXIT
sleep 2

echo "[demo] running rule-injection client..."
python3 "$WS/src/rulesync_demo/rulesync_demo/demo_client.py" \
  --goal-x "$GOAL_X" --goal-y "$GOAL_Y" --csv /tmp/rulesync_latency.csv

kill "$BAG_PID" 2>/dev/null || true
echo "[demo] done. bag: /tmp/rulesync_demo_bag  csv: /tmp/rulesync_latency.csv"
echo "[demo] sim/nav2 logs: /tmp/rulesync_sim.log  nodes: /tmp/rulesync_nodes.log"
