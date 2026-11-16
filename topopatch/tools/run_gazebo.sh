#!/bin/bash
# TopoRulePatch Gazebo closed-loop (tb3_sandbox, default domain).
# Kills the idle headless stack first (CPU), then: TB3 sim+Nav2 ->
# manager+bridge -> topo_inference (fused v1, frozen tau) + pp_driver ->
# bag -> 3 scripted cases -> summary. Leaves nothing running on exit.
# Usage: bash run_gazebo.sh   (runs on onexplayer, needs display off/headless)
set -u
unset ROS_DOMAIN_ID
set +u
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
source ~/rulesync_ws/install/setup.bash
set -u
export PYTHONPATH=/home/yule/R2I-RL-checkout:/home/yule/R2I-RL-checkout/rulepatch:/home/yule/R2I-RL-checkout/topopatch_ros:${PYTHONPATH:-}
CK=~/R2I-RL-checkout
LOG=~/rulepatch/logs
TAU=1.0264495134353637
WTS=~/rulepatch/topo_full/best_fused.npz

echo "[gz] killing idle headless stack..."
pkill -9 -f 'nav2_map_server/map_server' 2>/dev/null || true
pkill -9 -f 'nav2_planner/planner_server' 2>/dev/null || true
pkill -9 -f 'nav2_lifecycle_manager/lifecycle_manager' 2>/dev/null || true
pkill -9 -f 'tf2_ros.*static_transform_publisher' 2>/dev/null || true
pkill -9 -f '[g]en_topo_dataset' 2>/dev/null || true
sleep 3

echo "[gz] starting TB3 sim + Nav2..."
bash -c 'ros2 launch rulesync_demo rulesync_tb3_sim.launch.py headless:=True use_rviz:=False slam:=False params_file:=$HOME/rulesync_ws/nav2_params_rulesync.yaml' > $LOG/gz_sim.log 2>&1 &
SIM_PID=$!
# Full-stack killer: ros2 launch children (gz, nav2 containers, bridges)
# survive a plain wrapper kill and block the next run ("world already
# running"). Kill by pattern on both entry and exit.
kill_stack() {
  pkill -9 -f 'gz sim' 2>/dev/null || true
  pkill -9 -f 'component_container' 2>/dev/null || true
  pkill -9 -f 'parameter_bridge' 2>/dev/null || true
  pkill -9 -f 'robot_state_publisher' 2>/dev/null || true
  pkill -9 -f 'rulesync_tb3_sim' 2>/dev/null || true
  pkill -9 -f 'topo_inference|rulepatch_ros|topo_case|rulesync_demo' \
    2>/dev/null || true
  kill $SIM_PID $(jobs -p) 2>/dev/null || true
}
trap kill_stack EXIT
kill_stack
sleep 2
echo "[gz] waiting for navigate_to_pose (up to 5 min)..."
for i in $(seq 1 60); do
  if ros2 action list 2>/dev/null | grep -q navigate_to_pose; then break; fi
  sleep 5
done
ros2 action list 2>/dev/null | grep navigate_to_pose || { echo "[gz] FATAL: no nav action"; exit 1; }

echo "[gz] starting manager + bridge..."
ros2 launch rulesync_demo rulesync_sim.launch.py > $LOG/gz_nodes.log 2>&1 &
sleep 8

echo "[gz] starting topo_inference + pp_driver..."
python3 -c "from topopatch_ros.topo_inference_node import main; main()" \
  --ros-args -p weights_npz:=$WTS -p tau:=$TAU > $LOG/gz_topo.log 2>&1 &
python3 -c "from rulepatch_ros.pp_driver import main; main()" \
  > $LOG/gz_pp.log 2>&1 &
sleep 5

echo "[gz] recording bag..."
rm -rf ~/rulepatch/gz_bag
ros2 bag record -o ~/rulepatch/gz_bag /cmd_vel /odom /plan \
  /rulepatch/status /rulepatch/repaired_path \
  /rulesync/staged_set > $LOG/gz_bag.log 2>&1 &
BAG_PID=$!
sleep 2

echo "[gz] running cases..."
python3 -m topopatch_ros.topo_case_driver --csv ~/rulepatch/gz_cases.csv \
  > $LOG/gz_cases.log 2>&1
echo "[gz] cases exit: $?"

kill $BAG_PID 2>/dev/null || true
sleep 2
echo "[gz] summary:"; cat ~/rulepatch/gz_cases.csv 2>/dev/null | head -8
trap - EXIT
kill_stack
echo "[gz] done."
