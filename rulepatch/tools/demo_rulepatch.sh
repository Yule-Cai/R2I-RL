#!/bin/bash
# RulePatchNet M2 Gazebo closed-loop demo (onexplayer, domain 0).
# Minimal deterministic stack (default): gz + spawn + bridges + map/amcl/
# headless-planner (manually activated, NO lifecycle manager, NO BT) +
# RuleSync + inference + pp_driver. Full Nav2 stack available via --stack full.
# Usage: bash demo_rulepatch.sh [--mode accept|fallback|rollback] [--stack minimal|full]
set -eo pipefail
# (no -u: ROS setup scripts reference unset AMENT_* vars under nounset)
WS=~/rulesync_ws
MODE=accept
STACK=minimal
GOAL_X=2.23
GOAL_Y=-0.88
WEIGHTS=~/rulepatch/runs/m3/best_fused.npz
for arg in "$@"; do
  case "$arg" in
    --mode=*) MODE="${arg#*=}" ;;
    --stack=*) STACK="${arg#*=}" ;;
    --goal-x=*) GOAL_X="${arg#*=}" ;;
    --goal-y=*) GOAL_Y="${arg#*=}" ;;
    --weights=*) WEIGHTS="${arg#*=}" ;;
  esac
done
source /opt/ros/lyrical/setup.bash
source "$WS/install/setup.bash"
export PYTHONPATH=/home/yule/R2I-RL-checkout/rulepatch:${PYTHONPATH:-}
# Deterministic discovery: WiFi multicast is lossy on this box; use a local
# FastDDS discovery server (unicast loopback) for ALL ROS processes.
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
# Bypass the ros2 daemon: it serves stale graphs after bringup cycles.
export ROS_DISABLE_ROS2_DAEMON=1
test -f "$WEIGHTS" || { echo "[demo] missing weights: $WEIGHTS"; exit 1; }

echo "[demo] pre-flight: kill stale sim/nav/rulesync/rulepatch + SHM..."
for pat in '[g]z-sim-main' '[n]av2_container' '[p]lanner_server' '[m]ap_server' \
           '[a]mcl' '[r]ulesync_manager/manager' '[r]ulesync_nav2_bridge/bridge' \
           '[b]in/inference' '[p]p_driver' '[r]os2 launch' '[x]vfb-run' '[X]vfb :' \
           '[r]obot_state_publisher' '[p]arameter_bridge' '[r]os2 bag record' \
           '[d]emo_rulepatch_client'; do
  pkill -f "$pat" 2>/dev/null || true
done
sleep 5
rm -f /dev/shm/fastdds_* /dev/shm/sem.fastdds_* 2>/dev/null || true

SIM_DIR=/opt/ros/lyrical/share/nav2_minimal_tb3_sim
BRINGUP_DIR=/opt/ros/lyrical/share/nav2_bringup
export GZ_SIM_RESOURCE_PATH=$SIM_DIR/models:${GZ_SIM_RESOURCE_PATH:-}

cleanup_pids=""
trap 'kill $cleanup_pids 2>/dev/null || true' EXIT
add_pid() { cleanup_pids="$cleanup_pids $1"; }

if [ "$STACK" = "minimal" ]; then
  echo "[demo] MINIMAL stack: gz + spawn + bridges, then map/amcl/planner (no manager, no BT)..."
  WORLD_SDF=/tmp/rulepatch_world.sdf
  xacro -o "$WORLD_SDF" headless:=True "$SIM_DIR/worlds/tb3_sandbox.sdf.xacro"
  xvfb-run -a gz sim -r -s "$WORLD_SDF" > /tmp/rulepatch_sim.log 2>&1 &
  SIM_PID=$!; add_pid $SIM_PID
  sleep 10
  echo "[demo] spawn TB3 + robot_state_publisher..."
  ros2 launch nav2_minimal_tb3_sim spawn_tb3.launch.py \
    robot_name:=turtlebot3_waffle robot_sdf:="$SIM_DIR/urdf/gz_waffle.sdf.xacro" \
    x_pose:=-2.0 y_pose:=-0.5 z_pose:=0.01 > /tmp/rulepatch_spawn.log 2>&1 &
  add_pid $!
  ros2 run robot_state_publisher robot_state_publisher --ros-args \
    -p use_sim_time:=True \
    -p robot_description:="$(cat $SIM_DIR/urdf/turtlebot3_waffle.urdf)" \
    > /tmp/rulepatch_rsp.log 2>&1 &
  add_pid $!
  echo "[demo] waiting for sim odom..."
  for i in $(seq 1 24); do
    if timeout 20 python3 ~/R2I-RL-checkout/rulepatch/tools/wait_topic.py /odom --timeout 15 2>/dev/null | grep -q flowing; then break; fi
    sleep 5
  done
  sleep 5
  echo "[demo] map_server + planner_server (standalone; static map->odom, no AMCL)..."
  MAP_YAML=$BRINGUP_DIR/maps/tb3_sandbox.yaml
  PARAMS=$HOME/rulesync_ws/nav2_params_rulesync.yaml
  cat > /tmp/rulepatch_map_params.yaml <<EOF
/**:
  ros__parameters:
    yaml_filename: "$MAP_YAML"
    topic_name: "map"
    frame_id: "map"
EOF
  ros2 run nav2_map_server map_server --ros-args -p use_sim_time:=True \
    --params-file /tmp/rulepatch_map_params.yaml \
    > /tmp/rulepatch_map.log 2>&1 &
  add_pid $!
  # Static localization for the known spawn (documented; AMCL omitted in
  # minimal stack). Odom drift over a 5-min demo is cm-level.
  ros2 run tf2_ros static_transform_publisher --frame-id map \
    --child-frame-id odom --x -2.0 --y -0.5 --z 0 \
    --qx 0 --qy 0 --qz 0 --qw 1 --ros-args -p use_sim_time:=True \
    > /tmp/rulepatch_statictf.log 2>&1 &
  add_pid $!
  ros2 run nav2_planner planner_server --ros-args -p use_sim_time:=True --params-file ~/R2I-RL-checkout/rulepatch/nav2_params_rulepatch.yaml > /tmp/rulepatch_planner.log 2>&1 &
  add_pid $!
  sleep 8
  echo "[demo] activate map/planner (no manager to fight)..."
  python3 ~/R2I-RL-checkout/rulepatch/tools/activate_nav2.py --timeout 150 --nodes map_server,planner_server || echo "[demo] WARN: activation incomplete"
else
  echo "[demo] FULL stack (M1 bringup)..."
  bash -c 'xvfb-run -a ros2 launch rulesync_demo rulesync_tb3_sim.launch.py headless:=True use_rviz:=False slam:=False params_file:=$HOME/rulesync_ws/nav2_params_rulesync.yaml' \
    > /tmp/rulepatch_sim.log 2>&1 &
  SIM_PID=$!; add_pid $SIM_PID
  echo "[demo] waiting for sim odom (before Nav2 needs it)..."
  for i in $(seq 1 24); do
    if timeout 20 python3 ~/R2I-RL-checkout/rulepatch/tools/wait_topic.py /odom --timeout 15 2>/dev/null | grep -q flowing; then break; fi
    sleep 5
  done
  echo "[demo] seed AMCL pose FIRST (manager autostart needs map->odom)..."
  python3 ~/R2I-RL-checkout/rulepatch/tools/seed_pose.py || echo "[demo] WARN: seed failed, continuing"
  echo "[demo] waiting for Nav2 action server (autostart converges with TF)..."
  for i in $(seq 1 60); do
    if ros2 action list 2>/dev/null | grep -q navigate_to_pose; then break; fi
    sleep 5
  done
  echo "[demo] activation top-up (best-effort; autostart usually converges)..."
  python3 ~/R2I-RL-checkout/rulepatch/tools/activate_nav2.py --timeout 150 || echo "[demo] WARN: activation incomplete, continuing"
fi

echo "[demo] RuleSync nodes..."
ros2 launch rulesync_demo rulesync_sim.launch.py > /tmp/rulepatch_nodes.log 2>&1 &
add_pid $!
sleep 5

echo "[demo] rulepatch inference + pp_driver..."
python3 -c "from rulepatch_ros.inference_node import main; main()" --ros-args -p weights_npz:="$WEIGHTS" \
  > /tmp/rulepatch_inference.log 2>&1 &
add_pid $!
"$WS/install/rulepatch_ros/bin/pp_driver" > /tmp/rulepatch_driver.log 2>&1 &
add_pid $!
sleep 5
echo "[demo] bridge automation params (no auto-motion; rollback armed)..."
for i in 1 2 3 4 5 6; do
  if python3 ~/R2I-RL-checkout/rulepatch/tools/rp_params.py rulesync_nav2_bridge auto_replan false 2>&1 | grep -q True; then break; fi
  echo "[demo] bridge not discovered yet ($i/6)..."
  sleep 10
done
python3 ~/R2I-RL-checkout/rulepatch/tools/rp_params.py rulesync_nav2_bridge auto_rollback true || true
python3 ~/R2I-RL-checkout/rulepatch/tools/rp_params.py rulesync_nav2_bridge verify_retries 2 || true

echo "[demo] recording bag..."
rm -rf /tmp/rulepatch_demo_bag
ros2 bag record -o /tmp/rulepatch_demo_bag --topics /cmd_vel /odom /scan /plan \
  /rulepatch/repaired_path /rulepatch/status /rulesync/keepout_mask \
  > /tmp/rulepatch_bag.log 2>&1 &
add_pid $!
sleep 2

DRIVE=pp
[ "$STACK" = "full" ] && DRIVE=followpath
echo "[demo] scenario: $MODE (drive=$DRIVE, up to 3 client attempts)..."
attempt=1
while [ $attempt -le 3 ]; do
  echo "[demo] client attempt $attempt..."
  if python3 ~/R2I-RL-checkout/rulepatch/tools/demo_rulepatch_client.py \
    --mode "$MODE" --goal-x "$GOAL_X" --goal-y "$GOAL_Y" --drive "$DRIVE" \
    --csv /tmp/rulepatch_demo_${MODE}.csv \
    --status-json /tmp/rulepatch_demo_${MODE}_status.json; then
    echo "[demo] client exit 0 on attempt $attempt"
    break
  fi
  echo "[demo] client failed on attempt $attempt."
  attempt=$((attempt + 1))
  sleep 5
done

echo "[demo] done. bag=/tmp/rulepatch_demo_bag csv=/tmp/rulepatch_demo.csv"
