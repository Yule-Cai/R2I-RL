#!/bin/bash
# TopoRulePatch M3 Gazebo closed-loop (tb3_sandbox, minimal stack).
# Mirrors M2's proven demo_rulepatch.sh: discovery server, SHM hygiene,
# pre-flight pkill, xvfb gz + spawn + RSP, map_server + static map->odom
# TF (AMCL omitted) + standalone planner_server (manually activated) +
# manager/bridge + topo_inference (fused v1, frozen tau) + pp_driver +
# bag + scripted cases. M2 files untouched.
# Usage: bash demo_topo.sh
WS=~/rulesync_ws
WEIGHTS=~/rulepatch/topo_full/best_fused.npz
TAU=1.0264495134353637
source /opt/ros/lyrical/setup.bash
source "$WS/install/setup.bash"
export PYTHONPATH=/home/yule/R2I-RL-checkout:/home/yule/R2I-RL-checkout/rulepatch:/home/yule/R2I-RL-checkout/topopatch_ros:${PYTHONPATH:-}
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export ROS_DISABLE_ROS2_DAEMON=1
export PYTHONDONTWRITEBYTECODE=1
test -f "$WEIGHTS" || { echo "[demo] missing weights: $WEIGHTS"; exit 1; }

echo "[demo] pre-flight: kill stale + SHM..."
for pat in '[g]z-sim-main' '[g]z sim' '[n]av2_container' '[p]lanner_server' \
           '[m]ap_server' '[a]mcl' '[r]ulesync_manager/manager' \
           '[r]ulesync_nav2_bridge/bridge' '[t]opo_inference' \
           '[t]opo_case_driver' '[p]p_driver' '[r]os2 launch' '[x]vfb-run' \
           '[X]vfb :' '[r]obot_state_publisher' '[p]arameter_bridge' \
           '[r]os2 bag record' '[g]en_topo_dataset' '[t]rain_topo' \
           '[e]valuate_topo'; do
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

echo "[demo] discovery server..."
pkill -9 -f '[f]astdds(\.py)? discovery|[f]ast-discovery-server' 2>/dev/null || true
sleep 2
bash -c 'source /opt/ros/lyrical/setup.bash && setsid nohup fastdds discovery -i 0 -l 127.0.0.1 -p 11811 > /tmp/ds_server.log 2>&1 < /dev/null & echo bg'
sleep 3

echo "[demo] gz sim (xvfb, headless)..."
WORLD_SDF=/tmp/rulepatch_world.sdf
xacro -o "$WORLD_SDF" headless:=True "$SIM_DIR/worlds/tb3_sandbox.sdf.xacro"
xvfb-run -a gz sim -r -s "$WORLD_SDF" > /tmp/topo_sim.log 2>&1 &
add_pid $!
sleep 10
echo "[demo] spawn + RSP..."
ros2 launch nav2_minimal_tb3_sim spawn_tb3.launch.py \
  robot_name:=turtlebot3_waffle robot_sdf:="$SIM_DIR/urdf/gz_waffle.sdf.xacro" \
  x_pose:=-2.0 y_pose:=-0.5 z_pose:=0.01 > /tmp/topo_spawn.log 2>&1 &
add_pid $!
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p use_sim_time:=True \
  -p robot_description:="$(cat $SIM_DIR/urdf/turtlebot3_waffle.urdf)" \
  > /tmp/topo_rsp.log 2>&1 &
add_pid $!
echo "[demo] waiting for sim odom..."
for i in $(seq 1 24); do
  if timeout 20 python3 ~/R2I-RL-checkout/rulepatch/tools/wait_topic.py /odom --timeout 15 2>/dev/null | grep -q flowing; then break; fi
  sleep 5
done
sleep 5

echo "[demo] map_server + static TF + planner..."
MAP_YAML=$BRINGUP_DIR/maps/tb3_sandbox.yaml
cat > /tmp/topo_map_params.yaml <<EOF
/**:
  ros__parameters:
    yaml_filename: "$MAP_YAML"
    topic_name: "map"
    frame_id: "map"
EOF
ros2 run nav2_map_server map_server --ros-args -p use_sim_time:=True \
  --params-file /tmp/topo_map_params.yaml > /tmp/topo_map.log 2>&1 &
add_pid $!
ros2 run tf2_ros static_transform_publisher --frame-id map \
  --child-frame-id odom --x -2.0 --y -0.5 --z 0 \
  --qx 0 --qy 0 --qz 0 --qw 1 --ros-args -p use_sim_time:=True \
  > /tmp/topo_statictf.log 2>&1 &
add_pid $!
ros2 run nav2_planner planner_server --ros-args -p use_sim_time:=True \
  --params-file ~/R2I-RL-checkout/topopatch/nav2_params_topopatch.yaml \
  > /tmp/topo_planner.log 2>&1 &
add_pid $!
sleep 8
echo "[demo] activate map/planner..."
python3 ~/R2I-RL-checkout/rulepatch/tools/activate_nav2.py --timeout 150 \
  --nodes map_server,planner_server || echo "[demo] WARN: activation incomplete"

echo "[demo] manager + bridge..."
ros2 launch rulesync_demo rulesync_sim.launch.py > /tmp/topo_nodes.log 2>&1 &
add_pid $!
sleep 5

echo "[demo] topo_inference + pp_driver..."
python3 -c "from topopatch_ros.topo_inference_node import main; main()" \
  --ros-args -p weights_npz:="$WEIGHTS" -p tau:=$TAU \
  > /tmp/topo_inference.log 2>&1 &
add_pid $!
"$WS/install/rulepatch_ros/bin/pp_driver" > /tmp/topo_driver.log 2>&1 &
add_pid $!
sleep 5
echo "[demo] bridge: no auto-motion..."
for i in 1 2 3 4 5 6; do
  if python3 ~/R2I-RL-checkout/rulepatch/tools/rp_params.py rulesync_nav2_bridge auto_replan false 2>&1 | grep -q True; then break; fi
  echo "[demo] bridge not discovered yet ($i/6)..."
  sleep 10
done

echo "[demo] recording bag..."
rm -rf ~/rulepatch/gz_bag
ros2 bag record -o ~/rulepatch/gz_bag --topics /cmd_vel /odom /scan /plan \
  /rulepatch/repaired_path /rulepatch/status /rulesync/keepout_mask \
  /rulesync/staged_set > /tmp/topo_bag.log 2>&1 &
add_pid $!
sleep 2

echo "[demo] cases (up to 2 attempts)..."
attempt=1
while [ $attempt -le 2 ]; do
  if python3 -m topopatch_ros.topo_case_driver \
    --csv ~/rulepatch/gz_cases.csv; then
    echo "[demo] driver exit 0 on attempt $attempt"
    break
  fi
  echo "[demo] driver failed on attempt $attempt."
  attempt=$((attempt + 1))
  sleep 5
done

echo "[demo] summary:"; cat ~/rulepatch/gz_cases.csv 2>/dev/null
echo "[demo] done. bag=~/rulepatch/gz_bag"
