#!/bin/bash
# Paper physics matrix (called by paper_sim.sh phase_C).
# Worlds x planners x exec-backends; gate_node enforced, pp remapped to
# gated_path (architectural invariant live).
# Usage: paper_physics.sh "<pl1,pl2>" "<exec1,exec2>"
PLUGINS="$1"
EXECS="$2"
CK=~/R2I-RL-checkout
set +u
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
source ~/rulesync_ws/install/setup.bash
set -u
export PYTHONPATH=/home/yule/R2I-RL-checkout:/home/yule/R2I-RL-checkout/rulepatch:/home/yule/R2I-RL-checkout/topopatch_ros:${PYTHONPATH:-}
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export ROS_DISABLE_ROS2_DAEMON=1
export PYTHONDONTWRITEBYTECODE=1
PAPER=~/rulepatch/paper
LOG=$PAPER/logs
WEIGHTS=~/rulepatch/topo_scope/best_fused.npz
TAU_SCOPE=1.8216713190078737
SIM_DIR=/opt/ros/lyrical/share/nav2_minimal_tb3_sim
export GZ_SIM_RESOURCE_PATH=$SIM_DIR/models:${GZ_SIM_RESOURCE_PATH:-}
# M2 pattern: manager vote-helper + service discovery need the local
# discovery server (headless stacks without it starve: gate rules-unknown).
bash -c 'source /opt/ros/lyrical/setup.bash && setsid nohup fastdds discovery -i 0 -l 127.0.0.1 -p 11811 > /tmp/ds_physics.log 2>&1 < /dev/null & echo bg' >/dev/null 2>&1 || true
sleep 3
pkill -9 -f '[f]astdds(\.py)? discovery|[f]ast-discovery-server' 2>/dev/null || true
  sleep 2
    bash -c 'source /opt/ros/lyrical/setup.bash && setsid nohup fastdds discovery -i 0 -l 127.0.0.1 -p 11811 > /tmp/ds_paper.log 2>&1 < /dev/null & echo bg' >/dev/null 2>&1 || true
  sleep 3

kill_world() {
  for pat in '[g]z-sim-main' '[g]z sim' '[n]av2_container' '[p]lanner_server' \
             '[m]ap_server' '[c]ontroller_server' '[r]obot_state_publisher' \
             '[p]arameter_bridge' '[r]os2 launch' '[t]opo_inference' \
             '[g]ate_node' '[p]p_driver' '[t]opo_case_driver' '[x]vfb-run' \
             '[r]ulesync_manager' '[r]ulesync_nav2_bridge'; do
    pkill -9 -f "$pat" 2>/dev/null || true
  done
  sleep 4
}

run_world() { # <tag> <sdf|--sandbox> <map_yaml> <spawn_x> <spawn_y> <goals_json>
  local TAG=$1 SDF=$2 MAPYAML=$3 SX=$4 SY=$5 GOALS=$6
  echo "[C] world $TAG spawn=($SX,$SY)..."
  if [ "$SDF" = "--sandbox" ]; then
    xacro -o /tmp/paper_world.sdf headless:=True "$SIM_DIR/worlds/tb3_sandbox.sdf.xacro"
    SDF=/tmp/paper_world.sdf
  fi
  xvfb-run -a gz sim -r -s "$SDF" > $LOG/C_${TAG}_sim.log 2>&1 &
  sleep 12
  ros2 launch nav2_minimal_tb3_sim spawn_tb3.launch.py \
    robot_name:=turtlebot3_waffle robot_sdf:="$SIM_DIR/urdf/gz_waffle.sdf.xacro" \
    x_pose:=$SX y_pose:=$SY z_pose:=0.01 > $LOG/C_${TAG}_spawn.log 2>&1 &
  ros2 run robot_state_publisher robot_state_publisher --ros-args \
    -p use_sim_time:=True \
    -p robot_description:="$(cat $SIM_DIR/urdf/turtlebot3_waffle.urdf)" \
    > $LOG/C_${TAG}_rsp.log 2>&1 &
  for i in $(seq 1 24); do
    if timeout 20 python3 $CK/rulepatch/tools/wait_topic.py /odom --timeout 15 2>/dev/null | grep -q flowing; then break; fi
    sleep 5
  done
  sleep 3
  python3 $CK/topopatch/tools/paper_planner_cfg.py --pgm-yaml $MAPYAML \
    --plugins "$PLUGINS" --out /tmp/paper_planner_$TAG.yaml || return 1
  cat > /tmp/paper_map_$TAG.yaml <<EOF
/**:
  ros__parameters:
    yaml_filename: "$MAPYAML"
    topic_name: "map"
    frame_id: "map"
EOF
  ros2 run nav2_map_server map_server --ros-args -p use_sim_time:=True \
    --params-file /tmp/paper_map_$TAG.yaml > $LOG/C_${TAG}_map.log 2>&1 &
  ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom \
    --x $SX --y $SY --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --ros-args -p use_sim_time:=True \
    > $LOG/C_${TAG}_tf.log 2>&1 &
  ros2 run nav2_planner planner_server --ros-args -p use_sim_time:=True \
    --params-file /tmp/paper_planner_$TAG.yaml > $LOG/C_${TAG}_planner.log 2>&1 &
  ros2 run nav2_controller controller_server --ros-args -p use_sim_time:=True \
    --params-file $CK/topopatch/nav2_controllers.yaml > $LOG/C_${TAG}_ctrl.log 2>&1 &
  sleep 8
  python3 $CK/rulepatch/tools/activate_nav2.py --timeout 150 \
    --nodes map_server,planner_server,controller_server > $LOG/C_${TAG}_act.log 2>&1 || true
  ros2 launch rulesync_demo rulesync_sim.launch.py > $LOG/C_${TAG}_nodes.log 2>&1 &
  sleep 8
  python3 -c "from topopatch_ros.topo_inference_node import main; main()" \
    --ros-args -p weights_npz:=$WEIGHTS -p tau:=$TAU_SCOPE -p use_scope:=True \
    > $LOG/C_${TAG}_topo.log 2>&1 &
  python3 -c "from topopatch_ros.gate_node import main; main()" \
    > $LOG/C_${TAG}_gate.log 2>&1 &
  python3 -c "from rulepatch_ros.pp_driver import main; main()" --ros-args \
    -r /rulepatch/repaired_path:=/rulepatch/gated_path -p speed:=0.25 \
    > $LOG/C_${TAG}_pp.log 2>&1 &
  sleep 5
  for i in 1 2 3 4 5 6; do
    if python3 $CK/rulepatch/tools/rp_params.py rulesync_nav2_bridge auto_replan false 2>&1 | grep -q True; then break; fi
    sleep 10
  done
  python3 -c "import json; g=json.load(open('$GOALS')); print(json.dumps([{'goal': g['goals'][0], 'epoch': 601}, {'goal': g['goals'][1], 'epoch': 611}]))" > /tmp/paper_cases_$TAG.json
  pli=0
  for pl in $(echo $PLUGINS | tr ',' ' '); do
    pli=$((pli + 1))
    exi=0
    for ex in $(echo $EXECS | tr ',' ' '); do
      exi=$((exi + 1))
      # unique epochs per cell (manager rejects stale reuse across cells)
      EBOFF=$((pli * 100 + exi * 20))
      python3 -c "import json; g=json.load(open('$GOALS')); print(json.dumps([{'goal': g['goals'][0], 'epoch': 601+$EBOFF}, {'goal': g['goals'][1], 'epoch': 611+$EBOFF}]))" > /tmp/paper_cases_${TAG}_${pl}_${ex}.json
      if [ "$ex" = "pp" ]; then MODE_ARGS="--exec pp"; else MODE_ARGS="--exec followpath --controller $ex"; fi
      TOPO_SPAWN="$SX,$SY" TOPO_MAP="$TAG" timeout 900 python3 -m topopatch_ros.topo_case_driver \
        --csv $PAPER/driver_${TAG}_${pl}_${ex}.csv --planner $pl \
        --cases-file /tmp/paper_cases_${TAG}_${pl}_${ex}.json $MODE_ARGS \
        > $LOG/C_${TAG}_${pl}_${ex}.log 2>&1 || true
    done
  done
  kill_world
}

SANDBOX_YAML=/opt/ros/lyrical/share/nav2_bringup/maps/tb3_sandbox.yaml
# Optional world filter as $3: paper_physics.sh "<pls>" "<execs>" [only <tag>|all]
ONLY="${3:-all}"
should_run() { [ "$ONLY" = "all" ] || [ "$ONLY" = "$1" ]; }
# sandbox goals (cleared cells near arena)
~/miniforge3/envs/causal-repair/bin/python $CK/topopatch/tools/paper_goals.py --pgm /opt/ros/lyrical/share/nav2_bringup/maps/tb3_sandbox.pgm \
  --start=-2.0,-0.5 --k 2 --seed 0 --out $PAPER/goals_sandbox.json || true
kill_world
should_run sandbox && run_world sandbox --sandbox $SANDBOX_YAML -2.0 -0.5 $PAPER/goals_sandbox.json
for m in office-A-s101 narrow-A-s301 maze-A-s201; do
  should_run $m || continue
  SP=$(python3 -c "import json;print(','.join(map(str,json.load(open('$CK/topopatch/tools/paper_goals_${m%%-*}.json'))['start'])))" 2>/dev/null || echo "-2.0,-0.5")
  SX=${SP%%,*}; SY=${SP##*,}
  run_world $m $PAPER/world_$m.sdf ~/rulepatch/topo_maps/$m.yaml $SX $SY \
    $CK/topopatch/tools/paper_goals_${m%%-*}.json
done
kill_world
echo "[C] physics matrix done."
