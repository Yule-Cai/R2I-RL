#!/bin/bash
# PAPER sim evaluation orchestrator (runs on onexplayer).
# Frozen method (r2i-rulepatch-m4 @ 9d63a8a): weights, thresholds,
# verifier, scope policy, dataset splits. This script + harness are
# evaluation code, NOT method changes. No retraining/tuning anywhere.
#
# Phases (each resumable; evidence under ~/rulepatch/paper/):
#   P: headless planner matrix (warehouse/office/maze/narrow x
#      planners-available x n_rules 1..3): repair metrics, no execution.
#   C: closed-loop physics (sandbox + built SDF worlds) x planners x
#      exec(pp/DWB/MPPI): arrivals, collisions, violations=0, latencies.
#   F: fault injection F1..F6 (fail-closed + recovery latencies).
#   E: 60-min endurance (mem, liveness, epochs, deadlocks).
#   R: analyze -> TABLES.md + paper_summary.json
# Usage: bash paper_sim.sh [P|C|F|E|R|ALL]   (default ALL)
PH=${1:-ALL}
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export ROS_DISABLE_ROS2_DAEMON=1
export PYTHONDONTWRITEBYTECODE=1
set -u
set +u
source /opt/ros/lyrical/setup.bash
source ~/rulesync_ws/install/setup.bash
set -u
export PYTHONPATH=/home/yule/R2I-RL-checkout:/home/yule/R2I-RL-checkout/rulepatch:/home/yule/R2I-RL-checkout/topopatch_ros:${PYTHONPATH:-}
CK=~/R2I-RL-checkout
PAPER=~/rulepatch/paper
LOG=$PAPER/logs
WEIGHTS=~/rulepatch/topo_scope/best_fused.npz
mkdir -p $PAPER $LOG
TAU_SCOPE=1.8216713190078737

probe_plugins() {
  # echoes available planner ids / controller availability
  PLUGINS="GridBased"
  for spec in "nav2_smac_planner:Smac2D" "nav2_theta_star_planner:ThetaStar"; do
    pkg=${spec%%:*}; id=${spec##*:}
    if ros2 pkg prefix $pkg >/dev/null 2>&1; then PLUGINS="$PLUGINS,$id"; fi
  done
  echo "$PLUGINS" > $PAPER/planners_available.txt
  CTRLS="pp"
  if ros2 pkg prefix nav2_dwb_controller >/dev/null 2>&1 || ros2 pkg prefix dwb_core >/dev/null 2>&1; then CTRLS="$CTRLS,dwb"; fi
  if ros2 pkg prefix nav2_mppi_controller >/dev/null 2>&1 || ros2 pkg prefix mppi_controller >/dev/null 2>&1; then CTRLS="$CTRLS,mppi"; fi
  echo "$CTRLS" > $PAPER/controllers_available.txt
  cat $PAPER/planners_available.txt $PAPER/controllers_available.txt
  cp "$WEIGHTS" $PAPER/ 2>/dev/null || true
  sha256sum $WEIGHTS 2>/dev/null | tee $PAPER/weights.sha256
  cat ~/rulepatch/freeze_m4.json 2>/dev/null | head -5 > $PAPER/freeze_ref.txt || true
}

kill_stack() {
  for pat in '[g]z-sim-main' '[g]z sim' '[n]av2_container' '[p]lanner_server' \
             '[m]ap_server' '[c]ontroller_server' '[a]mcl' '[l]ifecycle_manager' \
             '[r]ulesync_manager' '[r]ulesync_nav2_bridge' '[t]opo_inference' \
             '[g]ate_node' '[p]p_driver' '[t]opo_case_driver' '[b]ench_client' \
             '[f]ault_client' '[m]ask_delay' '[e]ndurance' '[r]obot_state_publisher' \
             '[p]arameter_bridge' '[r]os2 launch' '[r]os2 bag record' '[x]vfb-run'; do
    pkill -9 -f "$pat" 2>/dev/null || true
  done
  sleep 3
}

phase_P() {
  echo "[P] headless planner matrix..."
  probe_plugins
  PLUGINS=$(cat $PAPER/planners_available.txt)
  for map in warehouse office-A-s101 maze-A-s201 narrow-A-s301; do
    case $map in
      warehouse) PGM=/opt/ros/lyrical/share/nav2_bringup/maps/warehouse.pgm; YL=/opt/ros/lyrical/share/nav2_bringup/maps/warehouse.yaml; CM=0.5 ;;
      office-A-s101) PGM=~/rulepatch/topo_maps/office-A-s101.pgm; YL=${PGM%.pgm}.yaml; CM=0.5 ;;
      maze-A-s201) PGM=~/rulepatch/topo_maps/maze-A-s201.pgm; YL=${PGM%.pgm}.yaml; CM=0.36 ;;
      narrow-A-s301) PGM=~/rulepatch/topo_maps/narrow-A-s301.pgm; YL=${PGM%.pgm}.yaml; CM=0.5 ;;
    esac
    # goals (procedural frozen in repo; warehouse generated on host)
    if [ ! -f $PAPER/goals_$map.json ]; then
      ~/miniforge3/envs/causal-repair/bin/python $CK/topopatch/tools/paper_goals.py --pgm $PGM --res 0.05 \
        --ox -10 --oy -10 --clear-m $CM --k 4 --seed 0 \
        --out $PAPER/goals_$map.json || { echo "[P] goals failed $map"; continue; }
    fi
    YL=${PGM%.pgm}.yaml
    python3 $CK/topopatch/tools/paper_planner_cfg.py --pgm-yaml $YL \
      --plugins $PLUGINS --out /tmp/paper_planner_$map.yaml
    START=$(python3 -c "import json;print(','.join(map(str,json.load(open('$PAPER/goals_$map.json'))['start'])))")
    kill_stack
    unset ROS_DOMAIN_ID
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/lyrical/setup.bash
    set -u
  # M2 pattern: manager vote-helper hardcodes ROS_DISCOVERY_SERVER=127.0.0.1:11811
  # on the DEFAULT domain; headless stacks must provide it (else every
  # commit aborts with no-service votes).
pkill -9 -f '[f]astdds(\.py)? discovery|[f]ast-discovery-server' 2>/dev/null || true
  sleep 2
    bash -c 'source /opt/ros/lyrical/setup.bash && setsid nohup fastdds discovery -i 0 -l 127.0.0.1 -p 11811 > /tmp/ds_paper.log 2>&1 < /dev/null & echo bg' >/dev/null 2>&1 || true
  sleep 3
    cd $CK
    SX=${START%%,*}; SY=${START##*,}
    ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom \
      --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 > $LOG/P_${map}_tf.log 2>&1 &
    ros2 run tf2_ros static_transform_publisher --frame-id odom --child-frame-id base_link \
      --x $SX --y $SY --z 0 --qx 0 --qy 0 --qz 0 --qw 1 >> $LOG/P_${map}_tf.log 2>&1 &
    ros2 run nav2_map_server map_server --ros-args --params-file /tmp/paper_planner_$map.yaml > $LOG/P_${map}_map.log 2>&1 &
    ros2 run nav2_planner planner_server --ros-args --params-file /tmp/paper_planner_$map.yaml > $LOG/P_${map}_planner.log 2>&1 &
    sleep 8
    python3 rulepatch/tools/activate_nav2.py --timeout 100 --nodes map_server,planner_server > $LOG/P_${map}_act.log 2>&1 || true
    ros2 launch rulesync_demo rulesync_sim.launch.py > $LOG/P_${map}_nodes.log 2>&1 &
    sleep 8
    python3 -c "from topopatch_ros.topo_inference_node import main; main()" \
      --ros-args -p weights_npz:=$WEIGHTS -p tau:=$TAU_SCOPE -p use_scope:=True > $LOG/P_${map}_topo.log 2>&1 &
    setsid nohup python3 -m topopatch_ros.scan_stub > $LOG/P_${map}_scan.log 2>&1 < /dev/null &
    sleep 4
    pli=0
    for pl in $(echo $PLUGINS | tr ',' ' '); do
      pli=$((pli + 1))
      for nr in 1 2 3; do
        python3 -m topopatch_ros.bench_client --csv $PAPER/bench_${map}_${pl}_r${nr}.csv \
          --planner $pl --map $map --start $START --goals $PAPER/goals_$map.json \
          --epoch0 $((300 + pli * 100 + nr * 10)) --n-rules $nr > $LOG/P_${map}_${pl}_r${nr}.log 2>&1 || true
      done
    done
    kill_stack
    unset ROS_DOMAIN_ID
  done
}

phase_C() {
  echo "[C] closed-loop physics..."
  PLUGINS=$(cat $PAPER/planners_available.txt 2>/dev/null || echo GridBased)
  CTRLS=$(cat $PAPER/controllers_available.txt 2>/dev/null || echo pp)
  # build procedural worlds (sandbox uses stock xacro world)
  for m in office-A-s101 narrow-A-s301 maze-A-s201; do
    if [ ! -f $PAPER/world_$m.sdf ]; then
      python3 $CK/topopatch/tools/pgm_to_sdf.py --pgm ~/rulepatch/topo_maps/$m.pgm \
        --out $PAPER/world_$m.sdf > $LOG/world_$m.log 2>&1 || echo "[C] world build failed $m"
    fi
  done
  echo "[C] physics matrix: see paper_cases.sh per-cell logs"
  bash $CK/topopatch/tools/paper_physics.sh "$PLUGINS" "$CTRLS"
}

phase_F() {
  echo "[F] fault injection F1..F6 (sandbox headless)..."
  bash $CK/topopatch/tools/paper_faults.sh
}

phase_E() {
  echo "[E] 60-min endurance (sandbox headless)..."
  bash $CK/topopatch/tools/paper_endurance.sh
}

phase_R() {
  echo "[R] analyzing..."
  ~/miniforge3/envs/causal-repair/bin/python -m topopatch.tools.analyze_paper \
    --bench "$PAPER/bench_*.csv" --driver "$PAPER/driver_*.csv" \
    --faults "$PAPER/fault_*.csv" --endurance $PAPER/endurance.csv \
    --out $PAPER/analysis
}

case $PH in
  P) phase_P;; C) phase_C;; F) phase_F;; E) phase_E;; R) phase_R;;
  ALL) phase_P; phase_C; phase_F; phase_E; phase_R;;
esac
echo "[paper] phase $PH done."
