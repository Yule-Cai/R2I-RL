#!/bin/bash
# Crash-consistency matrix: >=20 controlled crash points.
# F8m grid: hook pause P in {0,2,8}s x kill-delay K in
# {0.5,1.5,3.0,5.0,9.0}s (15 pts) + F10 both-restart x3 orders +
# 2 no-crash baselines (kill-delay 120 s) = 20 points.
# Fresh manager+bridge + deleted snapshot per point (isolation).
export PYTHONDONTWRITEBYTECODE=1
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export ROS_DISABLE_ROS2_DAEMON=1
set +u
source /opt/ros/lyrical/setup.bash
source /home/yule/rulesync_ws/install/setup.bash
set -u
export PYTHONPATH=/home/yule/R2I-RL-checkout:/home/yule/R2I-RL-checkout/rulepatch:/home/yule/R2I-RL-checkout/topopatch_ros:${PYTHONPATH:-}
CK=/home/yule/R2I-RL-checkout
PAPER=/home/yule/rulepatch/paper
LOG=$PAPER/logs
WEIGHTS=/home/yule/rulepatch/topo_scope/best_fused.npz
TAU_SCOPE=1.8216713190078737
SANDBOX_YAML=/opt/ros/lyrical/share/nav2_bringup/maps/tb3_sandbox.yaml
MATRIX=$PAPER/crash_matrix.csv
rm -f $MATRIX

kill_all() {
  for pat in '[g]z-sim-main' '[g]z sim' '[n]av2_container' '[p]lanner_server' \
             '[m]ap_server' '[c]ontroller_server' '[a]mcl' '[l]ifecycle_manager' \
             '[r]ulesync_manager' '[r]ulesync_nav2_bridge' '[t]opo_inference' \
             '[g]ate_node' '[p]p_driver' '[t]opo_case_driver' '[b]ench_client' \
             '[f]ault_client' '[r]obot_state_publisher' '[p]arameter_bridge' \
             '[r]os2 launch' '[r]os2 run' '[s]tatic_transform_publisher' \
             '[s]can_stub' '[m]ask_delay' '[x]vfb-run'; do
    pkill -9 -f "$pat" 2>/dev/null || true
  done
  sleep 4
}

fresh_ds() {
  pkill -9 -f '[f]astdds(.py)? discovery|[f]ast-discovery-server' 2>/dev/null || true
  sleep 2
  rm -f /dev/shm/fastdds_* /dev/shm/sem.fastdds_* 2>/dev/null || true
  bash -c 'source /opt/ros/lyrical/setup.bash && setsid nohup fastdds discovery -i 0 -l 127.0.0.1 -p 11811 > /tmp/ds_crash.log 2>&1 < /dev/null & echo bg' >/dev/null 2>&1 || true
  sleep 4
}

bringup_base() {
  cat > /tmp/fc_map.yaml <<EOF
/**:
  ros__parameters:
    yaml_filename: "$SANDBOX_YAML"
    topic_name: "map"
    frame_id: "map"
EOF
  setsid nohup ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom \
    --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 > $LOG/CM_tf.log 2>&1 < /dev/null &
  setsid nohup ros2 run tf2_ros static_transform_publisher --frame-id odom --child-frame-id base_link \
    --x -2 --y -0.5 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 >> $LOG/CM_tf.log 2>&1 < /dev/null &
  setsid nohup ros2 run nav2_map_server map_server --ros-args -p use_sim_time:=False \
    --params-file /tmp/fc_map.yaml > $LOG/CM_map.log 2>&1 < /dev/null &
  python3 $CK/topopatch/tools/paper_planner_cfg.py --pgm-yaml $SANDBOX_YAML \
    --plugins GridBased --out /tmp/fc_planner.yaml
  setsid nohup ros2 run nav2_planner planner_server --ros-args -p use_sim_time:=False \
    --params-file /tmp/fc_planner.yaml > $LOG/CM_planner.log 2>&1 < /dev/null &
  sleep 8
  timeout 90 python3 $CK/rulepatch/tools/activate_nav2.py --timeout 80 \
    --nodes map_server,planner_server 2>&1 | tail -1
  setsid nohup python3 -c "from topopatch_ros.topo_inference_node import main; main()" \
    --ros-args -p weights_npz:=$WEIGHTS -p tau:=$TAU_SCOPE -p use_scope:=True \
    > $LOG/CM_topo.log 2>&1 < /dev/null &
  setsid nohup python3 -c "from topopatch_ros.gate_node import main; main()" \
    > $LOG/CM_gate.log 2>&1 < /dev/null &
  setsid nohup python3 -m topopatch_ros.scan_stub > $LOG/CM_scan.log 2>&1 < /dev/null &
  sleep 6
}

launch_pair() { # $1 = hook seconds (0 = plain)
  rm -f /tmp/rulesync_state.json
  setsid nohup bash -c "source /opt/ros/lyrical/setup.bash; source /home/yule/rulesync_ws/install/setup.bash; export ROS_DISCOVERY_SERVER=127.0.0.1:11811 ROS_DISABLE_ROS2_DAEMON=1 RULESYNC_TEST_COMMIT_PAUSE_S=$1; ros2 launch rulesync_demo rulesync_sim.launch.py" > $LOG/CM_nodes.log 2>&1 < /dev/null &
  sleep 12
}

wait_pair_services() {
  # Per-restart discovery wait: a fresh manager+bridge needs time before
  # the client's proposes/list calls can succeed (else spurious fails).
  set +u
  source /opt/ros/lyrical/setup.bash
  set -u
  export ROS_DISCOVERY_SERVER=127.0.0.1:11811 ROS_DISABLE_ROS2_DAEMON=1
  for i in $(seq 1 12); do
    if timeout 10 ros2 service list 2>/dev/null | grep -q propose_rule; then
      echo "[matrix] pair services visible"
      return 0
    fi
    sleep 5
  done
  echo "[matrix] WARNING: pair services not visible" >&2
  return 1
}

kill_pair() {
  for pat in '[r]ulesync_manager' '[r]ulesync_nav2_bridge' '[r]os2 launch'; do
    pkill -9 -f "$pat" 2>/dev/null || true
  done
  sleep 3
}

run_fault() { # <fault> <epoch> [extra...] -> appends row to MATRIX
  local F=$1 EP=$2; shift 2
  cd $CK
  timeout 600 python3 -m topopatch_ros.fault_client --fault $F \
    --csv /tmp/cm_row.csv --epoch $EP \
    --start=-2.0,-0.5 --goal=2.0,0.5 "$@" 2>&1 | tail -1
  python3 -c "import csv; r=list(csv.DictReader(open('/tmp/cm_row.csv')))[0]; print('ROW:', {k:r[k] for k in ('fault','point','fail_closed','silent_divergence','motion') if k in r})"
  if [ ! -f $MATRIX ]; then
    python3 -c "import csv,sys; r=list(csv.DictReader(open('/tmp/cm_row.csv')))[0]; w=csv.DictWriter(open('$MATRIX','w',newline=''),fieldnames=sorted(r.keys())); w.writeheader(); w.writerow(r)"
  else
    python3 -c "import csv; rows=list(csv.DictReader(open('$MATRIX'))); r=list(csv.DictReader(open('/tmp/cm_row.csv')))[0]; keys=sorted(set().union(*[rr.keys() for rr in rows+[r]])); w=csv.DictWriter(open('$MATRIX','w',newline=''),fieldnames=keys); w.writeheader(); [w.writerow({k:rr.get(k) for k in keys}) for rr in rows+[r]]"
  fi
}

kill_all
fresh_ds
cd $CK
bringup_base
echo "[matrix] preflight..."
if ! timeout 150 python3 /tmp/rep_preflight.py -2.0 -0.5 2.0 0.5 2>&1 | tail -1 | grep -qE "PREFLIGHT nposes=([4-9]|[0-9]{2,})"; then
  echo "[matrix] PREFLIGHT FAILED" >&2; exit 1
fi
echo "[matrix] PREFLIGHT OK"
EP=30000
for P in 0 2 8; do
  for K in 0.5 1.5 3.0 5.0 9.0; do
    echo "[matrix] point P=$P K=$K EP=$EP"
    kill_pair
    launch_pair $P
    wait_pair_services
    run_fault f8m $EP --kill-delay $K --point "P${P}_K${K}"
    EP=$((EP + 100))
  done
done
# F10 both-restart orders (setup rule @ EP first)
for ORD in mgr-first bridge-first mgr-only; do
  echo "[matrix] F10 $ORD EP=$EP"
  kill_pair
  launch_pair 0
  wait_pair_services
  run_fault f10setup $EP --point "f10setup-$ORD" >/dev/null 2>&1
  for pat in '[r]ulesync_manager' '[r]ulesync_nav2_bridge' '[r]os2 launch'; do
    pkill -9 -f "$pat" 2>/dev/null || true
  done
  sleep 3
  case $ORD in
    mgr-first) launch_pair 0 ;;
    bridge-first)
      setsid nohup bash -c 'source /opt/ros/lyrical/setup.bash; source /home/yule/rulesync_ws/install/setup.bash; export ROS_DISCOVERY_SERVER=127.0.0.1:11811 ROS_DISABLE_ROS2_DAEMON=1; ros2 run rulesync_nav2_bridge bridge' > $LOG/CM_bridgeonly.log 2>&1 < /dev/null &
      sleep 8
      setsid nohup bash -c 'source /opt/ros/lyrical/setup.bash; source /home/yule/rulesync_ws/install/setup.bash; export ROS_DISCOVERY_SERVER=127.0.0.1:11811 ROS_DISABLE_ROS2_DAEMON=1; ros2 run rulesync_manager manager --ros-args -p participants:="[nav2_bridge]" -p txn_log_path:=/tmp/rulesync_txn.log' > $LOG/CM_mgronly.log 2>&1 < /dev/null &
      sleep 12 ;;
    mgr-only)
      setsid nohup bash -c 'source /opt/ros/lyrical/setup.bash; source /home/yule/rulesync_ws/install/setup.bash; export ROS_DISCOVERY_SERVER=127.0.0.1:11811 ROS_DISABLE_ROS2_DAEMON=1; ros2 run rulesync_manager manager --ros-args -p participants:="[nav2_bridge]" -p txn_log_path:=/tmp/rulesync_txn.log' > $LOG/CM_mgronly.log 2>&1 < /dev/null &
      sleep 12 ;;
  esac
  wait_pair_services
  run_fault f10 $((EP + 50)) --point "f10-$ORD"
  EP=$((EP + 100))
done
# baselines (no crash: killer fires long after return, cancelled)
for P in 0 8; do
  echo "[matrix] baseline P=$P"
  kill_pair
  launch_pair $P
  run_fault f8m $EP --kill-delay 120 --point "baseline-P$P"
  EP=$((EP + 100))
done
echo "[matrix] done. rows:"; python3 -c "import csv; print(len(list(csv.DictReader(open('$MATRIX')))))"
