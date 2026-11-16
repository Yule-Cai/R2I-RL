#!/bin/bash
# Fault + crash campaign: F1-F6 re-run under the strengthened gate,
# then F7-F9 crash/restart consistency. Sandbox headless stack, fresh
# manager+bridge per fault (also exercises restart robustness).
# Usage: run_fault_crash.sh [f1 f2 ...] (default: all f1..f9)
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
FAULTS="${@:-f1 f2 f3 f4 f5 f6 f7 f8 f9}"

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
  bash -c 'source /opt/ros/lyrical/setup.bash && setsid nohup fastdds discovery -i 0 -l 127.0.0.1 -p 11811 > /tmp/ds_fault.log 2>&1 < /dev/null & echo bg' >/dev/null 2>&1 || true
  sleep 4
  if ! pgrep -f '[f]ast-discovery-server' >/dev/null; then
    echo "[fault] FATAL: ds not up" >&2; exit 1
  fi
  echo "[fault] ds up"
}

bringup_base() {
  # map + TF + map_server (idempotent parts guarded: only start map
  # server/TF if absent)
  cat > /tmp/fc_map.yaml <<EOF
/**:
  ros__parameters:
    yaml_filename: "$SANDBOX_YAML"
    topic_name: "map"
    frame_id: "map"
EOF
  if ! pgrep -f '[t]f2_ros.*map.*odom' >/dev/null; then
    setsid nohup ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom \
      --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 > $LOG/FC_tf.log 2>&1 < /dev/null &
    setsid nohup ros2 run tf2_ros static_transform_publisher --frame-id odom --child-frame-id base_link \
      --x -2 --y -0.5 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 >> $LOG/FC_tf.log 2>&1 < /dev/null &
  fi
  if ! pgrep -f '[n]av2_map_server/map_server' >/dev/null; then
    setsid nohup ros2 run nav2_map_server map_server --ros-args -p use_sim_time:=False \
      --params-file /tmp/fc_map.yaml > $LOG/FC_map.log 2>&1 < /dev/null &
    sleep 3
    timeout 90 python3 $CK/rulepatch/tools/activate_nav2.py --timeout 80 \
      --nodes map_server 2>&1 | tail -1
  fi
  start_planner
}

start_planner() {
  python3 $CK/topopatch/tools/paper_planner_cfg.py --pgm-yaml $SANDBOX_YAML \
    --plugins GridBased --out /tmp/fc_planner.yaml
  if [ "${F1_PROXY:-0}" = "1" ]; then
    setsid nohup ros2 run nav2_planner planner_server --ros-args -p use_sim_time:=False \
      --params-file /tmp/fc_planner.yaml -r /rulesync/keepout_mask:=/mask_delayed \
      > $LOG/FC_planner.log 2>&1 < /dev/null &
    setsid nohup python3 -m topopatch_ros.mask_delay --delay 20.0 --out-topic /mask_delayed \
      > $LOG/FC_proxy.log 2>&1 < /dev/null &
  else
    setsid nohup ros2 run nav2_planner planner_server --ros-args -p use_sim_time:=False \
      --params-file /tmp/fc_planner.yaml > $LOG/FC_planner.log 2>&1 < /dev/null &
  fi
  sleep 8
  timeout 90 python3 $CK/rulepatch/tools/activate_nav2.py --timeout 80 \
    --nodes planner_server 2>&1 | tail -1
}

launch_rulesync() {
  # $1 = extra env assignments, e.g. RULESYNC_TEST_COMMIT_PAUSE_S=8
  setsid nohup bash -c "source /opt/ros/lyrical/setup.bash; source /home/yule/rulesync_ws/install/setup.bash; export ROS_DISCOVERY_SERVER=127.0.0.1:11811 ROS_DISABLE_ROS2_DAEMON=1; ${1:-:} ; ros2 launch rulesync_demo rulesync_sim.launch.py" > $LOG/FC_nodes.log 2>&1 < /dev/null &
  sleep 10
}

launch_topo_gate() {
  setsid nohup python3 -c "from topopatch_ros.topo_inference_node import main; main()" \
    --ros-args -p weights_npz:=$WEIGHTS -p tau:=$TAU_SCOPE -p use_scope:=True \
    > $LOG/FC_topo.log 2>&1 < /dev/null &
  setsid nohup python3 -c "from topopatch_ros.gate_node import main; main()" \
    > $LOG/FC_gate.log 2>&1 < /dev/null &
  setsid nohup python3 -m topopatch_ros.scan_stub > $LOG/FC_scan.log 2>&1 < /dev/null &
  sleep 6
}

restart_manager_bridge() {
  # fresh epoch-0 state between faults
  for pat in '[r]ulesync_manager' '[r]ulesync_nav2_bridge' '[r]os2 launch'; do
    pkill -9 -f "$pat" 2>/dev/null || true
  done
  sleep 4
  launch_rulesync "${1:-}"
}

run_client() { # <fault> <epoch> [extra args...]
  local F=$1 EP=$2; shift 2
  cd $CK
  timeout 600 python3 -m topopatch_ros.fault_client --fault $F \
    --csv $PAPER/fault_${F}.csv --epoch $EP \
    --start=-2.0,-0.5 --goal=2.0,0.5 "$@" 2>&1 | tail -2
}

kill_all
rm -f /tmp/rulesync_state.json
fresh_ds
cd $CK
bringup_base
launch_rulesync
launch_topo_gate
echo "[fault] preflight..."
if ! timeout 150 python3 /tmp/rep_preflight.py -2.0 -0.5 2.0 0.5 2>&1 | tail -1 | grep -qE "PREFLIGHT nposes=([4-9]|[0-9]{2,})"; then
  echo "[fault] PREFLIGHT FAILED" >&2; exit 1
fi
echo "[fault] PREFLIGHT OK"

for F in $FAULTS; do
  echo "[fault] ===== $F ====="
  case $F in
    f1) F1_PROXY=1
      for pat in '[p]lanner_server' '[m]ask_delay'; do pkill -9 -f "$pat" 2>/dev/null || true; done
      sleep 3; start_planner
      restart_manager_bridge
      run_client f1 20100 --delay 20.0
      unset F1_PROXY
      for pat in '[p]lanner_server' '[m]ask_delay'; do pkill -9 -f "$pat" 2>/dev/null || true; done
      sleep 3; start_planner ;;
    f2) restart_manager_bridge; run_client f2 20200 ;;
    f3) restart_manager_bridge
      pkill -9 -f '[n]av2_bridge' 2>/dev/null; sleep 2
      run_client f3 20300
      _restart_bridge_only ;;
    f4) restart_manager_bridge
      pkill -9 -f '[p]lanner_server' 2>/dev/null; sleep 2
      run_client f4 20400
      start_planner ;;
    f5) restart_manager_bridge; run_client f5 20500 ;;
    f6) # discovery delay: stop manager, run client, start manager late
      for pat in '[r]ulesync_manager' '[r]ulesync_nav2_bridge' '[r]os2 launch'; do pkill -9 -f "$pat" 2>/dev/null || true; done
      sleep 3
      ( sleep 25; launch_rulesync ) &
      run_client f6 20600 --recover-timeout 180.0 ;;
    f7) restart_manager_bridge; run_client f7 20700 ;;
    f8) # victim manager carries the commit-pause hook
      for pat in '[r]ulesync_manager' '[r]ulesync_nav2_bridge' '[r]os2 launch'; do pkill -9 -f "$pat" 2>/dev/null || true; done
      sleep 3
      export RULESYNC_TEST_COMMIT_PAUSE_S=8
      launch_rulesync "export RULESYNC_TEST_COMMIT_PAUSE_S=8"
      unset RULESYNC_TEST_COMMIT_PAUSE_S
      run_client f8 20900 ;;
    f9) restart_manager_bridge; run_client f9 21100 ;;
  esac
done
echo "[fault] campaign done."

_restart_bridge_only() {
  setsid nohup bash -c 'source /opt/ros/lyrical/setup.bash; source /home/yule/rulesync_ws/install/setup.bash; export ROS_DISCOVERY_SERVER=127.0.0.1:11811 ROS_DISABLE_ROS2_DAEMON=1; ros2 run rulesync_nav2_bridge bridge' > $LOG/FC_bridge2.log 2>&1 < /dev/null &
  sleep 8
}
