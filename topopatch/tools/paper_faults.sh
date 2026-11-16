#!/bin/bash
# Paper fault injection F1..F6 (sandbox headless; called by paper_sim.sh).
# Each fault asserts fail-closed (no motion) + measures latencies to CSV.
# Stack per fault is (re)built deterministically; see fault_client.py.
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
pkill -9 -f '[f]astdds(\.py)? discovery|[f]ast-discovery-server' 2>/dev/null || true
  sleep 2
    bash -c 'source /opt/ros/lyrical/setup.bash && setsid nohup fastdds discovery -i 0 -l 127.0.0.1 -p 11811 > /tmp/ds_paper.log 2>&1 < /dev/null & echo bg' >/dev/null 2>&1 || true
  sleep 3
PAPER=~/rulepatch/paper
LOG=$PAPER/logs
WEIGHTS=~/rulepatch/topo_scope/best_fused.npz
TAU_SCOPE=1.8216713190078737
SANDBOX_YAML=/opt/ros/lyrical/share/nav2_bringup/maps/tb3_sandbox.yaml

kill_all() {
  for pat in '[g]z-sim-main' '[g]z sim' '[n]av2_container' '[p]lanner_server' \
             '[m]ap_server' '[c]ontroller_server' '[r]obot_state_publisher' \
             '[p]arameter_bridge' '[r]os2 launch' '[t]opo_inference' \
             '[g]ate_node' '[p]p_driver' '[t]opo_case_driver' '[b]ench_client' \
             '[f]ault_client' '[m]ask_delay' '[e]ndurance' '[x]vfb-run' \
             '[r]ulesync_manager' '[r]ulesync_nav2_bridge'; do
    pkill -9 -f "$pat" 2>/dev/null || true
  done
  sleep 4
}

base_stack() { # [extra_planner_args...]: map+TF+planner+manager+bridge+topo+gate
  kill_all
  ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom \
    --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 > $LOG/F_tf.log 2>&1 &
  ros2 run tf2_ros static_transform_publisher --frame-id odom --child-frame-id base_link \
    --x -2.0 --y -0.5 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 >> $LOG/F_tf.log 2>&1 &
  cat > /tmp/paper_map_sb.yaml <<EOF
/**:
  ros__parameters:
    yaml_filename: "$SANDBOX_YAML"
    topic_name: "map"
    frame_id: "map"
EOF
  ros2 run nav2_map_server map_server --ros-args -p use_sim_time:=False \
    --params-file /tmp/paper_map_sb.yaml > $LOG/F_map.log 2>&1 &
  python3 $CK/topopatch/tools/paper_planner_cfg.py --pgm-yaml $SANDBOX_YAML \
    --plugins GridBased --out /tmp/paper_planner_sb.yaml
  ros2 run nav2_planner planner_server --ros-args -p use_sim_time:=False \
    --params-file /tmp/paper_planner_sb.yaml "$@" > $LOG/F_planner.log 2>&1 &
  sleep 8
  python3 $CK/rulepatch/tools/activate_nav2.py --timeout 100 \
    --nodes map_server,planner_server > $LOG/F_act.log 2>&1 || true
  ros2 launch rulesync_demo rulesync_sim.launch.py > $LOG/F_nodes.log 2>&1 &
  sleep 8
  python3 -c "from topopatch_ros.topo_inference_node import main; main()" \
    --ros-args -p weights_npz:=$WEIGHTS -p tau:=$TAU_SCOPE -p use_scope:=True \
    > $LOG/F_topo.log 2>&1 &
  python3 -c "from topopatch_ros.gate_node import main; main()" \
    > $LOG/F_gate.log 2>&1 &
  setsid nohup python3 -m topopatch_ros.scan_stub > $LOG/F_scan.log 2>&1 < /dev/null &
  python3 -c "from rulepatch_ros.pp_driver import main; main()" --ros-args \
    -r /rulepatch/repaired_path:=/rulepatch/gated_path -p speed:=0.25 \
    > $LOG/F_pp.log 2>&1 &
  sleep 5
}

echo "[F] F1 delayed mask (planner remapped to delayed relay)..."
base_stack -r /rulesync/keepout_mask:=/mask_delayed
python3 -m topopatch_ros.mask_delay --delay 5.0 --out-topic /mask_delayed \
  > $LOG/F1_proxy.log 2>&1 &
sleep 2
timeout 300 python3 -m topopatch_ros.fault_client --fault f1 --csv $PAPER/fault_f1.csv \
  --epoch 401 --start -2.0,-0.5 --goal 2.0,0.5 --delay 5.0 > $LOG/F1.log 2>&1 || true

echo "[F] F2 stale epoch..."
base_stack
timeout 300 python3 -m topopatch_ros.fault_client --fault f2 --csv $PAPER/fault_f2.csv \
  --epoch 402 > $LOG/F2.log 2>&1 || true

echo "[F] F3 missing ACK (bridge killed)..."
base_stack
pkill -9 -f '[r]ulesync_nav2_bridge/bridge' 2>/dev/null || true
sleep 3
timeout 300 python3 -m topopatch_ros.fault_client --fault f3 --csv $PAPER/fault_f3.csv \
  --epoch 403 > $LOG/F3.log 2>&1 || true

echo "[F] F4 planner down..."
base_stack
pkill -9 -f 'nav2_planner/planner_server' 2>/dev/null || true
sleep 3
timeout 300 python3 -m topopatch_ros.fault_client --fault f4 --csv $PAPER/fault_f4.csv \
  --epoch 404 --start -2.0,-0.5 --goal 2.0,0.5 > $LOG/F4.log 2>&1 || true

echo "[F] F5 invalid fallback (gate_node enforced, pp on gated)..."
base_stack
timeout 300 python3 -m topopatch_ros.fault_client --fault f5 --csv $PAPER/fault_f5.csv \
  --epoch 405 > $LOG/F5.log 2>&1 || true

echo "[F] F6 discovery delay (manager starts 25s late)..."
kill_all
ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom \
  --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 > $LOG/F6_tf.log 2>&1 &
ros2 run tf2_ros static_transform_publisher --frame-id odom --child-frame-id base_link \
  --x -2.0 --y -0.5 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 >> $LOG/F6_tf.log 2>&1 &
cat > /tmp/paper_map_sb.yaml <<EOF
/**:
  ros__parameters:
    yaml_filename: "$SANDBOX_YAML"
    topic_name: "map"
    frame_id: "map"
EOF
ros2 run nav2_map_server map_server --ros-args -p use_sim_time:=False \
  --params-file /tmp/paper_map_sb.yaml > $LOG/F6_map.log 2>&1 &
python3 $CK/topopatch/tools/paper_planner_cfg.py --pgm-yaml $SANDBOX_YAML \
  --plugins GridBased --out /tmp/paper_planner_sb.yaml
ros2 run nav2_planner planner_server --ros-args -p use_sim_time:=False \
  --params-file /tmp/paper_planner_sb.yaml > $LOG/F6_planner.log 2>&1 &
sleep 8
python3 $CK/rulepatch/tools/activate_nav2.py --timeout 100 \
  --nodes map_server,planner_server > $LOG/F6_act.log 2>&1 || true
python3 -c "from topopatch_ros.topo_inference_node import main; main()" \
  --ros-args -p weights_npz:=$WEIGHTS -p tau:=$TAU_SCOPE -p use_scope:=True \
  > $LOG/F6_topo.log 2>&1 &
python3 -c "from topopatch_ros.gate_node import main; main()" \
  > $LOG/F6_gate.log 2>&1 &
sleep 3
timeout 400 python3 -m topopatch_ros.fault_client --fault f6 --csv $PAPER/fault_f6.csv \
  --epoch 406 --recover-timeout 240 > $LOG/F6.log 2>&1 &
F6PID=$!
sleep 25
ros2 launch rulesync_demo rulesync_sim.launch.py > $LOG/F6_nodes.log 2>&1 &
wait $F6PID || true
kill_all
echo "[F] faults done."
