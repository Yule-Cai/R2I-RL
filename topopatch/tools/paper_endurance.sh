#!/bin/bash
# Paper 60-min endurance (sandbox headless; called by paper_sim.sh).
# Repeats commit/remove with planning; monitors RSS, liveness, epochs,
# missed verifier calls, iteration overruns.
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
for pat in '[p]lanner_server' '[m]ap_server' '[r]ulesync_manager' \
           '[r]ulesync_nav2_bridge' '[t]opo_inference' '[g]ate_node'; do
  pkill -9 -f "$pat" 2>/dev/null || true
done
sleep 4
ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom \
  --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 > $LOG/E_tf.log 2>&1 &
ros2 run tf2_ros static_transform_publisher --frame-id odom --child-frame-id base_link \
  --x -2.0 --y -0.5 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 >> $LOG/E_tf.log 2>&1 &
cat > /tmp/paper_map_sb.yaml <<EOF
/**:
  ros__parameters:
    yaml_filename: "$SANDBOX_YAML"
    topic_name: "map"
    frame_id: "map"
EOF
ros2 run nav2_map_server map_server --ros-args -p use_sim_time:=False \
  --params-file /tmp/paper_map_sb.yaml > $LOG/E_map.log 2>&1 &
python3 $CK/topopatch/tools/paper_planner_cfg.py --pgm-yaml $SANDBOX_YAML \
  --plugins GridBased --out /tmp/paper_planner_sb.yaml
ros2 run nav2_planner planner_server --ros-args -p use_sim_time:=False \
  --params-file /tmp/paper_planner_sb.yaml > $LOG/E_planner.log 2>&1 &
sleep 8
python3 $CK/rulepatch/tools/activate_nav2.py --timeout 100 \
  --nodes map_server,planner_server > $LOG/E_act.log 2>&1 || true
ros2 launch rulesync_demo rulesync_sim.launch.py > $LOG/E_nodes.log 2>&1 &
sleep 8
python3 -c "from topopatch_ros.topo_inference_node import main; main()" \
  --ros-args -p weights_npz:=$WEIGHTS -p tau:=$TAU_SCOPE -p use_scope:=True \
  > $LOG/E_topo.log 2>&1 &
python3 -c "from topopatch_ros.gate_node import main; main()" \
  > $LOG/E_gate.log 2>&1 &
setsid nohup python3 -m topopatch_ros.scan_stub > $LOG/E_scan.log 2>&1 < /dev/null &
sleep 4
timeout 3900 python3 -m topopatch_ros.endurance --csv $PAPER/endurance.csv \
  --minutes 60 --epoch0 501 --goal 2.0,0.5 > $LOG/E_run.log 2>&1 || true
echo "[E] endurance done."
