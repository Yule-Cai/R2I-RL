#!/bin/bash
# Rerun fault F1 only (delayed mask proxy + remapped planner).
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
SANDBOX_YAML=/opt/ros/lyrical/share/nav2_bringup/maps/tb3_sandbox.yaml
export PYTHONPATH=/home/yule/R2I-RL-checkout:/home/yule/R2I-RL-checkout/rulepatch:/home/yule/R2I-RL-checkout/topopatch_ros:${PYTHONPATH:-}
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export ROS_DISABLE_ROS2_DAEMON=1
for pat in '[p]lanner_server' '[m]ap_server' '[r]ulesync_manager' '[r]ulesync_nav2_bridge' '[t]opo_inference' '[g]ate_node' '[f]ault_client' '[m]ask_delay'; do
  pkill -9 -f "$pat" 2>/dev/null || true
done
sleep 4
for _ in $(seq 1 6); do
  left=$(pgrep -f '[r]ulesync_manager|[n]av2_bridge|[m]ap_server|[p]lanner_server' | head)
  [ -z "$left" ] && break
  echo "[kill] survivors: $left, retrying..."
  sleep 2
  for pat in '[r]ulesync_manager' '[n]av2_bridge' '[m]ap_server' '[p]lanner_server'; do
    pkill -9 -f "$pat" 2>/dev/null || true
  done
done
pkill -9 -f '[f]astdds(\.py)? discovery|[f]ast-discovery-server' 2>/dev/null || true
sleep 2
bash -c 'source /opt/ros/lyrical/setup.bash && setsid nohup fastdds discovery -i 0 -l 127.0.0.1 -p 11811 > /tmp/ds_f1.log 2>&1 < /dev/null & echo bg' >/dev/null 2>&1 || true
sleep 3
ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom \
  --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 > $LOG/F1b_tf.log 2>&1 &
ros2 run tf2_ros static_transform_publisher --frame-id odom --child-frame-id base_link \
  --x -2.0 --y -0.5 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 >> $LOG/F1b_tf.log 2>&1 &
cat > /tmp/f1b_map.yaml <<EOF
/**:
  ros__parameters:
    yaml_filename: "$SANDBOX_YAML"
    topic_name: "map"
    frame_id: "map"
EOF
ros2 run nav2_map_server map_server --ros-args -p use_sim_time:=False \
  --params-file /tmp/f1b_map.yaml > $LOG/F1b_map.log 2>&1 &
python3 $CK/topopatch/tools/paper_planner_cfg.py --pgm-yaml $SANDBOX_YAML \
  --plugins GridBased --out /tmp/f1b_planner.yaml
ros2 run nav2_planner planner_server --ros-args -p use_sim_time:=False \
  --params-file /tmp/f1b_planner.yaml -r /rulesync/keepout_mask:=/mask_delayed \
  > $LOG/F1b_planner.log 2>&1 &
sleep 8
python3 $CK/rulepatch/tools/activate_nav2.py --timeout 100 \
  --nodes map_server,planner_server > $LOG/F1b_act.log 2>&1 || true
ros2 launch rulesync_demo rulesync_sim.launch.py > $LOG/F1b_nodes.log 2>&1 &
sleep 8
python3 -c "from topopatch_ros.topo_inference_node import main; main()" \
  --ros-args -p weights_npz:=$WEIGHTS -p tau:=$TAU_SCOPE -p use_scope:=True \
  > $LOG/F1b_topo.log 2>&1 &
python3 -c "from topopatch_ros.gate_node import main; main()" \
  > $LOG/F1b_gate.log 2>&1 &
setsid nohup python3 -m topopatch_ros.scan_stub > $LOG/F1b_scan.log 2>&1 < /dev/null &
sleep 3
python3 -m topopatch_ros.mask_delay --delay 20.0 --out-topic /mask_delayed \
  > $LOG/F1b_proxy.log 2>&1 &
sleep 2
timeout 300 python3 -m topopatch_ros.fault_client --fault f1 --csv $PAPER/fault_f1.csv \
  --epoch 431 --start=-2.0,-0.5 --goal=2.0,0.5 --delay 20.0 > $LOG/F1b.log 2>&1 || true
echo "[F1b] done."
