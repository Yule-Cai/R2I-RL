#!/bin/bash
# Verified-clean single-fault runner (paper F-reruns).
# Kills ROS user processes, VERIFIES zero survivors (manager/bridge/
# planner/map/topo/gate/fault/bench), then brings up the F-stack and
# runs ONE fault client. Usage: run_fault_clean.sh <f1|f2|f3|f4> <epoch>
F="$1"
EP="$2"
export PYTHONDONTWRITEBYTECODE=1
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export ROS_DISABLE_ROS2_DAEMON=1
set +u
source /opt/ros/lyrical/setup.bash
source ~/rulesync_ws/install/setup.bash
set -u
export PYTHONPATH=/home/yule/R2I-RL-checkout:/home/yule/R2I-RL-checkout/rulepatch:/home/yule/R2I-RL-checkout/topopatch_ros:${PYTHONPATH:-}
CK=~/R2I-RL-checkout
PAPER=~/rulepatch/paper
LOG=$PAPER/logs
WEIGHTS=~/rulepatch/topo_scope/best_fused.npz
TAU_SCOPE=1.8216713190078737
SANDBOX_YAML=/opt/ros/lyrical/share/nav2_bringup/maps/tb3_sandbox.yaml
for i in 1 2 3; do
  pkill -9 -f '[r]ulesync_manager|[n]av2_bridge|[m]ap_server|[p]lanner_server|[t]opo_inference|[g]ate_node|[f]ault_client|[b]ench_txn|[m]ask_delay|[s]can_stub|[e]ndurance' 2>/dev/null || true
  sleep 2
  N=$(ps -eo args | grep -E '[r]ulesync_manager|[n]av2_bridge|[m]ap_server|[p]lanner_server' | wc -l)
  echo "[clean] pass$i survivors=$N"
  [ "$N" -eq 0 ] && break
  sleep 2
done
bash -c 'source /opt/ros/lyrical/setup.bash && setsid nohup fastdds discovery -i 0 -l 127.0.0.1 -p 11811 > /tmp/ds_fclean.log 2>&1 < /dev/null & echo bg' >/dev/null 2>&1 || true
sleep 2
cd $CK
setsid nohup ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom \
  --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 > $LOG/FC_tf.log 2>&1 < /dev/null &
setsid nohup ros2 run tf2_ros static_transform_publisher --frame-id odom --child-frame-id base_link \
  --x -2 --y -0.5 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 >> $LOG/FC_tf.log 2>&1 < /dev/null &
cat > /tmp/fc_map.yaml <<EOF
/**:
  ros__parameters:
    yaml_filename: "$SANDBOX_YAML"
    topic_name: "map"
    frame_id: "map"
EOF
setsid nohup ros2 run nav2_map_server map_server --ros-args -p use_sim_time:=False \
  --params-file /tmp/fc_map.yaml > $LOG/FC_map.log 2>&1 < /dev/null &
python3 $CK/topopatch/tools/paper_planner_cfg.py --pgm-yaml $SANDBOX_YAML \
  --plugins GridBased --out /tmp/fc_planner.yaml
if [ "$F" = "f1" ]; then
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
timeout 90 python3 rulepatch/tools/activate_nav2.py --timeout 80 \
  --nodes map_server,planner_server 2>&1 | tail -1
setsid nohup bash -c 'source /opt/ros/lyrical/setup.bash; source ~/rulesync_ws/install/setup.bash; export ROS_DISCOVERY_SERVER=127.0.0.1:11811 ROS_DISABLE_ROS2_DAEMON=1; ros2 launch rulesync_demo rulesync_sim.launch.py' > $LOG/FC_nodes.log 2>&1 < /dev/null &
sleep 8
setsid nohup python3 -c "from topopatch_ros.topo_inference_node import main; main()" \
  --ros-args -p weights_npz:=$WEIGHTS -p tau:=$TAU_SCOPE -p use_scope:=True \
  > $LOG/FC_topo.log 2>&1 < /dev/null &
setsid nohup python3 -c "from topopatch_ros.gate_node import main; main()" \
  > $LOG/FC_gate.log 2>&1 < /dev/null &
setsid nohup python3 -m topopatch_ros.scan_stub > $LOG/FC_scan.log 2>&1 < /dev/null &
sleep 3
case $F in
  f1) timeout 300 python3 -m topopatch_ros.fault_client --fault f1 --csv $PAPER/fault_f1.csv \
        --epoch $EP --start=-2.0,-0.5 --goal=2.0,0.5 --delay 20.0 2>&1 | tail -1 ;;
  f2) timeout 200 python3 -m topopatch_ros.fault_client --fault f2 --csv $PAPER/fault_f2.csv \
        --epoch $EP 2>&1 | tail -1 ;;
  f3) pkill -9 -f '[n]av2_bridge' 2>/dev/null; sleep 2;
      timeout 200 python3 -m topopatch_ros.fault_client --fault f3 --csv $PAPER/fault_f3.csv \
        --epoch $EP 2>&1 | tail -1 ;;
  f4) pkill -9 -f '[p]lanner_server' 2>/dev/null; sleep 2;
      timeout 200 python3 -m topopatch_ros.fault_client --fault f4 --csv $PAPER/fault_f4.csv \
        --epoch $EP --start=-2.0,-0.5 --goal=2.0,0.5 2>&1 | tail -1 ;;
esac
echo "[FC] $F done."
