#!/bin/bash
# Blocker B runner: per-map headless stack + native-vs-verified baseline.
# Usage: run_native.sh <maze|narrow> [delay-secs, 0 = no proxy]
WHICH=$1
DELAY=${2:-0}
if [ "$WHICH" = "maze" ]; then
  TAG=topo_ext_maze-B-s202
  PGM=/home/yule/rulepatch/topo_maps/maze-B-s202.pgm
  STEPCASE=3
else
  TAG=topo_ext_narrow-B-s302
  PGM=/home/yule/rulepatch/topo_maps/narrow-B-s302.pgm
  STEPCASE=6
fi
CK=/home/yule/R2I-RL-checkout
PAPER=/home/yule/rulepatch/paper
LOG=$PAPER/logs
export PYTHONPATH=/home/yule/R2I-RL-checkout:/home/yule/R2I-RL-checkout/rulepatch:/home/yule/R2I-RL-checkout/topopatch_ros:${PYTHONPATH:-}
export PYTHONDONTWRITEBYTECODE=1
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export ROS_DISABLE_ROS2_DAEMON=1
set +u
source /opt/ros/lyrical/setup.bash
source /home/yule/rulesync_ws/install/setup.bash
set -u
for pat in "[p]lanner_server" "[m]ap_server" "[r]ulesync_manager" "[r]ulesync_nav2_bridge" "[t]opo_inference" "[g]ate_node" "[t]opo_case_driver" "[b]ench_client" "[f]ault_client" "[n]ative_baseline" "[r]obot_state_publisher" "[p]arameter_bridge" "[r]os2 launch" "[r]os2 run" "[s]tatic_transform_publisher" "[s]can_stub" "[m]ask_delay"; do
  pkill -9 -f "$pat" 2>/dev/null || true
done
pkill -9 -f "[f]astdds(.py)? discovery|[f]ast-discovery-server" 2>/dev/null || true
sleep 4
rm -f /dev/shm/fastdds_* /dev/shm/sem.fastdds_* 2>/dev/null || true
bash -c 'source /opt/ros/lyrical/setup.bash && setsid nohup fastdds discovery -i 0 -l 127.0.0.1 -p 11811 > /tmp/ds_native.log 2>&1 < /dev/null & echo bg' >/dev/null 2>&1 || true
sleep 5
cd $CK
python3 topopatch/tools/paper_planner_cfg.py --pgm-yaml ${PGM%.pgm}.yaml \
  --plugins GridBased,Smac2D,ThetaStar --out /tmp/native_planner_$TAG.yaml
SX=$(python3 -c "import json;print(json.load(open('/home/yule/rulepatch/repairable_manifest.json'))['cases'][0]['start'][0])")
SY=$(python3 -c "import json;print(json.load(open('/home/yule/rulepatch/repairable_manifest.json'))['cases'][0]['start'][1])")
setsid nohup ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom \
  --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 > $LOG/NAT_${TAG}_tf.log 2>&1 < /dev/null &
setsid nohup ros2 run tf2_ros static_transform_publisher --frame-id odom --child-frame-id base_link \
  --x $SX --y $SY --z 0 --qx 0 --qy 0 --qz 0 --qw 1 >> $LOG/NAT_${TAG}_tf.log 2>&1 < /dev/null &
setsid nohup ros2 run nav2_map_server map_server --ros-args \
  --params-file /tmp/native_planner_$TAG.yaml > $LOG/NAT_${TAG}_map.log 2>&1 < /dev/null &
if [ "$DELAY" != "0" ]; then
  setsid nohup ros2 run nav2_planner planner_server --ros-args \
    --params-file /tmp/native_planner_$TAG.yaml -r /rulesync/keepout_mask:=/mask_delayed \
    > $LOG/NAT_${TAG}_planner.log 2>&1 < /dev/null &
  setsid nohup python3 -m topopatch_ros.mask_delay --delay $DELAY --out-topic /mask_delayed \
    > $LOG/NAT_${TAG}_proxy.log 2>&1 < /dev/null &
else
  setsid nohup ros2 run nav2_planner planner_server --ros-args \
    --params-file /tmp/native_planner_$TAG.yaml > $LOG/NAT_${TAG}_planner.log 2>&1 < /dev/null &
fi
sleep 8
timeout 120 python3 rulepatch/tools/activate_nav2.py --timeout 100 \
  --nodes map_server,planner_server > $LOG/NAT_${TAG}_act.log 2>&1 || true
# Benchmark stacks disable bridge auto-replan/rollback (live-demo
# behaviors): the bridge unilaterally removing rules breaks per-case
# independence and races the verified removal protocol.
disable_bridge_auto() {
  for i in 1 2 3 4 5 6; do
    if python3 $CK/rulepatch/tools/rp_params.py rulesync_nav2_bridge auto_replan false 2>&1 | grep -q True; then break; fi
    sleep 10
  done
  for i in 1 2 3 4 5 6; do
    if python3 $CK/rulepatch/tools/rp_params.py rulesync_nav2_bridge auto_rollback false 2>&1 | grep -q True; then break; fi
    sleep 10
  done
}
setsid nohup bash -c 'source /opt/ros/lyrical/setup.bash; source ~/rulesync_ws/install/setup.bash; export ROS_DISCOVERY_SERVER=127.0.0.1:11811 ROS_DISABLE_ROS2_DAEMON=1; ros2 launch rulesync_demo rulesync_sim.launch.py' > $LOG/NAT_${TAG}_nodes.log 2>&1 < /dev/null &
sleep 10
disable_bridge_auto
setsid nohup python3 -m topopatch_ros.scan_stub > $LOG/NAT_${TAG}_scan.log 2>&1 < /dev/null &
sleep 4
python3 -c "import json;m=json.load(open('/home/yule/rulepatch/repairable_manifest.json'));cs=[c for c in m['cases'] if c['map_tag']=='$TAG'][::$STEPCASE][:30];json.dump({'cases':cs},open('/tmp/native_cases_$TAG.json','w'));print('cases:',len(cs))"
echo "[native] preflight..."
PF0=$(python3 -c "import json;d=json.load(open('/tmp/native_cases_$TAG.json'));c=d['cases'][0];print(c['start'][0],c['start'][1],c['goal'][0],c['goal'][1])")
if ! timeout 150 python3 /tmp/rep_preflight.py $PF0 2>&1 | tail -1 | grep -qE "PREFLIGHT nposes=([4-9]|[0-9]{2,})"; then
  echo "[native] PREFLIGHT FAILED" >&2; exit 1
fi
echo "[native] PREFLIGHT OK -- running $TAG delay=$DELAY"
SUFFIX=""
if [ "$DELAY" != "0" ]; then SUFFIX="_dly"; fi
PLI=0
for PL in GridBased Smac2D ThetaStar; do
  PLI=$((PLI + 1))
  for ATTEMPT in 1 2; do
    echo "[native] arm $PL attempt $ATTEMPT (manager restart for epoch isolation)"
    for pat in '[r]ulesync_manager' '[r]ulesync_nav2_bridge' '[r]os2 launch'; do
      pkill -9 -f "$pat" 2>/dev/null || true
    done
    sleep 4
    rm -f /tmp/rulesync_state.json
    setsid nohup bash -c 'source /opt/ros/lyrical/setup.bash; source ~/rulesync_ws/install/setup.bash; export ROS_DISCOVERY_SERVER=127.0.0.1:11811 ROS_DISABLE_ROS2_DAEMON=1; ros2 launch rulesync_demo rulesync_sim.launch.py' > $LOG/NAT_${TAG}_nodes_$PL.log 2>&1 < /dev/null &
    sleep 12
    disable_bridge_auto
    timeout 5400 python3 -m topopatch_ros.native_baseline \
      --csv $PAPER/native_${TAG}_${PL}${SUFFIX}.csv --planners $PL \
      --map $TAG --cases-file /tmp/native_cases_$TAG.json \
      > $LOG/NAT_${TAG}_bench_$PL.log 2>&1 || echo "[native] $PL exit: $?"
    NROWS=$(python3 -c "import csv;print(len(list(csv.DictReader(open('$PAPER/native_${TAG}_${PL}${SUFFIX}.csv')))))" 2>/dev/null || echo 0)
    echo "[native] arm $PL attempt $ATTEMPT rows=$NROWS"
    if [ "$NROWS" -ge 29 ]; then break; fi
    echo "[native] arm $PL incomplete ($NROWS) -- retrying once"
  done
done
echo "[native] $TAG done."
