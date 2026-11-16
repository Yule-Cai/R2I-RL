#!/bin/bash
# One-map headless bench (resumable unit for the paper matrix).
# Usage: narrow_only.sh <map>  (map: warehouse|office-A-s101|maze-A-s201|narrow-A-s301)
MAP="$1"
CK=~/R2I-RL-checkout
PAPER=~/rulepatch/paper
LOG=$PAPER/logs
WEIGHTS=~/rulepatch/topo_scope/best_fused.npz
TAU_SCOPE=1.8216713190078737
export ROS_DOMAIN_ID=7
set +u
source /opt/ros/lyrical/setup.bash
source ~/rulesync_ws/install/setup.bash
set -u
export PYTHONPATH=/home/yule/R2I-RL-checkout:/home/yule/R2I-RL-checkout/rulepatch:/home/yule/R2I-RL-checkout/topopatch_ros:${PYTHONPATH:-}
export PYTHONDONTWRITEBYTECODE=1
PLUGINS=$(cat $PAPER/planners_available.txt)
cd $CK
  # M2 pattern: manager vote-helper hardcodes ROS_DISCOVERY_SERVER=127.0.0.1:11811
  # on the DEFAULT domain; headless stacks must provide it (else every
  # commit aborts with no-service votes).
pkill -9 -f '[f]astdds(\.py)? discovery|[f]ast-discovery-server' 2>/dev/null || true
  sleep 2
    bash -c 'source /opt/ros/lyrical/setup.bash && setsid nohup fastdds discovery -i 0 -l 127.0.0.1 -p 11811 > /tmp/ds_paper.log 2>&1 < /dev/null & echo bg' >/dev/null 2>&1 || true
  sleep 3
for pat in '[m]ap_server' '[p]lanner_server' '[t]opo_inference' '[b]ench_client'; do
  pkill -9 -f "$pat" 2>/dev/null || true
done
sleep 4
START=$(python3 -c "import json;print(','.join(map(str,json.load(open('$PAPER/goals_$MAP.json'))['start'])))")
SX=${START%%,*}; SY=${START##*,}
setsid nohup ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom \
  --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 > $LOG/P1_${MAP}_tf.log 2>&1 < /dev/null &
setsid nohup ros2 run tf2_ros static_transform_publisher --frame-id odom --child-frame-id base_link \
  --x $SX --y $SY --z 0 --qx 0 --qy 0 --qz 0 --qw 1 >> $LOG/P1_${MAP}_tf.log 2>&1 < /dev/null &
setsid nohup ros2 run nav2_map_server map_server --ros-args \
  --params-file /tmp/paper_planner_$MAP.yaml > $LOG/P1_${MAP}_map.log 2>&1 < /dev/null &
setsid nohup ros2 run nav2_planner planner_server --ros-args \
  --params-file /tmp/paper_planner_$MAP.yaml > $LOG/P1_${MAP}_planner.log 2>&1 < /dev/null &
sleep 10
timeout 120 python3 rulepatch/tools/activate_nav2.py --timeout 100 \
  --nodes map_server,planner_server > $LOG/P1_${MAP}_act.log 2>&1 || true
setsid nohup bash -c 'ros2 launch rulesync_demo rulesync_sim.launch.py' > $LOG/P1_${MAP}_nodes.log 2>&1 < /dev/null &
sleep 8
setsid nohup python3 -c "from topopatch_ros.topo_inference_node import main; main()" \
  --ros-args -p weights_npz:=$WEIGHTS -p tau:=$TAU_SCOPE -p use_scope:=True \
  > $LOG/P1_${MAP}_topo.log 2>&1 < /dev/null &
  setsid nohup python3 -m topopatch_ros.scan_stub > $LOG/P1_${MAP}_scan.log 2>&1 < /dev/null &
  sleep 2
pli=0
for pl in $(echo $PLUGINS | tr ',' ' '); do
  pli=$((pli + 1))
  for nr in 1 2 3; do
    python3 -m topopatch_ros.bench_client --csv $PAPER/bench_${MAP}_${pl}_r${nr}.csv \
      --planner $pl --map $MAP --start $START --goals $PAPER/goals_$MAP.json \
      --epoch0 $((900 + pli * 100 + nr * 10)) --n-rules $nr > $LOG/P1_${MAP}_${pl}_r${nr}.log 2>&1 || true
  done
done
echo "[P1] $MAP done."
