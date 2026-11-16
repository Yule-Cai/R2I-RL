#!/bin/bash
# Churn experiment: does kill/restart churn degrade fresh-client
# discovery? Steady baseline vs post-churn timings for the same 3
# services. All logic in-file (no ssh-cmdline self-match).
source /opt/ros/lyrical/setup.bash
source /home/yule/rulesync_ws/install/setup.bash
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export ROS_DISABLE_ROS2_DAEMON=1
export PYTHONPATH=/home/yule/R2I-RL-checkout:/home/yule/R2I-RL-checkout/rulepatch:/home/yule/R2I-RL-checkout/topopatch_ros:$PYTHONPATH
export PYTHONDONTWRITEBYTECODE=1
CK=/home/yule/R2I-RL-checkout
OUT=/home/yule/rulepatch/3d/correctness_revision
LOG=$OUT/churn_exp.log
{
echo "===== churn exp start $(date)"
if [ -f /tmp/ds_churn.pid ]; then kill -9 "$(cat /tmp/ds_churn.pid)" 2>/dev/null || true; fi
pkill -9 -f '[f]ast-discovery-server' 2>/dev/null || true
pkill -9 -f '[f]astdds discovery' 2>/dev/null || true
sleep 2
rm -f /dev/shm/fastdds_* /dev/shm/sem.fastdds_* 2>/dev/null || true
setsid nohup fastdds discovery -i 0 -l 127.0.0.1 -p 11811 > /tmp/ds_churn.log 2>&1 < /dev/null &
sleep 2
pgrep -nf '[f]astdds discovery' > /tmp/ds_churn.pid 2>/dev/null || true
sleep 2
for pat in '[r]ulesync_manager' '[r]ulesync_nav2_bridge' '[u]av_bridge'; do pkill -9 -f "$pat" 2>/dev/null || true; done
sleep 3
rm -f /tmp/rulesync_state.json /tmp/rulesync_txn.log /tmp/rulesync_wal.jsonl
setsid nohup bash -c 'source /opt/ros/lyrical/setup.bash; source /home/yule/rulesync_ws/install/setup.bash; export ROS_DISCOVERY_SERVER=127.0.0.1:11811 ROS_DISABLE_ROS2_DAEMON=1; ros2 launch rulesync_demo rulesync_sim.launch.py' > $OUT/mgr_churn.log 2>&1 < /dev/null &
for _ in $(seq 1 18); do
  sleep 10
  if grep -a -q "SYSTEM_READY (reconciled)" $OUT/mgr_churn.log 2>/dev/null; then break; fi
done
echo "stack ready (or timeout)"
python3 -c "from rulesync3d.rulesync3d.worlds_3d import WORLDS; import json; ab=[]; [ab.append((c[0]-s[0]/2,c[1]-s[1]/2,c[2]-s[2]/2,c[0]+s[0]/2,c[1]+s[1]/2,c[2]+s[2]/2)) for (c,s) in [((b[0],b[1],b[2]),(b[3],b[4],b[5])) for b in WORLDS['open_3d']()]]; open('/tmp/boxes_churn.json','w').write(json.dumps(ab))"
BOXES=$(cat /tmp/boxes_churn.json)
setsid nohup python3 -m topopatch_ros.uav_bridge_node --ros-args -p world_boxes:="'$BOXES'" -p spawn_xyz:="[1.0,10.0,1.0]" > $OUT/ubridge_churn.log 2>&1 < /dev/null &
sleep 15
if pgrep -f '[u]av_bridge_node' > /dev/null; then echo "bridge ALIVE"; else echo "bridge MISSING (abort)"; exit 1; fi
echo "--- BASELINE steady (3 probes) ---"
for k in 1 2 3; do timeout 200 python3 $CK/rulesync3d/tools/disco_probe.py 2; done
for round in 1 2; do
  echo "--- CHURN round $round: kill+restart manager+uav ---"
  for pat in '[r]ulesync_manager' '[u]av_bridge'; do pkill -9 -f "$pat" 2>/dev/null || true; done
  sleep 3
  setsid nohup bash -c 'source /opt/ros/lyrical/setup.bash; source /home/yule/rulesync_ws/install/setup.bash; export ROS_DISCOVERY_SERVER=127.0.0.1:11811 ROS_DISABLE_ROS2_DAEMON=1; ros2 run rulesync_manager manager --ros-args -p txn_log_path:=/tmp/rulesync_txn.log' > $OUT/mgr_churn2.log 2>&1 < /dev/null &
  setsid nohup python3 -m topopatch_ros.uav_bridge_node --ros-args -p world_boxes:="'$BOXES'" -p spawn_xyz:="[1.0,10.0,1.0]" > $OUT/ubridge_churn2.log 2>&1 < /dev/null &
  sleep 20
  if pgrep -f '[r]ulesync_manager/lib' > /dev/null; then echo "mgr ALIVE"; else echo "mgr MISSING"; fi
  if pgrep -f '[u]av_bridge_node' > /dev/null; then echo "bridge ALIVE"; else echo "bridge MISSING"; fi
  for k in 1 2 3; do timeout 200 python3 $CK/rulesync3d/tools/disco_probe.py 2; done
done
echo "===== churn exp done $(date)"
} >> $LOG 2>&1
