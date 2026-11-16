#!/bin/bash
# EXP4: recovery-timing — after kill -9 + restart (matrix recovery shape),
# when do processes exist AND all 3 services become discoverable?
# Samples every 10 s for 6 min: pgrep counts + disco probe (1 trial).
# All logic in-file (no ssh-cmdline self-match).
source /opt/ros/lyrical/setup.bash
source /home/yule/rulesync_ws/install/setup.bash
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export ROS_DISABLE_ROS2_DAEMON=1
export PYTHONPATH=/home/yule/R2I-RL-checkout:/home/yule/R2I-RL-checkout/rulepatch:/home/yule/R2I-RL-checkout/topopatch_ros:$PYTHONPATH
export PYTHONDONTWRITEBYTECODE=1
CK=/home/yule/R2I-RL-checkout
OUT=/home/yule/rulepatch/3d/correctness_revision
LOG=$OUT/exp4.log
{
echo "===== exp4 start $(date)"
# fresh DS + stack
if [ -f /tmp/ds_e4.pid ]; then kill -9 "$(cat /tmp/ds_e4.pid)" 2>/dev/null || true; fi
pkill -9 -f '[f]ast-discovery-server' 2>/dev/null || true
pkill -9 -f '[f]astdds discovery' 2>/dev/null || true
sleep 2
rm -f /dev/shm/fastdds_* /dev/shm/sem.fastdds_* 2>/dev/null || true
setsid nohup fastdds discovery -i 0 -l 127.0.0.1 -p 11811 > /tmp/ds_e4.log 2>&1 < /dev/null &
sleep 2
pgrep -nf '[f]astdds discovery' > /tmp/ds_e4.pid 2>/dev/null || true
sleep 2
for pat in '[r]ulesync_manager' '[r]ulesync_nav2_bridge' '[u]av_bridge'; do pkill -9 -f "$pat" 2>/dev/null || true; done
sleep 3
rm -f /tmp/rulesync_state.json /tmp/rulesync_txn.log /tmp/rulesync_wal.jsonl
setsid nohup bash -c 'source /opt/ros/lyrical/setup.bash; source /home/yule/rulesync_ws/install/setup.bash; export ROS_DISCOVERY_SERVER=127.0.0.1:11811 ROS_DISABLE_ROS2_DAEMON=1; ros2 launch rulesync_demo rulesync_sim.launch.py' > $OUT/mgr_e4.log 2>&1 < /dev/null &
for _ in $(seq 1 18); do
  sleep 10
  if grep -a -q "SYSTEM_READY (reconciled)" $OUT/mgr_e4.log 2>/dev/null; then break; fi
done
python3 -c "from rulesync3d.rulesync3d.worlds_3d import WORLDS; import json; ab=[]; [ab.append((c[0]-s[0]/2,c[1]-s[1]/2,c[2]-s[2]/2,c[0]+s[0]/2,c[1]+s[1]/2,c[2]+s[2]/2)) for (c,s) in [((b[0],b[1],b[2]),(b[3],b[4],b[5])) for b in WORLDS['open_3d']()]]; open('/tmp/boxes_e4.json','w').write(json.dumps(ab))"
BOXES=$(cat /tmp/boxes_e4.json)
setsid nohup python3 -m topopatch_ros.uav_bridge_node --ros-args -p world_boxes:="'$BOXES'" -p spawn_xyz:="[1.0,10.0,1.0]" > $OUT/ubridge_e4.log 2>&1 < /dev/null &
sleep 15
echo "--- make non-empty state: commit V0 ---"
timeout 200 python3 $CK/rulesync3d/tools/proto_op.py --uid e4setup --op ADD --epoch 500000 --out /tmp/e4_rows.jsonl --expect /tmp/e4_expect.json --phase both
echo "--- simulate RECOVERY: kill -9 both, restart both (matrix shape) ---"
for pat in '[r]ulesync_manager' '[r]ulesync_nav2_bridge' '[u]av_bridge'; do pkill -9 -f "$pat" 2>/dev/null || true; done
sleep 3
setsid nohup bash -c 'source /opt/ros/lyrical/setup.bash; source /home/yule/rulesync_ws/install/setup.bash; export ROS_DISCOVERY_SERVER=127.0.0.1:11811 ROS_DISABLE_ROS2_DAEMON=1; ros2 launch rulesync_demo rulesync_sim.launch.py' > $OUT/mgr_e4b.log 2>&1 < /dev/null &
setsid nohup python3 -m topopatch_ros.uav_bridge_node --ros-args -p world_boxes:="'$BOXES'" -p spawn_xyz:="[1.0,10.0,1.0]" > $OUT/ubridge_e4b.log 2>&1 < /dev/null &
echo "--- sample procs + discovery every 15 s for 6 min ---"
for k in $(seq 1 24); do
  M=$(pgrep -fc '[r]ulesync_manager/lib' || true)
  U=$(pgrep -fc '[u]av_bridge_node' || true)
  N2=$(pgrep -fc '[r]ulesync_nav2_bridge/lib' || true)
  R=$(timeout 100 python3 $CK/rulesync3d/tools/disco_probe.py 1 2>&1 | tr '\n' ' ')
  echo "t+$((k*15))s mgr=$M uav=$U nav2=$N2 $R"
  sleep 15
done
echo "===== exp4 done $(date)"
} >> $LOG 2>&1
