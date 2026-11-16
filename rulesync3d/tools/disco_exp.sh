#!/bin/bash
# Controlled discovery characterization (no protocol load).
# EXP1: steady-state service discovery latency (6 samples).
# EXP2: post-manager-restart propose_volume discovery latency (10 samples).
# EXP3: post-bridge-restart bridge-state discovery latency (10 samples).
# All kills/starts inside THIS file (avoids ssh-cmdline self-match).
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export ROS_DISABLE_ROS2_DAEMON=1
export PYTHONPATH=/home/yule/R2I-RL-checkout:/home/yule/rulesync_ws/install/rulesync_interfaces/lib/python3.14/site-packages:/home/yule/rulesync_ws/install/rulesync_manager/lib/python3.14/site-packages:/home/yule/rulesync_ws/install/rulesync_nav2_bridge/lib/python3.14/site-packages:/home/yule/rulesync_ws/install/rulesync_demo/lib/python3.14/site-packages:/opt/ros/lyrical/lib/python3.14/site-packages
source /opt/ros/lyrical/setup.bash
source /home/yule/rulesync_ws/install/setup.bash
CK=/home/yule/R2I-RL-checkout
OUT=/home/yule/rulepatch/3d/correctness_revision
LOG=$OUT/disco_exp.log
{
echo "===== disco experiment start $(date)"
# fresh DS + shm
if [ -f /tmp/ds_exp.pid ]; then kill -9 "$(cat /tmp/ds_exp.pid)" 2>/dev/null || true; fi
pkill -9 -f '[f]ast-discovery-server' 2>/dev/null || true
pkill -9 -f '[f]astdds discovery' 2>/dev/null || true
sleep 2
rm -f /dev/shm/fastdds_* /dev/shm/sem.fastdds_* 2>/dev/null || true
setsid nohup fastdds discovery -i 0 -l 127.0.0.1 -p 11811 > /tmp/ds_exp_ds.log 2>&1 < /dev/null &
sleep 2
pgrep -nf '[f]astdds discovery' > /tmp/ds_exp.pid 2>/dev/null || true
sleep 2
# fresh stack (launch: manager + nav2)
for pat in '[r]ulesync_manager' '[r]ulesync_nav2_bridge' '[u]av_bridge'; do pkill -9 -f "$pat" 2>/dev/null || true; done
sleep 3
rm -f /tmp/rulesync_state.json /tmp/rulesync_txn.log /tmp/rulesync_wal.jsonl
setsid nohup bash -c 'source /opt/ros/lyrical/setup.bash; source /home/yule/rulesync_ws/install/setup.bash; export ROS_DISCOVERY_SERVER=127.0.0.1:11811 ROS_DISABLE_ROS2_DAEMON=1; ros2 launch rulesync_demo rulesync_sim.launch.py' > $OUT/mgr_exp.log 2>&1 < /dev/null &
for _ in $(seq 1 18); do
  sleep 10
  if grep -a -q "SYSTEM_READY (reconciled)" $OUT/mgr_exp.log 2>/dev/null; then break; fi
done
echo "stack ready (or timeout)"
# uav bridge (3D open_3d boxes, test pose)
python3 -c "from rulesync3d.rulesync3d.worlds_3d import WORLDS; import json; ab=[]; [ab.append((c[0]-s[0]/2,c[1]-s[1]/2,c[2]-s[2]/2,c[0]+s[0]/2,c[1]+s[1]/2,c[2]+s[2]/2)) for (c,s) in [((b[0],b[1],b[2]),(b[3],b[4],b[5])) for b in WORLDS['open_3d']()]]; open('/tmp/boxes_exp.json','w').write(json.dumps(ab))"
BOXES=$(cat /tmp/boxes_exp.json)
setsid nohup python3 -m topopatch_ros.uav_bridge_node --ros-args -p world_boxes:="'$BOXES'" -p spawn_xyz:="[1.0,10.0,1.0]" > $OUT/ubridge_exp.log 2>&1 < /dev/null &
sleep 15
echo "EXP1 steady-state (6 trials):"
timeout 400 python3 $CK/rulesync3d/tools/disco_probe.py 6
echo "EXP2 kill+restart manager, then rapid propose waits:"
for pat in '[r]ulesync_manager'; do pkill -9 -f "$pat" 2>/dev/null || true; done
sleep 3
setsid nohup bash -c 'source /opt/ros/lyrical/setup.bash; source /home/yule/rulesync_ws/install/setup.bash; export ROS_DISCOVERY_SERVER=127.0.0.1:11811 ROS_DISABLE_ROS2_DAEMON=1; ros2 run rulesync_manager manager --ros-args -p txn_log_path:=/tmp/rulesync_txn.log' > $OUT/mgr_exp2.log 2>&1 < /dev/null &
sleep 5
for i in 1 2 3 4 5 6 7 8 9 10; do
  timeout 60 python3 $CK/rulesync3d/tools/disco_probe.py 1
  sleep 5
done
echo "EXP3 kill+restart uav bridge, then rapid state waits:"
for pat in '[u]av_bridge'; do pkill -9 -f "$pat" 2>/dev/null || true; done
sleep 3
setsid nohup python3 -m topopatch_ros.uav_bridge_node --ros-args -p world_boxes:="'$BOXES'" -p spawn_xyz:="[1.0,10.0,1.0]" > $OUT/ubridge_exp2.log 2>&1 < /dev/null &
sleep 5
for i in 1 2 3 4 5 6 7 8 9 10; do
  timeout 60 python3 $CK/rulesync3d/tools/disco_probe.py 1
  sleep 5
done
echo "===== disco experiment done $(date)"
} >> $LOG 2>&1
