#!/bin/bash
# Smoke: vote path validation (2 cases) on open_3d.
SIM_DIR=/opt/ros/lyrical/share/nav2_minimal_tb3_sim
export GZ_SIM_RESOURCE_PATH=$SIM_DIR/models:${GZ_SIM_RESOURCE_PATH:-}
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export ROS_DISABLE_ROS2_DAEMON=1
export PYTHONPATH=/home/yule/R2I-RL-checkout:/home/yule/R2I-RL-checkout/rulepatch:/home/yule/R2I-RL-checkout/topopatch_ros:${PYTHONPATH:-}
export PYTHONDONTWRITEBYTECODE=1
export OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1
set +u
source /opt/ros/lyrical/setup.bash
source /home/yule/rulesync_ws/install/setup.bash
set -u
CK=/home/yule/R2I-RL-checkout
LOG=/home/yule/rulepatch/3d/logs
WORLD=open_3d
SX=1.0; SY=10.0; SZ=1.0
echo "[smoke] starting discovery+manager"
pkill -9 -f '[f]astdds(.py)? discovery|[f]ast-discovery-server' 2>/dev/null || true
sleep 2
rm -f /dev/shm/fastdds_* /dev/shm/sem.fastdds_* 2>/dev/null || true
rm -f /tmp/rulesync_state.json /tmp/rulesync_txn.log
bash -c 'source /opt/ros/lyrical/setup.bash && setsid nohup fastdds discovery -i 0 -l 127.0.0.1 -p 11811 > /tmp/ds_3d.log 2>&1 < /dev/null & echo bg' >/dev/null 2>&1 || true
sleep 4
for pat in '[r]ulesync_manager' '[r]ulesync_nav2_bridge' '[u]av_bridge' '[u]av_case_driver'; do
  pkill -9 -f "$pat" 2>/dev/null || true
done
sleep 2
setsid nohup bash -c 'source /opt/ros/lyrical/setup.bash; source /home/yule/rulesync_ws/install/setup.bash; export ROS_DISCOVERY_SERVER=127.0.0.1:11811 ROS_DISABLE_ROS2_DAEMON=1; ros2 launch rulesync_demo rulesync_sim.launch.py' > $LOG/mgr_smoke.log 2>&1 < /dev/null &
sleep 12
grep -E "SYSTEM_READY|SYSTEM_DEGRADED" $LOG/mgr_smoke.log | tail -1
echo "[smoke] starting sim"
# NOTE: never blanket-kill '[r]os2 launch'/'[r]os2 run' here: that matches
# the MANAGER's own launch command and kills it (voided an entire prior
# campaign era). Kill sim-side processes by specific patterns only.
for pat in '[g]z-sim-main' '[g]z sim' '[r]obot_state_publisher' '[s]pawn_tb3' '[p]arameter_bridge' '[s]tatic_transform_publisher' '[x]vfb-run'; do
  pkill -9 -f "$pat" 2>/dev/null || true
done
sleep 3
setsid nohup xvfb-run -a gz sim -r -s /home/yule/rulepatch/3d/worlds/$WORLD.sdf > $LOG/sim_smoke.log 2>&1 < /dev/null &
sleep 12
setsid nohup ros2 launch nav2_minimal_tb3_sim spawn_tb3.launch.py \
  robot_name:=r2i_quad robot_sdf:="/home/yule/R2I-RL-checkout/rulesync3d/worlds/r2i_quad.sdf" \
  x_pose:=$SX y_pose:=$SY z_pose:=$SZ > $LOG/spawn_smoke.log 2>&1 < /dev/null &
sleep 8
setsid nohup ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -p config_file:=/tmp/quad_bridge.yaml -p use_sim_time:=True \
  -p expand_gz_topic_names:=True > $LOG/bridge_smoke.log 2>&1 < /dev/null &
sleep 6
python3 -c "from rulesync3d.rulesync3d.worlds_3d import WORLDS; import json; ab=[]; [ab.append((c[0]-s[0]/2,c[1]-s[1]/2,c[2]-s[2]/2,c[0]+s[0]/2,c[1]+s[1]/2,c[2]+s[2]/2)) for (c,s) in [((b[0],b[1],b[2]),(b[3],b[4],b[5])) for b in WORLDS['$WORLD']()]]; open('/tmp/boxes_smoke.json','w').write(json.dumps(ab)); print('boxes:',len(ab))"
BOXES=$(cat /tmp/boxes_smoke.json)
setsid nohup python3 -m topopatch_ros.uav_bridge_node --ros-args \
  -p world_boxes:="'$BOXES'" -p spawn_xyz:="[$SX,$SY,$SZ]" \
  > $LOG/ubridge_smoke.log 2>&1 < /dev/null &
sleep 8
python3 -c "import json;cs=json.load(open('/tmp/stage_a_3d.json'));sub=[c for c in cs if c['uid'] in ('open-baseline-astar','open-float-aabb-astar')];json.dump(sub,open('/tmp/cases_smoke.json','w'));print('cases:',len(sub))"
cd $CK
timeout 900 python3 -m topopatch_ros.uav_case_driver \
  --csv /home/yule/rulepatch/paper/smoke_3d.csv --cases /tmp/cases_smoke.json \
  --world-boxes /tmp/boxes_smoke.json --spawn $SX,$SY,$SZ \
  --epoch-base 50000 \
  > $LOG/driver_smoke.log 2>&1 || echo "[smoke] exit: $?"
echo "[smoke] done."
