#!/bin/bash
# Stage-A 3D campaign: per world (fresh manager + Gazebo + quad + bridges
# + uav_bridge + driver subset). 20 dev scenarios.
#
# Epoch hygiene (hard-won): committed_epoch is globally monotonic, so each
# world gets a FRESH manager+snapshot and a disjoint epoch block
# (base + idx*100, assigned in run order by the driver). Never reuse a
# manager across worlds; never trust scenario-file epochs.
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
PAPER=/home/yule/rulepatch/paper
LOG=/home/yule/rulepatch/3d/logs
mkdir -p $LOG
# Single-instance guard: a zombie runner split-brain voided v2/v3
# (two sim servers, cross-talk on global topics). flock-on-fd does NOT
# work here (background children inherit the fd and hold the lock after
# the parent dies); use a pidfile with liveness check instead.
if [ -f /tmp/stage_a.pid ]; then
  OLDPID=$(cat /tmp/stage_a.pid 2>/dev/null || echo "")
  if [ -n "$OLDPID" ] && ps -p "$OLDPID" -o args= 2>/dev/null | grep -q run_stage_a; then
    echo "[stage-a] FATAL: live runner PID $OLDPID holds /tmp/stage_a.pid" >&2
    exit 42
  fi
  echo "[stage-a] stale pidfile (PID ${OLDPID:-?} dead), taking over"
fi
echo $$ > /tmp/stage_a.pid
trap "rm -f /tmp/stage_a.pid" EXIT
# world:spawn:epochbase — spawns are per-world CLEAR + MISSION-COVERING.
# SEED (env, default 0): frozen-campaign seed index; shifts epoch blocks
# by SEED*1000 and suffixes CSVs (deterministic RRT sampling via epoch).
SEED=${SEED:-0}
WORLDS="open_3d:10.0,10.0,3.0:50000 floating_barrier:5.0,10.0,4.0:70000 vertical_corridor:10.0,10.0,3.0:90000 multi_level:10.0,10.0,3.0:110000"

kill_sim() {
  # Never blanket-kill '[r]os2 launch'/'[r]os2 run': that matches the
  # manager's own launch command (voided a prior campaign era).
  for pat in '[g]z-sim-main' '[g]z sim' '[g]z-sim' '[r]obot_state_publisher' '[s]pawn_tb3' '[p]arameter_bridge' '[s]tatic_transform_publisher' '[x]vfb-run'; do
    pkill -9 -f "$pat" 2>/dev/null || true
  done
  sleep 3
}

preflight() {
  # Fail fast: exactly one sim server, expected walls present, quad
  # spawned, jz actuator alive. Stale multi-server soup voided v2-barrier.
  local world=$1 nsrv
  nsrv=$(pgrep -fc 'gz-sim|gz sim' || true)
  echo "[preflight] gz-server-procs=$nsrv (want 1-2: xvfb+server)"
  local walls
  walls=$(timeout 25 gz model --list 2>/dev/null | grep -c wall || true)
  echo "[preflight] walls=$walls"
  if [ "$walls" -lt 4 ]; then echo "[preflight] FAIL: walls missing"; return 1; fi
  return 0
}

kill_mgr() {
  for pat in '[r]ulesync_manager' '[r]ulesync_nav2_bridge' '[t]opo_inference' '[g]ate_node' '[u]av_bridge' '[u]av_case_driver'; do
    pkill -9 -f "$pat" 2>/dev/null || true
  done
  sleep 3
}

start_mgr() {
  rm -f /tmp/rulesync_state.json
  rm -f /tmp/rulesync_txn.log
  setsid nohup bash -c 'source /opt/ros/lyrical/setup.bash; source /home/yule/rulesync_ws/install/setup.bash; export ROS_DISCOVERY_SERVER=127.0.0.1:11811 ROS_DISABLE_ROS2_DAEMON=1; ros2 launch rulesync_demo rulesync_sim.launch.py' > $LOG/mgr_$1$CSVSUFFIX.log 2>&1 < /dev/null &
  sleep 12
  grep -E "SYSTEM_READY|SYSTEM_DEGRADED" $LOG/mgr_$1$CSVSUFFIX.log | tail -1
}

restart_ds() {
  # Fresh discovery server per world: kill -9'd nodes never unregister,
  # and stale GUIDs poison name-reusing successors (verified publisher/
  # client degradation). 6 s cost per world, deterministic matching.
  pkill -9 -f '[f]astdds(.py)? discovery|[f]ast-discovery-server' 2>/dev/null || true
  sleep 2
  rm -f /dev/shm/fastdds_* /dev/shm/sem.fastdds_* 2>/dev/null || true
  bash -c 'source /opt/ros/lyrical/setup.bash && setsid nohup fastdds discovery -i 0 -l 127.0.0.1 -p 11811 > /tmp/ds_3d.log 2>&1 < /dev/null & echo bg' >/dev/null 2>&1 || true
  sleep 4
}

# Discovery server is (re)started per world by restart_ds() inside the
# loop (stale-GUID hygiene); nothing one-time here.
cd $CK

for ENTRY in $WORLDS; do
  WORLD=$(echo $ENTRY | cut -d: -f1)
  SP=$(echo $ENTRY | cut -d: -f2)
  EBASE0=$(echo $ENTRY | cut -d: -f3)
  EBASE=$((EBASE0 + SEED * 1000))
  CSVSUFFIX=""
  if [ "$SEED" != "0" ]; then CSVSUFFIX="_s$SEED"; fi
  SX=$(echo $SP | cut -d, -f1); SY=$(echo $SP | cut -d, -f2); SZ=$(echo $SP | cut -d, -f3)
  echo "[stage-a] ===== $WORLD spawn=($SX,$SY,$SZ) epochbase=$EBASE ====="
  kill_sim
  kill_mgr
  restart_ds
  start_mgr $WORLD
  setsid nohup xvfb-run -a gz sim -r -s /home/yule/rulepatch/3d/worlds/$WORLD.sdf > $LOG/sim_$WORLD$CSVSUFFIX.log 2>&1 < /dev/null &
  sleep 12
  if ! preflight $WORLD; then
    echo "[stage-a] $WORLD PREFLIGHT FAIL — dumping procs, skipping world"
    ps aux | grep -E 'gz|xvfb' | grep -v grep | head -5 || true
    continue
  fi
  setsid nohup ros2 launch nav2_minimal_tb3_sim spawn_tb3.launch.py \
    robot_name:=r2i_quad robot_sdf:="/home/yule/R2I-RL-checkout/rulesync3d/worlds/r2i_quad.sdf" \
    x_pose:=$SX y_pose:=$SY z_pose:=$SZ > $LOG/spawn_$WORLD$CSVSUFFIX.log 2>&1 < /dev/null &
  sleep 8
  setsid nohup ros2 run ros_gz_bridge parameter_bridge --ros-args \
    -p config_file:=/tmp/quad_bridge.yaml -p use_sim_time:=True \
    -p expand_gz_topic_names:=True > $LOG/bridge_$WORLD$CSVSUFFIX.log 2>&1 < /dev/null &
  sleep 6
  # uav_bridge with THIS world's boxes (feasibility occupancy)
  python3 -c "from rulesync3d.rulesync3d.worlds_3d import WORLDS; import json; ab=[]; [ab.append((c[0]-s[0]/2,c[1]-s[1]/2,c[2]-s[2]/2,c[0]+s[0]/2,c[1]+s[1]/2,c[2]+s[2]/2)) for (c,s) in [((b[0],b[1],b[2]),(b[3],b[4],b[5])) for b in WORLDS['$WORLD']()]]; open('/tmp/boxes_$WORLD.json','w').write(json.dumps(ab)); print('boxes:',len(ab))"
  for pat in '[u]av_bridge'; do pkill -9 -f "$pat" 2>/dev/null || true; done
  sleep 2
  BOXES=$(cat /tmp/boxes_$WORLD.json)
  setsid nohup python3 -m topopatch_ros.uav_bridge_node --ros-args \
    -p world_boxes:="'$BOXES'" -p spawn_xyz:="[$SX,$SY,$SZ]" \
    > $LOG/ubridge_$WORLD$CSVSUFFIX.log 2>&1 < /dev/null &
  sleep 8
  # driver subset for this world
  python3 -c "import json;cs=json.load(open('/tmp/ompl_10.json'));sub=[c for c in cs if c['map']=='$WORLD'];json.dump(sub,open('/tmp/cases_$WORLD.json','w'));print('cases:',len(sub))"
  timeout 2400 python3 -m topopatch_ros.uav_case_driver \
    --csv $PAPER/ompl_$WORLD$CSVSUFFIX.csv --cases /tmp/cases_$WORLD.json \
    --world-boxes /tmp/boxes_$WORLD.json --spawn $SX,$SY,$SZ \
    --epoch-base $EBASE \
    > $LOG/driver_$WORLD$CSVSUFFIX.log 2>&1 || echo "[stage-a] $WORLD exit: $?"
  echo "[stage-a] $WORLD done."
done
echo "[stage-a] all done."
