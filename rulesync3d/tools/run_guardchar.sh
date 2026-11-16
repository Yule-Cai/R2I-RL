#!/bin/bash
# GuardChar-32: frozen RuleGuard three-arm characterization (32 runs).
# Arms: A gate (PATH_GATE_ONLY) / B gate_geomon (PATH_GATE+GEOMETRY_MONITOR)
#       / C gate_guard (FULL_RULEGUARD: geometry + epoch/hash binding).
# Cases frozen in scenarios_guardchar.py (NOT modified here); all 32 rows
# are executed FRESH (the manifest's reuse note is superseded: reused rows
# from older driver versions lack the current schema fields; deviation
# documented in RULEGUARD_CHARACTERIZATION.md).
# Outputs: artifacts/correctness_revision/ruleguard_runs.csv (canonical
# schema) + ruleguard_trajectories/*.npz + manifests/GuardChar-32.json.
# Requires the Gazebo stack (conflicts with protocol matrices: serialize).
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
OUT=/home/yule/rulepatch/3d/correctness_revision
TRAJ=$OUT/ruleguard_trajectories
MAN=$OUT/manifests
mkdir -p $OUT $TRAJ $MAN
LOG=$OUT/guardchar.log
echo "[guardchar] starting GuardChar-32" | tee $LOG

kill_all() {
  for pat in '[r]ulesync_manager' '[r]ulesync_nav2_bridge' '[u]av_bridge' '[g]z-sim-main' '[g]z sim' '[p]arameter_bridge' '[u]av_case_driver'; do
    pkill -9 -f "$pat" 2>/dev/null || true
  done
  sleep 3
}

restart_ds() {
  pkill -9 -f '[f]astdds(.py)? discovery|[f]ast-discovery-server' 2>/dev/null || true
  sleep 2
  rm -f /dev/shm/fastdds_* /dev/shm/sem.fastdds_* 2>/dev/null || true
  bash -c 'source /opt/ros/lyrical/setup.bash && setsid nohup fastdds discovery -i 0 -l 127.0.0.1 -p 11811 > /tmp/ds_gc.log 2>&1 < /dev/null & echo bg' >/dev/null 2>&1 || true
  sleep 4
}

kill_all
rm -f /tmp/rulesync_state.json /tmp/rulesync_txn.log /tmp/rulesync_wal.jsonl
restart_ds

# --- sim: open_3d + quad + joint bridges (proven bringup_3d pattern) ---
setsid nohup xvfb-run -a gz sim -r -s /home/yule/rulepatch/3d/worlds/open_3d.sdf > $OUT/sim_guardchar.log 2>&1 < /dev/null &
sleep 12
# NOTE: sim spawn is MISSION-COVERING (10,10,1), NOT the mission start
# (1,10,1): the gantry joints span ±15 m, so spawn (1,10,1) caps x at 16
# and the x=19 goal is unreachable (verified: all tracks stall at
# x=16.0). The frozen mission (start/goal/volumes) is untouched; only
# the sim spawn moves. Driver --spawn and uav_bridge spawn_xyz MUST
# match (both derive pose as spawn+joints).
setsid nohup ros2 launch nav2_minimal_tb3_sim spawn_tb3.launch.py \
  robot_name:=r2i_quad robot_sdf:="/home/yule/R2I-RL-checkout/rulesync3d/worlds/r2i_quad.sdf" \
  x_pose:=10.0 y_pose:=10.0 z_pose:=1.0 > $OUT/spawn_guardchar.log 2>&1 < /dev/null &
sleep 8
cat > /tmp/quad_bridge.yaml <<'EOF'
- topic_name: "/r2i_quad/jx/cmd_vel"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"
  direction: ROS_TO_GZ
- topic_name: "/r2i_quad/jy/cmd_vel"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"
  direction: ROS_TO_GZ
- topic_name: "/r2i_quad/jz/cmd_vel"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"
  direction: ROS_TO_GZ
- topic_name: "/r2i_quad/jyaw/cmd_vel"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"
  direction: ROS_TO_GZ
# Joint state arrives as gz.msgs.Model (NOT gz.msgs.JointState — that
# pair has no ros_gz_bridge specialization; verified against working
# bridge_open_3d.log).
- topic_name: "/r2i_quad/joint_state"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "gz.msgs.Model"
  direction: GZ_TO_ROS
- topic_name: "/body_contact"
  ros_type_name: "ros_gz_interfaces/msg/Contacts"
  gz_type_name: "gz.msgs.Contacts"
  direction: GZ_TO_ROS
EOF
setsid nohup ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -p config_file:=/tmp/quad_bridge.yaml -p use_sim_time:=True \
  -p expand_gz_topic_names:=True > $OUT/bridge_guardchar.log 2>&1 < /dev/null &
sleep 6

# --- manager + uav_bridge ---
setsid nohup bash -c 'source /opt/ros/lyrical/setup.bash; source /home/yule/rulesync_ws/install/setup.bash; export ROS_DISCOVERY_SERVER=127.0.0.1:11811 ROS_DISABLE_ROS2_DAEMON=1; ros2 launch rulesync_demo rulesync_sim.launch.py' > $OUT/mgr_guardchar.log 2>&1 < /dev/null &
sleep 60
python3 -c "from rulesync3d.rulesync3d.worlds_3d import WORLDS; import json; ab=[]; [ab.append((c[0]-s[0]/2,c[1]-s[1]/2,c[2]-s[2]/2,c[0]+s[0]/2,c[1]+s[1]/2,c[2]+s[2]/2)) for (c,s) in [((b[0],b[1],b[2]),(b[3],b[4],b[5])) for b in WORLDS['open_3d']()]]; open('/tmp/boxes_gc.json','w').write(json.dumps(ab)); print('boxes:',len(ab))"
BOXES=$(cat /tmp/boxes_gc.json)
setsid nohup python3 -m topopatch_ros.uav_bridge_node --ros-args \
  -p world_boxes:="'$BOXES'" -p spawn_xyz:="[10.0,10.0,1.0]" \
  > $OUT/ubridge_guardchar.log 2>&1 < /dev/null &
sleep 8

# --- frozen cases + driver (all 32 fresh) ---
python3 $CK/rulesync3d/tools/scenarios_guardchar.py | tee -a $LOG
cp $CK/rulesync3d/tools/guardchar_manifest.json $MAN/GuardChar-32.json
timeout 5400 python3 -m topopatch_ros.uav_case_driver \
  --csv $OUT/guardchar_raw.csv --cases /tmp/cases_guardchar.json \
  --world-boxes /tmp/boxes_gc.json --spawn 10.0,10.0,1.0 \
  --epoch-base 170000 --dump-all --dump-dir $TRAJ \
  >> $LOG 2>&1 || echo "[guardchar] driver exit: $?" | tee -a $LOG

# --- compile canonical schema ---
python3 - "$OUT" <<'PYEOF'
import csv, json, sys
out = sys.argv[1]
rows = list(csv.DictReader(open(f"{out}/guardchar_raw.csv")))
canon = ["uid","arm","speed","monitor_hz","pose_delay_ms",
         "stop_delay_ms","localization_noise","margin","hysteresis",
         "time_to_warning_ms","time_to_hold_ms","minimum_clearance",
         "maximum_penetration","samples_inside","time_inside_ms",
         "warning_count","hold_count","false_hold","goal_arrival",
         "snapshot_mismatch_hold","disturb","track_dump","reason"]
with open(f"{out}/ruleguard_runs.csv","w",newline="") as f:
    w = csv.DictWriter(f, fieldnames=canon)
    w.writeheader()
    for r in rows:
        try:
            hz = float(r.get("guard_monitor_hz") or 0)
        except Exception:
            hz = 0.0
        try:
            inside = int(float(r.get("guard_inside_n") or 0))
        except Exception:
            inside = 0
        t_inside = round(inside / hz * 1000.0, 1) if hz > 0 else 0.0
        try:
            holds = int(float(r.get("guard_hold_n") or 0))
        except Exception:
            holds = 0
        try:
            mclr = r.get("guard_min_clearance")
            mclr_f = float(mclr) if mclr not in (None, "", "None") else None
        except Exception:
            mclr_f = None
        stale = bool(r.get("guard_stale", "") in ("True", "true", "1")) or \
            "stale" in str(r.get("uid", "")) or \
            str(r.get("reason", "")).startswith("stale")
        missing = str(r.get("guard_missing", "")) in ("True", "true", "1") \
            or "missing" in str(r.get("uid", ""))
        mm_hold = bool((stale or missing) and holds > 0)
        # false HOLD: hold recorded with no inside samples and no
        # snapshot inconsistency (geometry-only disagreement with the
        # executed track); snapshot-mismatch holds are NOT false.
        fh = bool(holds > 0 and inside == 0 and not mm_hold)
        try:
            delay = float(r.get("guard_delay_ms") or 0.0)
        except Exception:
            delay = 0.0
        w.writerow({
            "uid": r.get("uid"), "arm": r.get("arm"),
            "speed": r.get("vmax_xy") or r.get("actual_speed_mean"),
            "monitor_hz": r.get("guard_monitor_hz") or r.get("track_rate_hz"),
            "pose_delay_ms": delay,
            # stop delay is not an independent knob in this harness: the
            # single effective sensing/stop delay is guard_delay_ms
            # (documented; Q1 reads it as the combined delay factor).
            "stop_delay_ms": delay,
            "localization_noise": r.get("guard_noise"),
            "margin": r.get("warn_margin"),
            "hysteresis": r.get("release_hyst"),
            "time_to_warning_ms": r.get("guard_t_warning_ms"),
            "time_to_hold_ms": r.get("guard_t_hold_ms"),
            "minimum_clearance": r.get("guard_min_clearance"),
            "maximum_penetration": r.get("guard_max_penetration"),
            "samples_inside": inside,
            "time_inside_ms": t_inside,
            "warning_count": r.get("guard_warn_n"),
            "hold_count": holds,
            "false_hold": fh,
            "goal_arrival": r.get("arrived"),
            "snapshot_mismatch_hold": mm_hold,
            "disturb": r.get("disturb"),
            "track_dump": r.get("track_dump"),
            "reason": r.get("reason")})
print(f"guardchar rows: {len(rows)}")
PYEOF
echo "[guardchar] done." | tee -a $LOG
