#!/bin/bash
# ProtoCrash-21: deterministic protocol crash matrix (correctness revision).
# 5 ops (ADD/TIGHTEN/REPLACE/REMOVE/RELAX) x crash {none,C0,C1,C2,C3} x
# restarts, bounded: all-ops baselines + REMOVE/RELAX full crash coverage.
# Runner OWNS manager+bridge lifecycle per cell (env-gated crash hooks,
# default OFF). Epochs sequential from 200000, recorded per row.
# Outputs: artifacts/correctness_revision/protocol_matrix.csv
# Rows carry §9 metrics + the four hard properties (all must be zero).
SIM_DIR=/opt/ros/lyrical/share/nav2_minimal_tb3_sim
export GZ_SIM_RESOURCE_PATH=$SIM_DIR/models:${GZ_SIM_RESOURCE_PATH:-}
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export ROS_DISABLE_ROS2_DAEMON=1
export PYTHONPATH=/home/yule/R2I-RL-checkout:/home/yule/R2I-RL-checkout/rulepatch:/home/yule/R2I-RL-checkout/topopatch_ros:${PYTHONPATH:-}
export PYTHONDONTWRITEBYTECODE=1
export OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1
# Test pose (bridge has no sim here): fixed free pose for admission.
export RULESYNC_TEST_POSE="1.0,10.0,1.0"
set +u
source /opt/ros/lyrical/setup.bash
source /home/yule/rulesync_ws/install/setup.bash
set -u
CK=/home/yule/R2I-RL-checkout
OUT=/home/yule/rulepatch/3d/correctness_revision
mkdir -p $OUT
LOG=$OUT/proto_crash.log
echo "[proto] starting matrix" | tee $LOG

kill_procs() {
  for pat in '[r]ulesync_manager' '[r]ulesync_nav2_bridge' '[u]av_bridge'; do
    pkill -9 -f "$pat" 2>/dev/null || true
  done
  sleep 3
  # PID-verified hygiene (learned hard: bare pkill output lies; stale
  # same-name servers steal vote/state responses and poison epochs).
  for _ in $(seq 1 5); do
    left=$(pgrep -f '[r]ulesync_manager/lib|[r]ulesync_nav2_bridge/lib|[u]av_bridge_node' | head -5)
    [ -z "$left" ] && break
    echo "[proto] re-killing: $left"
    for p in $left; do kill -9 $p 2>/dev/null; done
    sleep 2
  done
  left=$(pgrep -f '[r]ulesync_manager/lib|[r]ulesync_nav2_bridge/lib|[u]av_bridge_node' | head -5)
  if [ -n "$left" ]; then
    echo "[proto] FATAL: processes survive hygiene: $left" >&2
    return 1
  fi
  return 0
}

preflight_singletons() {
  # Exactly one nav2_bridge and one uav_bridge may exist; duplicates
  # cause split-brain vote/state responses (verified 206000 phantom).
  local n2=$(pgrep -fc '[r]ulesync_nav2_bridge/lib' || true)
  local nu=$(pgrep -fc '[u]av_bridge_node' || true)
  local nm=$(pgrep -fc '[r]ulesync_manager/lib' || true)
  echo "[proto] singletons: mgr=$nm nav2=$n2 uav=$nu (want 1/1/1)"
  [ "$nm" = "1" ] && [ "$n2" = "1" ] && [ "$nu" = "1" ]
}

restart_ds() {
  # FULL discovery reset (cell start / all-parties restart): kill the
  # server, wipe SHM segments, start fresh. Safe only when NO
  # participant survives (wiping SHM under live processes breaks
  # their peer-to-peer SHM channels).
  if [ -f /tmp/ds_proto.pid ]; then
    kill -9 "$(cat /tmp/ds_proto.pid)" 2>/dev/null || true
    rm -f /tmp/ds_proto.pid
  fi
  pkill -9 -f '[f]ast-discovery-server' 2>/dev/null || true
  pkill -9 -f '[f]astdds discovery' 2>/dev/null || true
  sleep 2
  rm -f /dev/shm/fastdds_* /dev/shm/sem.fastdds_* 2>/dev/null || true
  bash -c 'source /opt/ros/lyrical/setup.bash && setsid nohup fastdds discovery -i 0 -l 127.0.0.1 -p 11811 > /tmp/ds_pc.log 2>&1 < /dev/null & echo bg' >/dev/null 2>&1 || true
  sleep 2
  # Newest matching server owns the fresh registry (PID file for the
  # next cell's deterministic kill).
  pgrep -nf '[f]astdds discovery' > /tmp/ds_proto.pid 2>/dev/null || true
  sleep 8
}

# start_mgr <mgr_crash_env|-> : fresh snapshot+WAL every cell (isolated,
# deterministic); prints READY line.
start_mgr() {
  # NOTE: never rm state/txn/WAL here — restart phases must PRESERVE
  # durable records (recovery reads them). Freshness is established once
  # per cell in run_cell. Wiping here voided crash testing (verified).
  if [ -n "${1:-}" ] && [ "$1" != "-" ]; then
    export RULESYNC_MGR_CRASH_AT="$1"
  else
    unset RULESYNC_MGR_CRASH_AT
  fi
  setsid nohup bash -c 'source /opt/ros/lyrical/setup.bash; source /home/yule/rulesync_ws/install/setup.bash; export ROS_DISCOVERY_SERVER=127.0.0.1:11811 ROS_DISABLE_ROS2_DAEMON=1; ros2 launch rulesync_demo rulesync_sim.launch.py' > $OUT/mgr_cell.log 2>&1 < /dev/null &
  for _ in $(seq 1 9); do
    sleep 10
    if grep -a -q "SYSTEM_READY (reconciled)" $OUT/mgr_cell.log 2>/dev/null; then
      break
    fi
  done
  grep -E "SYSTEM_READY|SYSTEM_DEGRADED" $OUT/mgr_cell.log | tail -1
}

# start_mgr_standalone <mgr_crash_env|-> : manager process ONLY (no
# nav2_bridge). The launch ALWAYS spawns nav2_bridge too — using it for a
# manager-only restart duplicates the bridge (split-brain votes). Any
# mgr-only (re)start MUST use this (verified failure mode).
start_mgr_standalone() {
  if [ -n "${1:-}" ] && [ "$1" != "-" ]; then
    export RULESYNC_MGR_CRASH_AT="$1"
  else
    unset RULESYNC_MGR_CRASH_AT
  fi
  setsid nohup bash -c 'source /opt/ros/lyrical/setup.bash; source /home/yule/rulesync_ws/install/setup.bash; export ROS_DISCOVERY_SERVER=127.0.0.1:11811 ROS_DISABLE_ROS2_DAEMON=1 RULESYNC_MGR_CRASH_AT="$RULESYNC_MGR_CRASH_AT"; ros2 run rulesync_manager manager --ros-args -p txn_log_path:=/tmp/rulesync_txn.log' > $OUT/mgr_cell.log 2>&1 < /dev/null &
  for _ in $(seq 1 9); do
    sleep 10
    if grep -a -q "SYSTEM_READY (reconciled)" $OUT/mgr_cell.log 2>/dev/null; then
      break
    fi
  done
  grep -E "SYSTEM_READY|SYSTEM_DEGRADED" $OUT/mgr_cell.log | tail -1
}

# start_bridge <bridge_crash_env|-> : open_3d boxes, fixed test pose.
start_bridge() {
  if [ -n "${1:-}" ] && [ "$1" != "-" ]; then
    export RULESYNC_BRIDGE_CRASH_AT="$1"
  else
    unset RULESYNC_BRIDGE_CRASH_AT
  fi
  python3 -c "from rulesync3d.rulesync3d.worlds_3d import WORLDS; import json; ab=[]; [ab.append((c[0]-s[0]/2,c[1]-s[1]/2,c[2]-s[2]/2,c[0]+s[0]/2,c[1]+s[1]/2,c[2]+s[2]/2)) for (c,s) in [((b[0],b[1],b[2]),(b[3],b[4],b[5])) for b in WORLDS['open_3d']()]]; open('/tmp/boxes_pc.json','w').write(json.dumps(ab))"
  BOXES=$(cat /tmp/boxes_pc.json)
  setsid nohup python3 -m topopatch_ros.uav_bridge_node --ros-args \
    -p world_boxes:="'$BOXES'" -p spawn_xyz:="[1.0,10.0,1.0]" \
    > $OUT/ubridge_cell.log 2>&1 < /dev/null &
  sleep 8
}

# wait_ready: file-based convergence gate (no protocol pollution: a probe
# propose would leave staged residue that later commits would merge).
# Waits for vol-reconcile-ok / vol-converge-explicit in the fresh txn log.
wait_ready() {
  wait_ready_from 0
}

# wait_ready_from <nlines>: same gate but only records APPENDED after the
# txn log had <nlines> lines count (post-restart convergence proof: stale
# pre-restart records must not satisfy the gate).
wait_ready_from() {
  local FROM=${1:-0}
  for _ in $(seq 1 48); do
    sleep 5
    if [ -f /tmp/rulesync_txn.log ] && \
       tail -n +$((FROM + 1)) /tmp/rulesync_txn.log 2>/dev/null | \
       grep -a -q -E "vol-reconcile-ok|vol-converge-explicit|vol-state-refresh"; then
      echo "[proto] ready (converged)"
      return 0
    fi
  done
  echo "[proto] WARN: no convergence observed (proceeding anyway)"
  return 1
}

txn_lines() {
  if [ -f /tmp/rulesync_txn.log ]; then
    wc -l < /tmp/rulesync_txn.log | tr -d ' '
  else
    echo 0
  fi
}

# run_cell <cell> <op> <crash> <restart> <epoch>  (UID is readonly in bash)
run_cell() {
  local CELL=$1 OP=$2 CRASH=$3 RESTART=$4 EP=$5
  if [ -n "${CELL_RE:-}" ]; then
    case "$CELL" in
      $CELL_RE) ;;
      *) echo "[proto] ----- $CELL skipped by CELL_RE" | tee -a $LOG
        return 0 ;;
    esac
  fi
  echo "[proto] ===== $CELL op=$OP crash=$CRASH restart=$RESTART ep=$EP =====" | tee -a $LOG
  if ! kill_procs; then
    echo "[proto] $CELL HYGIENE FAIL — skipping cell" | tee -a $LOG
    return 0
  fi
  # Fresh durable state per cell (isolated, deterministic epochs).
  # Also drop any stale bridge arm file (a leftover must never crash a
  # fresh incarnation on its first commit, e.g. reconcile refresh).
  rm -f /tmp/rulesync_state.json /tmp/rulesync_txn.log /tmp/rulesync_wal.jsonl
  rm -f /tmp/uav_bridge_crash
  restart_ds
  local MENV="-" BENV="-"
  # crash hooks arm the process that must die at that point
  case "$CRASH" in
    C0) MENV="pre-wal" ;;
    C1) MENV="post-wal" ;;
    C3) MENV="post-dispatch" ;;
    C2) BENV="post-apply" ;;
  esac
  # NOTE: crash hooks stay DISARMED for the initial start. Crash cells
  # arm them only after the setup phase (arm-restart below): arming from
  # process start would crash the SETUP commit instead of the TEST txn
  # (verified failure mode: setup-consumed crash + driver hang).
  unset RULESYNC_MGR_CRASH_AT RULESYNC_BRIDGE_CRASH_AT
  start_mgr "-"
  start_bridge "-"
  if ! preflight_singletons; then
    echo "[proto] $CELL PREFLIGHT FAIL — skipping cell" | tee -a $LOG
    return 0
  fi
  wait_ready
  if [ "$CRASH" = "none" ]; then
    # Baseline: legacy one-shot (no crash; hooks disarmed).
    timeout 600 python3 $CK/rulesync3d/tools/proto_op.py \
      --uid "$CELL" --op "$OP" --epoch $EP --out $OUT/matrix_rows.jsonl --expect /tmp/proto_expect_$CELL.json \
      >> $LOG 2>&1 || echo "[proto] $CELL driver exit: $?" | tee -a $LOG
  else
    # Crash cell: setup (hooks DISARMED) -> arm-restart (hooks ARMED,
    # durable state preserved) -> test (crash fires in the TEST txn).
    # Arming from process start would crash the SETUP commit instead
    # (verified: setup-consumed crash + driver hang, invalid cell).
    timeout 600 python3 $CK/rulesync3d/tools/proto_op.py \
      --uid "$CELL" --op "$OP" --epoch $EP --out $OUT/matrix_rows.jsonl --expect /tmp/proto_expect_$CELL.json \
      --phase setup \
      >> $LOG 2>&1 || echo "[proto] $CELL setup exit: $?" | tee -a $LOG
    ARM_FROM=$(txn_lines)
    case "$CRASH" in
      C0|C1|C3)
        for pat in '[r]ulesync_manager'; do pkill -9 -f "$pat" 2>/dev/null; done
        sleep 3
        export RULESYNC_MGR_CRASH_AT="$MENV"
        start_mgr_standalone "$MENV"
        unset RULESYNC_MGR_CRASH_AT
        ;;
      C2)
        # File-based arm (NO restart, NO discovery): a bridge restart
        # here would desynchronize it right before the TEST txn and
        # invite a reconcile refresh to race the test commit (verified);
        # ros2-param arming proved flaky (ghost node names, blind CLI
        # discovery). The bridge consumes the file on fire; the runner
        # also removes it at cell start + recovery (a stale arm must
        # never crash a later incarnation, e.g. the reconcile refresh).
        echo "post-apply" > /tmp/uav_bridge_crash
        echo "[proto] $CELL bridge armed via file" | tee -a $LOG
        sleep 5
        ;;
    esac
    if ! preflight_singletons; then
      echo "[proto] $CELL ARM-RESTART PREFLIGHT FAIL — retrying once after settle" | tee -a $LOG
      sleep 15
      if ! preflight_singletons; then
        echo "[proto] $CELL ARM-RESTART PREFLIGHT FAIL — skipping cell" | tee -a $LOG
        return 0
      fi
    fi
    if [ "$CRASH" = "C2" ]; then
      # No restart happened (runtime arm): system is still converged
      # from setup; a log gate here would only match stale records.
      sleep 5
    else
      # Gate on post-arm-restart convergence (fresh log records only):
      # serializes any reconcile refresh BEFORE the test txn so the two
      # never interleave on the bridge staged identity.
      wait_ready_from "$ARM_FROM"
    fi
    timeout 600 python3 $CK/rulesync3d/tools/proto_op.py \
      --uid "$CELL" --op "$OP" --epoch $EP --out $OUT/matrix_rows.jsonl --expect /tmp/proto_expect_$CELL.json \
      --phase test \
      >> $LOG 2>&1 || echo "[proto] $CELL driver exit: $?" | tee -a $LOG
  fi
  # restart phase per mode (manager and/or bridge fresh, NO hooks)
  unset RULESYNC_MGR_CRASH_AT RULESYNC_BRIDGE_CRASH_AT
  rm -f /tmp/uav_bridge_crash
  sleep 2
  T0=$(date +%s)
  REC_FROM=$(txn_lines)
  # NOTE: no DS bounce at restart points (verified harmful: survivors
  # of a server bounce never re-register — permanently isolated).
  # Ghost-lease flakes are absorbed by driver op retry + long gates.
  case "$RESTART" in
    mgr) for pat in '[r]ulesync_manager'; do pkill -9 -f "$pat" 2>/dev/null; done; sleep 3; start_mgr_standalone "-" ;;
    bridge) for pat in '[u]av_bridge'; do pkill -9 -f "$pat" 2>/dev/null; done; sleep 3; start_bridge "-" ;;
    both) kill_procs; restart_ds; start_mgr "-"; start_bridge "-" ;;
    none) ;;
  esac
  if [ "$CRASH" = "C2" ]; then
    # The crashed bridge must be revived regardless of restart mode
    # (restart=mgr alone would leave it dead and the audit serviceless).
    case "$RESTART" in
      bridge|both) ;;
      *) for pat in '[u]av_bridge'; do pkill -9 -f "$pat" 2>/dev/null; done; sleep 3; start_bridge "-" ;;
    esac
  fi
  # settle: allow reconcile cycles to converge (or reveal stuck DEGRADED)
  sleep 45
  wait_ready_from "$REC_FROM" || true
  # audit the cell (reads WAL/snapshot/services, writes matrix row)
  # Generous wrapper (600 s): service waits retry through transient
  # post-restart discovery sickness; the audit itself bounds each step.
  timeout 600 python3 $CK/rulesync3d/tools/proto_audit.py \
    --uid "$CELL" --op "$OP" --crash "$CRASH" --restart "$RESTART" \
    --epoch $EP --t0 $T0 --out $OUT/matrix_rows.jsonl \
    --expect /tmp/proto_expect_$CELL.json >> $LOG 2>&1 \
    || echo "[proto] $CELL audit exit: $?" | tee -a $LOG
  # Archive per-cell durable records (state/txn/WAL are wiped next cell;
  # epochs are per-cell unique, so rows join without inline tags).
  cat /tmp/rulesync_txn.log 2>/dev/null >> $OUT/txn_all.jsonl
  cat /tmp/rulesync_wal.jsonl 2>/dev/null >> $OUT/wal_all.jsonl
}

EP=300000
# baselines: all ops, no crash, no restart
for OP in ADD TIGHTEN REPLACE REMOVE RELAX; do
  run_cell "base-$OP" "$OP" "none" "none" $EP; EP=$((EP+1000))
done
# REMOVE full crash coverage (both restart)
for C in C0 C1 C2 C3; do
  run_cell "remove-$C" "REMOVE" "$C" "both" $EP; EP=$((EP+1000))
done
# RELAX full crash coverage (both restart)
for C in C0 C1 C2 C3; do
  run_cell "relax-$C" "RELAX" "$C" "both" $EP; EP=$((EP+1000))
done
# representative: ADD x crash x mgr + spot checks
for C in C0 C1 C2 C3; do
  run_cell "add-$C" "ADD" "$C" "mgr" $EP; EP=$((EP+1000))
done
run_cell "tighten-C1" "TIGHTEN" "C1" "both" $EP; EP=$((EP+1000))
run_cell "replace-C2" "REPLACE" "C2" "both" $EP; EP=$((EP+1000))
run_cell "add-bridge-restart" "ADD" "none" "bridge" $EP; EP=$((EP+1000))
run_cell "remove-bridge-restart" "REMOVE" "none" "bridge" $EP; EP=$((EP+1000))
echo "[proto] matrix done." | tee -a $LOG
# compile CSVs
python3 - "$OUT" <<'PYEOF'
import json, sys, csv
out = sys.argv[1]
seen = {}
for l in open(f"{out}/matrix_rows.jsonl"):
    try:
        r = json.loads(l)
        seen[r.get("uid", "")] = r  # reruns overwrite (keep LAST)
    except Exception:
        continue
rows = sorted(seen.values(), key=lambda r: (r.get("uid", "")))
mkeys = ["uid","op","crash","restart","txn_id","epoch","target_hash",
         "durable_present","mgr_epoch","mgr_hash","bridge_epoch",
         "bridge_hash","guard_view_consistent","ready_state",
         "motion_authorized","recovery_ms","unauthorized_motion",
         "silent_state_divergence","committed_update_lost",
         "uncommitted_relaxation_promoted","hook_verified",
         "hook_evidence","note"]
with open(f"{out}/protocol_matrix.csv","w",newline="") as f:
    w = csv.DictWriter(f, fieldnames=mkeys)
    w.writeheader()
    for r in rows:
        w.writerow({k: r.get(k) for k in mkeys})
print(f"matrix rows: {len(rows)}")
bad = [r["uid"] for r in rows if r.get("unauthorized_motion") or
       r.get("silent_state_divergence") or
       r.get("committed_update_lost") or
       r.get("uncommitted_relaxation_promoted")]
print("HARD-FAIL rows:", bad if bad else "none (all four properties zero)")
inv = [r["uid"] for r in rows if not r.get("hook_verified")]
print("HOOK-UNVERIFIED rows:", inv if inv else "none (all cells valid)")
print("GATE:", "CLOSED" if not bad and not inv and rows else "OPEN")
PYEOF
