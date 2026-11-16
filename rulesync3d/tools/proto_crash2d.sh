#!/bin/bash
# ProtoCrash2D-15: targeted 2D RuleSpec crash matrix (correctness revision).
# 3 ops (ADD/REMOVE/RELAX) x crash {C0,C1,C2,C3} with manager restart as the
# primary crash actor (C2 restarts the crashed bridge) = 12 rows, plus
# ProtoRestart2D-3 (committed rule + manager/bridge/both restart) = 15 rows.
# Runner OWNS manager+bridge lifecycle per cell (env-gated crash hooks,
# default OFF). Epochs sequential from 400000, recorded per row.
# Headless: no Gazebo/Nav2 stack; the bridge uses RULESYNC_TEST_POSE2D /
# RULESYNC_TEST_GOAL2D feasibility overrides (test hook, default OFF).
# Outputs: artifacts/correctness_revision/protocol_matrix_2d.csv
# Rows carry the four hard properties (all must be zero).
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export ROS_DISABLE_ROS2_DAEMON=1
export PYTHONDONTWRITEBYTECODE=1
export OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1
# Headless test pose/goal for 2D feasibility (bridge test hook).
export RULESYNC_TEST_POSE2D="0.0,0.0"
export RULESYNC_TEST_GOAL2D="-2.0,-0.5"
set +u
source /opt/ros/lyrical/setup.bash
source /home/yule/rulesync_ws/install/setup.bash
set -u
CK=/home/yule/R2I-RL-checkout
OUT=/home/yule/rulepatch/3d/correctness_revision
mkdir -p $OUT
LOG=$OUT/proto_crash2d.log
echo "[proto2d] starting matrix" | tee $LOG

kill_procs() {
  for pat in '[r]ulesync_manager' '[r]ulesync_nav2_bridge' '[u]av_bridge'; do
    pkill -9 -f "$pat" 2>/dev/null || true
  done
  sleep 3
  # PID-verified hygiene (learned hard: bare pkill output lies; stale
  # same-name servers steal vote/state responses and poison epochs).
  for _ in $(seq 1 5); do
    left=$(pgrep -f '[r]ulesync_manager/lib|[r]ulesync_nav2_bridge/lib' | head -5)
    [ -z "$left" ] && break
    echo "[proto2d] re-killing: $left"
    for p in $left; do kill -9 $p 2>/dev/null; done
    sleep 2
  done
  left=$(pgrep -f '[r]ulesync_manager/lib|[r]ulesync_nav2_bridge/lib' | head -5)
  if [ -n "$left" ]; then
    echo "[proto2d] FATAL: processes survive hygiene: $left" >&2
    return 1
  fi
  return 0
}

preflight_singletons() {
  # Exactly one manager and one nav2_bridge may exist; duplicates cause
  # split-brain vote/state responses.
  local nm=$(pgrep -fc '[r]ulesync_manager/lib' || true)
  local n2=$(pgrep -fc '[r]ulesync_nav2_bridge/lib' || true)
  echo "[proto2d] singletons: mgr=$nm nav2=$n2 (want 1/1)"
  [ "$nm" = "1" ] && [ "$n2" = "1" ]
}

restart_ds() {
  # FULL discovery reset (cell start / all-parties restart): kill the
  # server, wipe SHM segments, start fresh. Safe only when NO
  # participant survives (wiping SHM under live processes breaks
  # their peer-to-peer SHM channels).
  if [ -f /tmp/ds_proto2d.pid ]; then
    kill -9 "$(cat /tmp/ds_proto2d.pid)" 2>/dev/null || true
    rm -f /tmp/ds_proto2d.pid
  fi
  pkill -9 -f '[f]ast-discovery-server' 2>/dev/null || true
  pkill -9 -f '[f]astdds discovery' 2>/dev/null || true
  sleep 2
  rm -f /dev/shm/fastdds_* /dev/shm/sem.fastdds_* 2>/dev/null || true
  bash -c 'source /opt/ros/lyrical/setup.bash && setsid nohup fastdds discovery -i 0 -l 127.0.0.1 -p 11811 > /tmp/ds_pc2d.log 2>&1 < /dev/null & echo bg' >/dev/null 2>&1 || true
  sleep 2
  pgrep -nf '[f]astdds discovery' > /tmp/ds_proto2d.pid 2>/dev/null || true
  sleep 8
}

# start_mgr <mgr_crash_env|-> : prints READY line.
start_mgr() {
  # NOTE: never rm state/txn/WAL here — restart phases must PRESERVE
  # durable records (recovery reads them). Freshness is established once
  # per cell in run_cell. Wiping here voided crash testing (verified).
  if [ -n "${1:-}" ] && [ "$1" != "-" ]; then
    export RULESYNC_MGR_CRASH_AT="$1"
  else
    unset RULESYNC_MGR_CRASH_AT
  fi
  setsid nohup bash -c 'source /opt/ros/lyrical/setup.bash; source /home/yule/rulesync_ws/install/setup.bash; export ROS_DISCOVERY_SERVER=127.0.0.1:11811 ROS_DISABLE_ROS2_DAEMON=1 RULESYNC_TEST_POSE2D="$RULESYNC_TEST_POSE2D" RULESYNC_TEST_GOAL2D="$RULESYNC_TEST_GOAL2D" RULESYNC_BRIDGE_CRASH_AT="$RULESYNC_BRIDGE_CRASH_AT" RULESYNC_MGR_CRASH_AT="$RULESYNC_MGR_CRASH_AT"; ros2 launch rulesync_demo rulesync_sim.launch.py' > $OUT/mgr_cell2d.log 2>&1 < /dev/null &
  for _ in $(seq 1 9); do
    sleep 10
    if grep -a -q "SYSTEM_READY (reconciled)" $OUT/mgr_cell2d.log 2>/dev/null; then
      break
    fi
  done
  grep -E "SYSTEM_READY|SYSTEM_DEGRADED" $OUT/mgr_cell2d.log | tail -1
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
  setsid nohup bash -c 'source /opt/ros/lyrical/setup.bash; source /home/yule/rulesync_ws/install/setup.bash; export ROS_DISCOVERY_SERVER=127.0.0.1:11811 ROS_DISABLE_ROS2_DAEMON=1 RULESYNC_TEST_POSE2D="$RULESYNC_TEST_POSE2D" RULESYNC_TEST_GOAL2D="$RULESYNC_TEST_GOAL2D" RULESYNC_BRIDGE_CRASH_AT="$RULESYNC_BRIDGE_CRASH_AT" RULESYNC_MGR_CRASH_AT="$RULESYNC_MGR_CRASH_AT"; ros2 run rulesync_manager manager --ros-args -p txn_log_path:=/tmp/rulesync_txn.log' > $OUT/mgr_cell2d.log 2>&1 < /dev/null &
  for _ in $(seq 1 9); do
    sleep 10
    if grep -a -q "SYSTEM_READY (reconciled)" $OUT/mgr_cell2d.log 2>/dev/null; then
      break
    fi
  done
  grep -E "SYSTEM_READY|SYSTEM_DEGRADED" $OUT/mgr_cell2d.log | tail -1
}

# start_bridge2d: standalone nav2_bridge (launch already starts one; this
# is for bridge-restart phases). Inherits the test pose/goal env.
start_bridge2d() {
  if [ -n "${1:-}" ] && [ "$1" != "-" ]; then
    export RULESYNC_BRIDGE_CRASH_AT="$1"
  else
    unset RULESYNC_BRIDGE_CRASH_AT
  fi
  setsid nohup bash -c 'source /opt/ros/lyrical/setup.bash; source /home/yule/rulesync_ws/install/setup.bash; export ROS_DISCOVERY_SERVER=127.0.0.1:11811 ROS_DISABLE_ROS2_DAEMON=1 RULESYNC_TEST_POSE2D="$RULESYNC_TEST_POSE2D" RULESYNC_TEST_GOAL2D="$RULESYNC_TEST_GOAL2D"; ros2 run rulesync_nav2_bridge bridge --ros-args -p odom_topic:=/odom -p goal_topic:=/goal_pose' > $OUT/n2bridge_cell2d.log 2>&1 < /dev/null &
  sleep 8
}

# wait_ready: file-based convergence gate (no protocol pollution: a probe
# propose would leave staged residue that later commits would merge).
# Waits for reconcile-ok / converge-explicit in the fresh txn log.
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
       grep -a -q -E "reconcile-ok|converge-explicit|state-refresh"; then
      echo "[proto2d] ready (converged)"
      return 0
    fi
  done
  echo "[proto2d] WARN: no convergence observed (proceeding anyway)"
  return 1
}

txn_lines() {
  if [ -f /tmp/rulesync_txn.log ]; then
    wc -l < /tmp/rulesync_txn.log | tr -d ' '
  else
    echo 0
  fi
}

# run_cell <cell> <op> <crash> <restart> <epoch>
run_cell() {
  local CELL=$1 OP=$2 CRASH=$3 RESTART=$4 EP=$5
  if [ -n "${CELL_RE:-}" ]; then
    case "$CELL" in
      $CELL_RE) ;;
      *) echo "[proto2d] ----- $CELL skipped by CELL_RE" | tee -a $LOG
        return 0 ;;
    esac
  fi
  echo "[proto2d] ===== $CELL op=$OP crash=$CRASH restart=$RESTART ep=$EP =====" | tee -a $LOG
  if ! kill_procs; then
    echo "[proto2d] $CELL HYGIENE FAIL — skipping cell" | tee -a $LOG
    return 0
  fi
  # Fresh durable state per cell (isolated, deterministic epochs).
  # Also drop any stale bridge arm file (a leftover must never crash a
  # fresh incarnation on its first commit, e.g. reconcile refresh).
  rm -f /tmp/rulesync_state.json /tmp/rulesync_txn.log /tmp/rulesync_wal.jsonl
  rm -f /tmp/nav2_bridge_crash
  restart_ds
  local MENV="-" BENV="-"
  # crash hooks arm the process that must die at that point
  case "$CRASH" in
    C0) MENV="pre-wal" ;;
    C1) MENV="post-wal" ;;
    C3) MENV="post-dispatch" ;;
    C2) BENV="post-apply" ;;
  esac
  # nav2_bridge rides in the launch; for C2 arm the bridge hook via env
  if [ "$BENV" != "-" ]; then
    export RULESYNC_BRIDGE_CRASH_AT="$BENV"
  else
    unset RULESYNC_BRIDGE_CRASH_AT
  fi
  # NOTE: crash hooks stay DISARMED for the initial start. Crash cells
  # arm them only after the setup phase (arm-restart below): arming from
  # process start would crash the SETUP commit instead of the TEST txn
  # (verified failure mode: setup-consumed crash + driver hang).
  unset RULESYNC_MGR_CRASH_AT RULESYNC_BRIDGE_CRASH_AT
  start_mgr "-"
  if ! preflight_singletons; then
    echo "[proto2d] $CELL PREFLIGHT FAIL — skipping cell" | tee -a $LOG
    return 0
  fi
  wait_ready
  if [ "$CRASH" = "none" ]; then
    # Baseline / restart-only: legacy one-shot (hooks disarmed).
    timeout 600 python3 $CK/rulesync3d/tools/proto2d_op.py \
      --uid "$CELL" --op "$OP" --epoch $EP --out $OUT/matrix2d_rows.jsonl --expect /tmp/proto2d_expect_$CELL.json \
      >> $LOG 2>&1 || echo "[proto2d] $CELL driver exit: $?" | tee -a $LOG
  else
    # Crash cell: setup (DISARMED) -> arm-restart (ARMED, durable state
    # preserved) -> test (crash fires in the TEST txn).
    timeout 600 python3 $CK/rulesync3d/tools/proto2d_op.py \
      --uid "$CELL" --op "$OP" --epoch $EP --out $OUT/matrix2d_rows.jsonl --expect /tmp/proto2d_expect_$CELL.json \
      --phase setup \
      >> $LOG 2>&1 || echo "[proto2d] $CELL setup exit: $?" | tee -a $LOG
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
        # File-based arm (NO restart, NO discovery): same rationale as
        # the 3D runner (restart races refresh; param arming flaky).
        # The bridge consumes the file on fire; runner removes it at
        # cell start + recovery.
        echo "post-apply" > /tmp/nav2_bridge_crash
        echo "[proto2d] $CELL bridge armed via file" | tee -a $LOG
        sleep 5
        ;;
    esac
    if ! preflight_singletons; then
      echo "[proto2d] $CELL ARM-RESTART PREFLIGHT FAIL — retrying once after settle" | tee -a $LOG
      sleep 15
      if ! preflight_singletons; then
        echo "[proto2d] $CELL ARM-RESTART PREFLIGHT FAIL — skipping cell" | tee -a $LOG
        return 0
      fi
    fi
    if [ "$CRASH" = "C2" ]; then
      # No restart happened (runtime arm): still converged from setup.
      sleep 5
    else
      # Gate on post-arm-restart convergence (fresh records only).
      wait_ready_from "$ARM_FROM"
    fi
    timeout 600 python3 $CK/rulesync3d/tools/proto2d_op.py \
      --uid "$CELL" --op "$OP" --epoch $EP --out $OUT/matrix2d_rows.jsonl --expect /tmp/proto2d_expect_$CELL.json \
      --phase test \
      >> $LOG 2>&1 || echo "[proto2d] $CELL driver exit: $?" | tee -a $LOG
  fi
  # restart phase per mode (manager and/or bridge fresh, NO hooks)
  unset RULESYNC_MGR_CRASH_AT RULESYNC_BRIDGE_CRASH_AT
  rm -f /tmp/nav2_bridge_crash
  sleep 2
  T0=$(date +%s)
  REC_FROM=$(txn_lines)
  # NOTE: no DS bounce at restart points (verified harmful: survivors
  # of a server bounce never re-register — permanently isolated).
  # Ghost-lease flakes are absorbed by driver op retry + long gates.
  case "$RESTART" in
    mgr) for pat in '[r]ulesync_manager'; do pkill -9 -f "$pat" 2>/dev/null; done; sleep 3; start_mgr_standalone "-" ;;
    bridge) for pat in '[r]ulesync_nav2_bridge'; do pkill -9 -f "$pat" 2>/dev/null; done; sleep 3; start_bridge2d "-" ;;
    both) kill_procs; unset RULESYNC_BRIDGE_CRASH_AT; restart_ds; start_mgr_standalone "-"; start_bridge2d "-" ;;
    none) ;;
  esac
  if [ "$CRASH" = "C2" ]; then
    # The crashed bridge must be revived regardless of restart mode.
    case "$RESTART" in
      bridge|both) ;;
      *) for pat in '[r]ulesync_nav2_bridge'; do pkill -9 -f "$pat" 2>/dev/null; done; sleep 3; start_bridge2d "-" ;;
    esac
  fi
  # settle: allow reconcile cycles to converge (or reveal stuck DEGRADED)
  sleep 45
  wait_ready_from "$REC_FROM" || true
  # audit the cell (reads WAL/snapshot/services, writes matrix row)
  # Generous wrapper (600 s): service waits retry through transient
  # post-restart discovery sickness; the audit itself bounds each step.
  timeout 600 python3 $CK/rulesync3d/tools/proto2d_audit.py \
    --uid "$CELL" --op "$OP" --crash "$CRASH" --restart "$RESTART" \
    --epoch $EP --t0 $T0 --out $OUT/matrix2d_rows.jsonl \
    --expect /tmp/proto2d_expect_$CELL.json >> $LOG 2>&1 \
    || echo "[proto2d] $CELL audit exit: $?" | tee -a $LOG
  # Archive per-cell durable records (state/txn/WAL are wiped next cell;
  # epochs are per-cell unique, so rows join without inline tags).
  cat /tmp/rulesync_txn.log 2>/dev/null >> $OUT/txn2d_all.jsonl
  cat /tmp/rulesync_wal.jsonl 2>/dev/null >> $OUT/wal2d_all.jsonl
}

EP=400000
# crash cells: manager restart primary (C2 restarts the crashed bridge)
for OP in ADD REMOVE RELAX; do
  for C in C0 C1 C3; do
    run_cell "2d-$OP-$C" "$OP" "$C" "mgr" $EP; EP=$((EP+1000))
  done
  run_cell "2d-$OP-C2" "$OP" "C2" "bridge" $EP; EP=$((EP+1000))
done
# restart-only cells (committed rule, no crash during txn)
for R in mgr bridge both; do
  run_cell "2d-restart-$R" "SETUP_ONLY" "none" "$R" $EP; EP=$((EP+1000))
done
echo "[proto2d] matrix done." | tee -a $LOG
# compile CSVs
python3 - "$OUT" <<'PYEOF'
import json, sys, csv
out = sys.argv[1]
seen = {}
for l in open(f"{out}/matrix2d_rows.jsonl"):
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
with open(f"{out}/protocol_matrix_2d.csv","w",newline="") as f:
    w = csv.DictWriter(f, fieldnames=mkeys)
    w.writeheader()
    for r in rows:
        w.writerow({k: r.get(k) for k in mkeys})
print(f"matrix2d rows: {len(rows)}")
bad = [r["uid"] for r in rows if r.get("unauthorized_motion") or
       r.get("silent_state_divergence") or
       r.get("committed_update_lost") or
       r.get("uncommitted_relaxation_promoted")]
print("HARD-FAIL rows:", bad if bad else "none (all four properties zero)")
inv = [r["uid"] for r in rows if not r.get("hook_verified")]
print("HOOK-UNVERIFIED rows:", inv if inv else "none (all cells valid)")
print("GATE:", "CLOSED" if not bad and not inv and rows else "OPEN")
PYEOF
