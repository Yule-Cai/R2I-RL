#!/bin/bash
# Per-map TopoRulePatch data campaign step: (re)start Nav2 stack for MAPKEY,
# then run the teacher generator in background.
# Usage: run_map.sh <mapkey> <n_target> [clear_m] [plan_timeout] [dist_max]
set -u
MAPKEY="$1"
NTARGET="$2"
CLEAR_M="${3:-0.5}"
PLAN_TMO="${4:-30.0}"
DIST_MAX="${5:-12.0}"
export ROS_DOMAIN_ID=7
CK=~/R2I-RL-checkout
MAPDIR=~/rulepatch/topo_maps
OUTDIR=~/rulepatch/topo_raw_${MAPKEY}
LOG=~/rulepatch/logs/gen_topo_${MAPKEY}.log

pkill -9 -f "bringup_topo|planner_server|map_server|controller_server|lifecycle_manager|gen_topo_dataset" 2>/dev/null
sleep 5
# hard guarantee: no stale Nav2 publishers (a leftover map_server on the
# same domain silently corrupts planning via the latched /map race).
left=$(pgrep -f "nav2_planner/planner_server|nav2_map_server/map_server|nav2_controller/controller_server" | head)
if [ -n "$left" ]; then
  echo "[campaign] ERROR: stale Nav2 survivors: $left" >&2
  pkill -9 -f "nav2_planner|nav2_map_server|nav2_controller" 2>/dev/null
  sleep 5
  left=$(pgrep -f "nav2_planner/planner_server|nav2_map_server/map_server|nav2_controller/controller_server" | head)
  [ -n "$left" ] && { echo "[campaign] FATAL: $left" >&2; exit 1; }
fi
set +u
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
set -u
cd "$CK" || exit 1
bash topopatch/tools/bringup_topo.sh "$MAPKEY" > ~/rulepatch/logs/bringup_${MAPKEY}.log 2>&1 &
echo "[campaign] bringup for $MAPKEY launched, waiting for planner action..."
for _ in $(seq 1 36); do
  if timeout 10 ros2 action list 2>/dev/null | grep -q compute_path_to_pose; then
    echo "[campaign] planner action available"
    break
  fi
  sleep 5
done
timeout 10 ros2 action list 2>/dev/null | grep -q compute_path_to_pose \
  || { echo "[campaign] ERROR: planner action never appeared"; exit 1; }
# exactly one /map publisher required (two map_servers = latched-map race)
npub=$(timeout 10 ros2 topic info /map 2>/dev/null | grep -i "publisher count" | grep -o "[0-9]*")
if [ "$npub" != "1" ]; then
  echo "[campaign] ERROR: /map publishers=$npub (want 1)" >&2
  exit 1
fi
echo "[campaign] stack sane (/map x1, planner up)"
setsid nohup python3 -m topopatch.tools.gen_topo_dataset \
  --map-id "$MAPKEY" --map-pgm "$MAPDIR/${MAPKEY}.pgm" \
  --map-res 0.05 --map-ox -10 --map-oy -10 --clear-m "$CLEAR_M" \
  --n-target "$NTARGET" --seed 101 --shard-every 100 \
  --plan-timeout "$PLAN_TMO" --dist-max "$DIST_MAX" \
  --out-dir "$OUTDIR" > "$LOG" 2>&1 < /dev/null &
echo "[campaign] gen for $MAPKEY (n=$NTARGET) launched, log=$LOG"
