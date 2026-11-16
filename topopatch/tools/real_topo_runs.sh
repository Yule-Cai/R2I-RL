#!/bin/bash
# M4 physical-robot supervised runbook (FIVE runs, NO autonomous motion).
#
# HUMAN OPERATOR IS REQUIRED BESIDE THE ROBOT WITH E-STOP FOR EVERY STEP.
# This script only starts/stops nodes and runs scripted cases; the robot
# moves ONLY while the operator supervises. Abort anytime with Ctrl-C
# (nodes die, pp_driver stops publishing, robot halts) or the e-stop.
#
# Prerequisites (operator):
#   1. Real-robot Nav2 stack running (rulesync_real.launch.py), localized.
#   2. Open space (>= 4 m clearance around the planned corridor).
#   3. Phone/camera recording video (script cannot do this).
#   4. e-stop reachable. Speed capped at 0.25 m/s below.
#
# The five runs (epochs 201-205); outcomes RECORDED AS OBSERVED
# (no open-space acceptance assumption):
#   1. no-rule baseline (no commit; plain plan + pp execution).
#   2. local repair accepted (commit rect; expect accept in open space).
#   3. alternate hypothesis accepted (second scene; any accepted hyp>0,
#      or second accepted run — recorded as observed).
#   4. abstain -> verified fallback (any abstain scene; fallback must be
#      gated-clean before pp executes).
#   5. impossible rule (sealing rect across a narrow passage OR huge
#      rect; expect bridge reject (rollback) or all-fail safe stop with
#      abort_recommended; robot must NOT move into the rule).
#
# Usage (on onexplayer, robot network up, default domain):
#   bash real_topo_runs.sh [--runs 1,2,3,4,5] [--goal-x .. --goal-y ..]
# Evidence: ~/rulepatch/real_cases.csv, ~/rulepatch/real_bag_*, video.
WS=~/rulesync_ws
WEIGHTS=~/rulepatch/topo_scope/best_fused.npz
RUNS="1,2,3,4,5"
GOAL_X=""
GOAL_Y=""
for arg in "$@"; do
  case "$arg" in
    --runs=*) RUNS="${arg#*=}" ;;
    --goal-x=*) GOAL_X="${arg#*=}" ;;
    --goal-y=*) GOAL_Y="${arg#*=}" ;;
  esac
done
source /opt/ros/lyrical/setup.bash
source "$WS/install/setup.bash"
export PYTHONPATH=/home/yule/R2I-RL-checkout:/home/yule/R2I-RL-checkout/rulepatch:/home/yule/R2I-RL-checkout/topopatch_ros:${PYTHONPATH:-}
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export ROS_DISABLE_ROS2_DAEMON=1
test -f "$WEIGHTS" || { echo "[real] missing weights: $WEIGHTS"; exit 1; }

echo "================ OPERATOR CHECKLIST ================"
echo " [ ] Beside the robot, e-stop in hand"
echo " [ ] Real-robot Nav2 stack running + localized (see /map, /odom)"
echo " [ ] Space clear, video recording ON"
echo " [ ] Runs to execute: $RUNS"
echo "Press ENTER to start nodes (Ctrl-C aborts anytime)..."
read -r _

echo "[real] starting manager + bridge..."
ros2 launch rulesync_demo rulesync_sim.launch.py > /tmp/real_nodes.log 2>&1 &
NODES_PID=$!
sleep 8

echo "[real] topo_inference (adaptive scope weights, use_scope) + pp (0.25)..."
python3 -c "from topopatch_ros.topo_inference_node import main; main()" \
  --ros-args -p weights_npz:="$WEIGHTS" -p tau:=1.8216713190078737 \
  -p use_scope:=True > /tmp/real_topo.log 2>&1 &
TOPO_PID=$!
"$WS/install/rulepatch_ros/bin/pp_driver" --ros-args -p speed:=0.25 \
  > /tmp/real_driver.log 2>&1 &
PP_PID=$!
sleep 5
python3 ~/R2I-RL-checkout/rulepatch/tools/rp_params.py rulesync_nav2_bridge auto_replan false || true

echo "[real] recording bag..."
rm -rf ~/rulepatch/real_bag
ros2 bag record -o ~/rulepatch/real_bag /cmd_vel /odom /scan /plan \
  /rulepatch/repaired_path /rulepatch/status /rulesync/keepout_mask \
  /rulesync/staged_set > /tmp/real_bag.log 2>&1 &
BAG_PID=$!
sleep 2

IFS=',' read -ra RUNLIST <<< "$RUNS"
# Run semantics: 1=baseline(no rule), 2-4=rule scenes (recorded as
# observed: accept / alternate-hyp accept / abstain-fallback),
# 5=impossible rule (sealing: expect bridge reject or safe stop).
# Goals default to the operator-provided --goal-x/--goal-y; when absent
# the driver reuses its scripted sandbox goals — OPERATOR MUST PASS GOALS
# for the real site.
if [ -z "$GOAL_X" ] || [ -z "$GOAL_Y" ]; then
  echo "[real] FATAL: pass --goal-x/--goal-y for the real site."
  exit 1
fi
for r in "${RUNLIST[@]}"; do
  echo "================ RUN $r ================"
  echo "Operator: confirm space clear + video on. ENTER to run, Ctrl-C to abort all."
  read -r _
  case "$r" in
    1) EP=201; ARGS="--single-goal $GOAL_X,$GOAL_Y --single-epoch $EP --no-rule" ;;
    5) EP=250; ARGS="--single-goal $GOAL_X,$GOAL_Y --single-epoch $EP --rect-half 2.0" ;;
    *) EP=$((200 + r * 10)); ARGS="--single-goal $GOAL_X,$GOAL_Y --single-epoch $EP" ;;
  esac
  python3 -m topopatch_ros.topo_case_driver --csv ~/rulepatch/real_run_${r}.csv $ARGS
  echo "[real] run $r done. Robot should be stationary. ENTER for next."
  read -r _
done

kill $BAG_PID $TOPO_PID $PP_PID $NODES_PID 2>/dev/null || true
echo "[real] done. csvs: ~/rulepatch/real_run_*.csv bag: ~/rulepatch/real_bag"
echo "Operator: stop video, keep e-stop until robot powered down."
