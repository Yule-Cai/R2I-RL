#!/bin/bash
# Repairable-benchmark live campaign (frozen manifest, scope arm).
# Headless bench per map (exact frozen start/goal/rules) + Gazebo physics
# per map world (same rules/goals, live starts recorded). Gate enforced.
# Usage: repairable_run.sh <manifest.json> [headless|physics|all]
MANIFEST="$1"
MODE="${2:-all}"
CK=~/R2I-RL-checkout
PAPER=~/rulepatch/paper
LOG=$PAPER/logs
WEIGHTS=~/rulepatch/topo_scope/best_fused.npz
TAU_SCOPE=1.8216713190078737
export PYTHONPATH=/home/yule/R2I-RL-checkout:/home/yule/R2I-RL-checkout/rulepatch:/home/yule/R2I-RL-checkout/topopatch_ros:${PYTHONPATH:-}
export PYTHONDONTWRITEBYTECODE=1
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export ROS_DISABLE_ROS2_DAEMON=1
set +u
source /opt/ros/lyrical/setup.bash
source ~/rulesync_ws/install/setup.bash
set -u

kill_ros() {
  # NOTE: must also cover '[r]os2 run' wrappers AND the binaries they
  # spawn ('[s]tatic_transform_publisher', '[s]can_stub'): killing only
  # the wrapper orphans the binary, which then accumulates across runs
  # (observed: 30+ stale TF publishers + duplicate scan_stubs poisoning
  # the ROS graph during the narrow-B campaign).
  for pat in '[g]z-sim-main' '[g]z sim' '[n]av2_container' '[p]lanner_server' \
             '[m]ap_server' '[c]ontroller_server' '[a]mcl' '[l]ifecycle_manager' \
             '[r]ulesync_manager' '[r]ulesync_nav2_bridge' '[t]opo_inference' \
             '[g]ate_node' '[p]p_driver' '[t]opo_case_driver' '[b]ench_client' \
             '[r]obot_state_publisher' '[p]arameter_bridge' '[r]os2 launch' \
             '[r]os2 run' '[s]tatic_transform_publisher' '[s]can_stub' \
             '[x]vfb-run'; do
    pkill -9 -f "$pat" 2>/dev/null || true
  done
  sleep 4
  for _ in $(seq 1 5); do
    left=$(pgrep -f '[r]ulesync_manager|[n]av2_bridge|[m]ap_server|[p]lanner_server' | head)
    [ -z "$left" ] && break
    sleep 2
  done
}
export -f kill_ros 2>/dev/null || true

ds_up() {
  # Kill stale discovery servers first: a second server cannot bind
  # port 11811 and dies silently (|| true), leaving flaky multicast
  # discovery (observed: 261/261 silent plan failures).
  pkill -9 -f '[f]astdds(\.py)? discovery|[f]ast-discovery-server' 2>/dev/null || true
  sleep 2
  bash -c 'source /opt/ros/lyrical/setup.bash && setsid nohup fastdds discovery -i 0 -l 127.0.0.1 -p 11811 > /tmp/ds_rep.log 2>&1 < /dev/null & echo bg' >/dev/null 2>&1 || true
  sleep 3
  if pgrep -f '[f]astdds(\.py)? discovery|[f]ast-discovery-server' >/dev/null && ! grep -q "wasn't able to allocate" /tmp/ds_rep.log 2>/dev/null; then
    echo "[ds] discovery server up"
  else
    echo "[ds] FATAL: discovery server not listening" >&2
    return 1
  fi
}

# Preflight: fail loudly (not silently) when the stack is unusable.
# Checks the plan action server is discoverable AND /map carries data
# AND one smoke plan returns >=4 poses.
# DDS lesson (narrow-B campaign): create ALL entities up front before
# waiting for anything. Creating the /map subscription first and the
# action client 15 s later (or vice versa) intermittently wedges the
# second entity's discovery (60 s wait fails); creating both up front
# then waiting succeeds in <2 s on the same stack. Do not split
# creation/wait phases.
preflight() { # <sx> <sy> <gx> <gy>
  local SX=$1 SY=$2 GX=$3 GY=$4
  timeout 120 python3 - "$SX" "$SY" "$GX" "$GY" <<'PYEOF' 2>&1 | tail -1
import sys, time, rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy
sx, sy, gx, gy = (float(v) for v in sys.argv[1:5])
rclpy.init()
n = rclpy.create_node('rep_preflight')
ac = ActionClient(n, ComputePathToPose, 'compute_path_to_pose')
got = {}
n.create_subscription(
    OccupancyGrid, '/map', lambda m: got.__setitem__('map', m),
    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
ok = ac.wait_for_server(timeout_sec=60.0)
if not ok:
    print("PREFLIGHT no-action-server")
    n.destroy_node(); rclpy.shutdown()
    sys.exit(0)
t0 = time.time()
while time.time() - t0 < 20.0 and 'map' not in got:
    rclpy.spin_once(n, timeout_sec=0.5)
m = got.get('map')
if m is None or int(m.info.width) < 10 or not any(
        v > 0 for v in list(m.data)[::97]):
    print("PREFLIGHT map-missing")
    n.destroy_node(); rclpy.shutdown()
    sys.exit(0)
def ps(p):
    m = PoseStamped(); m.header.frame_id = 'map'
    m.pose.position.x, m.pose.position.y = p[0], p[1]
    m.pose.orientation.w = 1.0
    return m
nposes = 0
if ok:
    g = ComputePathToPose.Goal()
    g.start, g.goal = ps((sx, sy)), ps((gx, gy))
    g.planner_id = 'GridBased'; g.use_start = True
    f = ac.send_goal_async(g)
    rclpy.spin_until_future_complete(n, f, timeout_sec=20.0)
    gh = f.result()
    if gh is not None and gh.accepted:
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(n, rf, timeout_sec=20.0)
        r = rf.result()
        nposes = len(r.result.path.poses) if r is not None else 0
print(f"PREFLIGHT nposes={nposes}")
n.destroy_node(); rclpy.shutdown()
PYEOF
}

# headless bench for one map tag: needs PGM + res/ox/oy + cases subset
run_headless() { # <map_tag> <pgm> <res> <ox> <oy> <cases_json>
  local TAG=$1 PGM=$2 RES=$3 OX=$4 OY=$5 CASES=$6
  echo "[rep] headless $TAG..."
  kill_ros
  ds_up
  cd $CK
  python3 topopatch/tools/paper_planner_cfg.py --pgm-yaml ${PGM%.pgm}.yaml \
    --plugins GridBased --out /tmp/rep_planner_$TAG.yaml
  SX=$(python3 -c "import json;print(json.load(open('$CASES'))['cases'][0]['start'][0])")
  SY=$(python3 -c "import json;print(json.load(open('$CASES'))['cases'][0]['start'][1])")
  setsid nohup ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom \
    --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 > $LOG/REP_${TAG}_tf.log 2>&1 < /dev/null &
  setsid nohup ros2 run tf2_ros static_transform_publisher --frame-id odom --child-frame-id base_link \
    --x $SX --y $SY --z 0 --qx 0 --qy 0 --qz 0 --qw 1 >> $LOG/REP_${TAG}_tf.log 2>&1 < /dev/null &
  setsid nohup ros2 run nav2_map_server map_server --ros-args \
    --params-file /tmp/rep_planner_$TAG.yaml > $LOG/REP_${TAG}_map.log 2>&1 < /dev/null &
  setsid nohup ros2 run nav2_planner planner_server --ros-args \
    --params-file /tmp/rep_planner_$TAG.yaml > $LOG/REP_${TAG}_planner.log 2>&1 < /dev/null &
  sleep 8
  timeout 120 python3 rulepatch/tools/activate_nav2.py --timeout 100 \
    --nodes map_server,planner_server > $LOG/REP_${TAG}_act.log 2>&1 || true
  setsid nohup bash -c 'source /opt/ros/lyrical/setup.bash; source ~/rulesync_ws/install/setup.bash; export ROS_DISCOVERY_SERVER=127.0.0.1:11811 ROS_DISABLE_ROS2_DAEMON=1; ros2 launch rulesync_demo rulesync_sim.launch.py' > $LOG/REP_${TAG}_nodes.log 2>&1 < /dev/null &
  sleep 8
  setsid nohup python3 -c "from topopatch_ros.topo_inference_node import main; main()" \
    --ros-args -p weights_npz:=$WEIGHTS -p tau:=$TAU_SCOPE -p use_scope:=True \
    > $LOG/REP_${TAG}_topo.log 2>&1 < /dev/null &
  setsid nohup python3 -c "from topopatch_ros.gate_node import main; main()" \
    > $LOG/REP_${TAG}_gate.log 2>&1 < /dev/null &
  setsid nohup python3 -m topopatch_ros.scan_stub > $LOG/REP_${TAG}_scan.log 2>&1 < /dev/null &
  sleep 4
  PF0=$(python3 -c "import json;d=json.load(open('$CASES'));c=d['cases'][0];print(c['start'][0],c['start'][1],c['goal'][0],c['goal'][1])")
  if ! preflight $PF0 | grep -qE "PREFLIGHT nposes=([4-9]|[0-9]{2,})"; then
    echo "[rep] $TAG PREFLIGHT FAILED — aborting map (no silent rows)" >&2
    return 1
  fi
  timeout 3000 python3 -m topopatch_ros.bench_client --csv $PAPER/rep_bench_$TAG.csv \
    --planner GridBased --map $TAG --cases-file $CASES > $LOG/REP_${TAG}_bench.log 2>&1 || true
  echo "[rep] headless $TAG done."
}

# physics for one map tag: world sdf + spawn + full minimal stack + driver
run_physics() { # <map_tag> <world_sdf|--sandbox> <map_yaml> <spawn_x> <spawn_y> <cases_json>
  local TAG=$1 SDF=$2 MAPYAML=$3 SX=$4 SY=$5 CASES=$6
  local SIM_DIR=/opt/ros/lyrical/share/nav2_minimal_tb3_sim
  export GZ_SIM_RESOURCE_PATH=$SIM_DIR/models:${GZ_SIM_RESOURCE_PATH:-}
  echo "[rep] physics $TAG spawn=($SX,$SY)..."
  kill_ros
  ds_up
  cd $CK
  if [ "$SDF" = "--sandbox" ]; then
    xacro -o /tmp/rep_world.sdf headless:=True "$SIM_DIR/worlds/tb3_sandbox.sdf.xacro"
    SDF=/tmp/rep_world.sdf
  fi
  setsid nohup xvfb-run -a gz sim -r -s "$SDF" > $LOG/REP_${TAG}_sim.log 2>&1 < /dev/null &
  sleep 12
  setsid nohup ros2 launch nav2_minimal_tb3_sim spawn_tb3.launch.py \
    robot_name:=turtlebot3_waffle robot_sdf:="$SIM_DIR/urdf/gz_waffle.sdf.xacro" \
    x_pose:=$SX y_pose:=$SY z_pose:=0.01 > $LOG/REP_${TAG}_spawn.log 2>&1 < /dev/null &
  setsid nohup ros2 run robot_state_publisher robot_state_publisher --ros-args \
    -p use_sim_time:=True \
    -p robot_description:="$(cat $SIM_DIR/urdf/turtlebot3_waffle.urdf)" \
    > $LOG/REP_${TAG}_rsp.log 2>&1 < /dev/null &
  for i in $(seq 1 24); do
    if timeout 20 python3 $CK/rulepatch/tools/wait_topic.py /odom --timeout 15 2>/dev/null | grep -q flowing; then break; fi
    sleep 5
  done
  sleep 3
  python3 $CK/topopatch/tools/paper_planner_cfg.py --pgm-yaml $MAPYAML \
    --plugins GridBased --out /tmp/rep_planner_$TAG.yaml
  cat > /tmp/rep_map_$TAG.yaml <<EOF
/**:
  ros__parameters:
    yaml_filename: "$MAPYAML"
    topic_name: "map"
    frame_id: "map"
EOF
  setsid nohup ros2 run nav2_map_server map_server --ros-args -p use_sim_time:=True \
    --params-file /tmp/rep_map_$TAG.yaml > $LOG/REP_${TAG}_map.log 2>&1 < /dev/null &
  setsid nohup ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom \
    --x $SX --y $SY --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --ros-args -p use_sim_time:=True \
    > $LOG/REP_${TAG}_tf.log 2>&1 < /dev/null &
  setsid nohup ros2 run nav2_planner planner_server --ros-args -p use_sim_time:=True \
    --params-file /tmp/rep_planner_$TAG.yaml > $LOG/REP_${TAG}_planner.log 2>&1 < /dev/null &
  sleep 8
  timeout 150 python3 $CK/rulepatch/tools/activate_nav2.py --timeout 140 \
    --nodes map_server,planner_server > $LOG/REP_${TAG}_act.log 2>&1 || true
  setsid nohup bash -c 'source /opt/ros/lyrical/setup.bash; source ~/rulesync_ws/install/setup.bash; export ROS_DISCOVERY_SERVER=127.0.0.1:11811 ROS_DISABLE_ROS2_DAEMON=1; ros2 launch rulesync_demo rulesync_sim.launch.py' > $LOG/REP_${TAG}_nodes.log 2>&1 < /dev/null &
  sleep 8
  setsid nohup python3 -c "from topopatch_ros.topo_inference_node import main; main()" \
    --ros-args -p weights_npz:=$WEIGHTS -p tau:=$TAU_SCOPE -p use_scope:=True \
    > $LOG/REP_${TAG}_topo.log 2>&1 < /dev/null &
  setsid nohup python3 -c "from topopatch_ros.gate_node import main; main()" \
    > $LOG/REP_${TAG}_gate.log 2>&1 < /dev/null &
  setsid nohup python3 -c "from rulepatch_ros.pp_driver import main; main()" --ros-args \
    -r /rulepatch/repaired_path:=/rulepatch/gated_path -p speed:=0.25 \
    > $LOG/REP_${TAG}_pp.log 2>&1 < /dev/null &
  sleep 5
  for i in 1 2 3 4 5 6; do
    if python3 $CK/rulepatch/tools/rp_params.py rulesync_nav2_bridge auto_replan false 2>&1 | grep -q True; then break; fi
    sleep 10
  done
  TOPO_SPAWN="$SX,$SY" TOPO_MAP="$TAG" timeout 5400 python3 -m topopatch_ros.topo_case_driver \
    --csv $PAPER/rep_driver_$TAG.csv --planner GridBased \
    --cases-file $CASES --exec pp > $LOG/REP_${TAG}_driver.log 2>&1 || true
  echo "[rep] physics $TAG done."
}

pgm_for_tag() { # <map_tag> -> prints PGM RES OX OY
  case "$1" in
    *warehouse*) echo "/opt/ros/lyrical/share/nav2_bringup/maps/warehouse.pgm 0.03 -15.1 -25" ;;
    *) echo "$HOME/rulepatch/topo_maps/$2 0.05 -10 -10" ;;
  esac
}

if [ "$MODE" = "headless" ] || [ "$MODE" = "all" ]; then
  for TAG in $(python3 -c "import json;print(' '.join(json.load(open('$MANIFEST'))['maps']))"); do
    PGMF=$(python3 -c "import json;m=json.load(open('$MANIFEST'));print(m['maps']['$TAG']['pgm'])")
    read -r PGM RES OX OY <<< "$(pgm_for_tag "$TAG" "$PGMF")"
    python3 -c "import json;m=json.load(open('$MANIFEST'));cs=[c for c in m['cases'] if c['map_tag']=='$TAG'];json.dump({'cases':cs},open('/tmp/rep_cases_$TAG.json','w'))"
    run_headless "$TAG" "$PGM" "$RES" "$OX" "$OY" /tmp/rep_cases_$TAG.json
  done
fi
if [ "$MODE" = "physics" ] || [ "$MODE" = "all" ]; then
  # Frozen physics table: tag|sdf|map_yaml|spawn. Spawns = max-clearance
  # cells (paper_goals, seed 0); worlds built from the SAME pgms.
  while IFS='|' read -r TAG SDF MAPYAML SX SY; do
    [ -z "$TAG" ] && continue
    python3 -c "import json;m=json.load(open('$MANIFEST'));cs=[c for c in m['cases'] if c['map_tag']=='$TAG'];json.dump({'cases':cs},open('/tmp/rep_cases_$TAG.json','w'));print(len(cs))"
    run_physics "$TAG" "$SDF" "$MAPYAML" "$SX" "$SY" /tmp/rep_cases_$TAG.json
  done <<'WORLDS'
topo_ext_maze-B-s202|/home/yule/rulepatch/paper/world_maze-B-s202.sdf|/home/yule/rulepatch/topo_maps/maze-B-s202.yaml|7.38|1.43
topo_ext_narrow-B-s302|/home/yule/rulepatch/paper/world_narrow-B-s302.sdf|/home/yule/rulepatch/topo_maps/narrow-B-s302.yaml|-6.92|-7.38
WORLDS
fi
echo "[rep] $MODE done."
