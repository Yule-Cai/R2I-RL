#!/bin/bash
# Blocker C: contact-instrumented physics on the pre-registered subset.
# Usage: run_contact.sh <maze|narrow>
# Subset (NOT by success): manifest order every 2nd (maze) / every 3rd
# (narrow), plus the 4 maze stationary-safe-stop uids for failure-domain
# coverage. Contact sensor injected into the robot SDF + bridged.
WHICH=$1
if [ "$WHICH" = "maze" ]; then
  TAG=topo_ext_maze-B-s202
  SDF=/home/yule/rulepatch/paper/world_maze-B-s202.sdf
  MAPYAML=/home/yule/rulepatch/topo_maps/maze-B-s202.yaml
  SX=7.38; SY=1.43
else
  TAG=topo_ext_narrow-B-s302
  SDF=/home/yule/rulepatch/paper/world_narrow-B-s302.sdf
  MAPYAML=/home/yule/rulepatch/topo_maps/narrow-B-s302.yaml
  SX=-6.92; SY=-7.38
fi
CK=/home/yule/R2I-RL-checkout
PAPER=/home/yule/rulepatch/paper
LOG=$PAPER/logs
WEIGHTS=/home/yule/rulepatch/topo_scope/best_fused.npz
TAU_SCOPE=1.8216713190078737
SIM_DIR=/opt/ros/lyrical/share/nav2_minimal_tb3_sim
export GZ_SIM_RESOURCE_PATH=$SIM_DIR/models:${GZ_SIM_RESOURCE_PATH:-}
export PYTHONPATH=/home/yule/R2I-RL-checkout:/home/yule/R2I-RL-checkout/rulepatch:/home/yule/R2I-RL-checkout/topopatch_ros:${PYTHONPATH:-}
export PYTHONDONTWRITEBYTECODE=1
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export ROS_DISABLE_ROS2_DAEMON=1
set +u
source /opt/ros/lyrical/setup.bash
source /home/yule/rulesync_ws/install/setup.bash
set -u
for pat in '[g]z-sim-main' '[g]z sim' '[n]av2_container' '[p]lanner_server' \
           '[m]ap_server' '[c]ontroller_server' '[a]mcl' '[l]ifecycle_manager' \
           '[r]ulesync_manager' '[r]ulesync_nav2_bridge' '[t]opo_inference' \
           '[g]ate_node' '[p]p_driver' '[t]opo_case_driver' '[b]ench_client' \
           '[f]ault_client' '[n]ative_baseline' '[r]obot_state_publisher' \
           '[p]arameter_bridge' '[r]os2 launch' '[r]os2 run' \
           '[s]tatic_transform_publisher' '[s]can_stub' '[x]vfb-run'; do
  pkill -9 -f "$pat" 2>/dev/null || true
done
pkill -9 -f '[f]astdds(.py)? discovery|[f]ast-discovery-server' 2>/dev/null || true
sleep 4
rm -f /dev/shm/fastdds_* /dev/shm/sem.fastdds_* 2>/dev/null || true
bash -c 'source /opt/ros/lyrical/setup.bash && setsid nohup fastdds discovery -i 0 -l 127.0.0.1 -p 11811 > /tmp/ds_contact.log 2>&1 < /dev/null & echo bg' >/dev/null 2>&1 || true
sleep 5
# Fresh epoch space per campaign: the manager snapshot persists across
# restarts by design (durability); a stale snapshot would stale-reject
# this campaign's lower epochs.
rm -f /tmp/rulesync_state.json
cd $CK
# contact sensor SDF: xacro-generate base, then inject the sensor
xacro $SIM_DIR/urdf/gz_waffle.sdf.xacro -o /tmp/waffle_base.sdf 2>&1 | tail -1 || \
  cp $SIM_DIR/urdf/gz_waffle.sdf /tmp/waffle_base.sdf
python3 topopatch/tools/inject_contact.py \
  /tmp/waffle_base.sdf /tmp/waffle_contact.sdf 2>&1 | tail -1
setsid nohup xvfb-run -a gz sim -r -s "$SDF" > $LOG/CONT_${TAG}_sim.log 2>&1 < /dev/null &
sleep 12
setsid nohup ros2 launch nav2_minimal_tb3_sim spawn_tb3.launch.py \
  robot_name:=turtlebot3_waffle robot_sdf:="/tmp/waffle_contact.sdf" \
  x_pose:=$SX y_pose:=$SY z_pose:=0.01 > $LOG/CONT_${TAG}_spawn.log 2>&1 < /dev/null &
setsid nohup ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p use_sim_time:=True \
  -p robot_description:="$(cat $SIM_DIR/urdf/turtlebot3_waffle.urdf)" \
  > $LOG/CONT_${TAG}_rsp.log 2>&1 < /dev/null &
for i in $(seq 1 24); do
  if timeout 20 python3 $CK/rulepatch/tools/wait_topic.py /odom --timeout 15 2>/dev/null | grep -q flowing; then break; fi
  sleep 5
done
sleep 3
python3 $CK/topopatch/tools/paper_planner_cfg.py --pgm-yaml $MAPYAML \
  --plugins GridBased --out /tmp/cont_planner_$TAG.yaml
cat > /tmp/cont_map_$TAG.yaml <<EOF
/**:
  ros__parameters:
    yaml_filename: "$MAPYAML"
    topic_name: "map"
    frame_id: "map"
EOF
setsid nohup ros2 run nav2_map_server map_server --ros-args -p use_sim_time:=True \
  --params-file /tmp/cont_map_$TAG.yaml > $LOG/CONT_${TAG}_map.log 2>&1 < /dev/null &
setsid nohup ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom \
  --x $SX --y $SY --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --ros-args -p use_sim_time:=True \
  > $LOG/CONT_${TAG}_tf.log 2>&1 < /dev/null &
setsid nohup ros2 run nav2_planner planner_server --ros-args -p use_sim_time:=True \
  --params-file /tmp/cont_planner_$TAG.yaml > $LOG/CONT_${TAG}_planner.log 2>&1 < /dev/null &
sleep 8
timeout 150 python3 $CK/rulepatch/tools/activate_nav2.py --timeout 140 \
  --nodes map_server,planner_server > $LOG/CONT_${TAG}_act.log 2>&1 || true
setsid nohup bash -c 'source /opt/ros/lyrical/setup.bash; source ~/rulesync_ws/install/setup.bash; export ROS_DISCOVERY_SERVER=127.0.0.1:11811 ROS_DISABLE_ROS2_DAEMON=1; ros2 launch rulesync_demo rulesync_sim.launch.py' > $LOG/CONT_${TAG}_nodes.log 2>&1 < /dev/null &
sleep 8
setsid nohup python3 -c "from topopatch_ros.topo_inference_node import main; main()" \
  --ros-args -p weights_npz:=$WEIGHTS -p tau:=$TAU_SCOPE -p use_scope:=True \
  > $LOG/CONT_${TAG}_topo.log 2>&1 < /dev/null &
setsid nohup python3 -c "from topopatch_ros.gate_node import main; main()" \
  > $LOG/CONT_${TAG}_gate.log 2>&1 < /dev/null &
setsid nohup python3 -c "from rulepatch_ros.pp_driver import main; main()" --ros-args \
  -r /rulepatch/repaired_path:=/rulepatch/gated_path -p speed:=0.25 \
  > $LOG/CONT_${TAG}_pp.log 2>&1 < /dev/null &
setsid nohup bash -c 'source /opt/ros/lyrical/setup.bash; ros2 run ros_gz_bridge parameter_bridge --ros-args --params-file /tmp/contact_bridge.yaml' > $LOG/CONT_${TAG}_contact_bridge.log 2>&1 < /dev/null &
sleep 5
for i in 1 2 3 4 5 6; do
  if python3 $CK/rulepatch/tools/rp_params.py rulesync_nav2_bridge auto_replan false 2>&1 | grep -q True; then break; fi
  sleep 10
done
echo "[contact] GZ contact topic check (best effort)..."
timeout 20 gz topic -e /base_contact -n 1 2>&1 | head -3 || echo "[contact] gz-cli check skipped"
# pre-registered subset (NOT by success): manifest order every 2nd
# (maze) / every 3rd (narrow); maze adds the 4 stationary-safe-stop
# uids for failure-domain coverage.
python3 -c "import json,csv;m=json.load(open('/home/yule/rulepatch/repairable_manifest.json'));cs=[c for c in m['cases'] if c['map_tag']=='$TAG'];sub=cs[::2] if '$TAG'.startswith('topo_ext_maze') else cs[::3];subids=set(c['uid'] for c in sub);extra=[]
for f in ['rep_driver_topo_ext_maze-B-s202','rep_driver_topo_ext_narrow-B-s302']:
    try:
        rows=list(csv.DictReader(open('/home/yule/rulepatch/paper/%s.csv'%f)))
    except Exception: continue
    for r in rows:
        if r.get('rule_compliant')=='False' and r.get('uid','') not in subids:
            hit=[c for c in cs if c['uid']==r['uid']]
            if hit: extra.append(hit[0]); subids.add(r['uid'])
sub=sub+extra;json.dump(sub,open('/tmp/contact_cases_$TAG.json','w'));json.dump([{'epoch':c['epoch'],'goal':c['goal'],'rules':c['rules'],'uid':c['uid']} for c in sub],open('/tmp/contact_rules_$TAG.json','w'));print('contact subset:',len(sub),'extra:',len(extra))"
TOPO_SPAWN="$SX,$SY" TOPO_MAP="$TAG" timeout 12000 python3 -m topopatch_ros.topo_case_driver \
  --csv $PAPER/contact_$TAG.csv --planner GridBased \
  --cases-file /tmp/contact_cases_$TAG.json \
  --cases /tmp/contact_rules_$TAG.json --exec pp \
  > $LOG/CONT_${TAG}_driver.log 2>&1 || echo "[contact] driver exit: $?"
echo "[contact] physics $TAG done."
