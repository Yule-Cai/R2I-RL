#!/bin/bash
# Debug: sandbox pp tracking trace (run on onexplayer).
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export ROS_DISABLE_ROS2_DAEMON=1
source /opt/ros/lyrical/setup.bash
source ~/rulesync_ws/install/setup.bash
export PYTHONPATH=/home/yule/R2I-RL-checkout:/home/yule/R2I-RL-checkout/rulepatch:/home/yule/R2I-RL-checkout/topopatch_ros:${PYTHONPATH:-}
CK=~/R2I-RL-checkout
SIM_DIR=/opt/ros/lyrical/share/nav2_minimal_tb3_sim
export GZ_SIM_RESOURCE_PATH=$SIM_DIR/models:${GZ_SIM_RESOURCE_PATH:-}
pkill -9 -f '[f]astdds(\.py)? discovery|[f]ast-discovery-server' 2>/dev/null || true
sleep 2
bash -c 'source /opt/ros/lyrical/setup.bash && setsid nohup fastdds discovery -i 0 -l 127.0.0.1 -p 11811 > /tmp/ds_dbg.log 2>&1 < /dev/null & echo bg' >/dev/null 2>&1 || true
sleep 2
xacro -o /tmp/dbg_world.sdf headless:=True "$SIM_DIR/worlds/tb3_sandbox.sdf.xacro"
setsid nohup xvfb-run -a gz sim -r -s /tmp/dbg_world.sdf > /tmp/dbg_sim.log 2>&1 < /dev/null &
sleep 12
setsid nohup ros2 launch nav2_minimal_tb3_sim spawn_tb3.launch.py \
  robot_name:=turtlebot3_waffle robot_sdf:="$SIM_DIR/urdf/gz_waffle.sdf.xacro" \
  x_pose:=-2.0 y_pose:=-0.5 z_pose:=0.01 > /tmp/dbg_spawn.log 2>&1 < /dev/null &
setsid nohup ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p use_sim_time:=True \
  -p robot_description:="$(cat $SIM_DIR/urdf/turtlebot3_waffle.urdf)" \
  > /tmp/dbg_rsp.log 2>&1 < /dev/null &
sleep 8
printf '/**:\n  ros__parameters:\n    yaml_filename: "/opt/ros/lyrical/share/nav2_bringup/maps/tb3_sandbox.yaml"\n    topic_name: "map"\n    frame_id: "map"\n' > /tmp/dbg_map.yaml
setsid nohup ros2 run nav2_map_server map_server --ros-args -p use_sim_time:=True \
  --params-file /tmp/dbg_map.yaml > /tmp/dbg_map.log 2>&1 < /dev/null &
setsid nohup ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom \
  --x -2.0 --y -0.5 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --ros-args -p use_sim_time:=True \
  > /tmp/dbg_tf.log 2>&1 < /dev/null &
setsid nohup ros2 run nav2_planner planner_server --ros-args -p use_sim_time:=True \
  --params-file $CK/topopatch/nav2_params_topopatch.yaml > /tmp/dbg_planner.log 2>&1 < /dev/null &
sleep 8
python3 $CK/rulepatch/tools/activate_nav2.py --timeout 100 --nodes map_server,planner_server
setsid nohup ros2 launch rulesync_demo rulesync_sim.launch.py > /tmp/dbg_nodes.log 2>&1 < /dev/null &
sleep 6
setsid nohup python3 -c "from topopatch_ros.topo_inference_node import main; main()" \
  --ros-args -p weights_npz:=/home/yule/rulepatch/topo_scope/best_fused.npz \
  -p tau:=1.8216713190078737 -p use_scope:=True > /tmp/dbg_topo.log 2>&1 < /dev/null &
setsid nohup python3 -c "from topopatch_ros.gate_node import main; main()" \
  > /tmp/dbg_gate.log 2>&1 < /dev/null &
setsid nohup python3 -c "from rulepatch_ros.pp_driver import main; main()" --ros-args \
  -r /rulepatch/repaired_path:=/rulepatch/gated_path -p speed:=0.35 \
  > /tmp/dbg_pp.log 2>&1 < /dev/null &
sleep 4
echo STACK_READY
