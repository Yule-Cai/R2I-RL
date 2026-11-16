#!/bin/bash
# Standalone bench stack (sandbox headless): TF + map + planner +
# manager/bridge + scan stub. Idempotent-ish: kills its own kind first.
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export ROS_DISABLE_ROS2_DAEMON=1
export PYTHONDONTWRITEBYTECODE=1
set +u
source /opt/ros/lyrical/setup.bash
source ~/rulesync_ws/install/setup.bash
set -u
export PYTHONPATH=/home/yule/R2I-RL-checkout:/home/yule/R2I-RL-checkout/rulepatch:/home/yule/R2I-RL-checkout/topopatch_ros:${PYTHONPATH:-}
CK=~/R2I-RL-checkout
for pat in '[m]ap_server' '[p]lanner_server' '[r]ulesync_manager' '[r]ulesync_nav2_bridge' '[s]can_stub'; do
  pkill -9 -f "$pat" 2>/dev/null || true
done
sleep 3
pkill -9 -f '[f]astdds(\.py)? discovery|[f]ast-discovery-server' 2>/dev/null || true
sleep 2
bash -c 'source /opt/ros/lyrical/setup.bash && setsid nohup fastdds discovery -i 0 -l 127.0.0.1 -p 11811 > /tmp/ds_bench.log 2>&1 < /dev/null & echo bg' >/dev/null 2>&1 || true
sleep 2
cd $CK
SANDBOX_YAML=/opt/ros/lyrical/share/nav2_bringup/maps/tb3_sandbox.yaml
python3 topopatch/tools/paper_planner_cfg.py --pgm-yaml $SANDBOX_YAML \
  --plugins GridBased --out /tmp/paper_planner_sb.yaml
setsid nohup ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom \
  --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 > /tmp/bench_tf.log 2>&1 < /dev/null &
setsid nohup ros2 run tf2_ros static_transform_publisher --frame-id odom --child-frame-id base_link \
  --x -2 --y -0.5 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 >> /tmp/bench_tf.log 2>&1 < /dev/null &
setsid nohup ros2 run nav2_map_server map_server --ros-args -p use_sim_time:=False \
  --params-file /tmp/paper_planner_sb.yaml > /tmp/bench_map.log 2>&1 < /dev/null &
setsid nohup ros2 run nav2_planner planner_server --ros-args -p use_sim_time:=False \
  --params-file /tmp/paper_planner_sb.yaml > /tmp/bench_planner.log 2>&1 < /dev/null &
sleep 8
timeout 120 python3 rulepatch/tools/activate_nav2.py --timeout 100 \
  --nodes map_server,planner_server 2>&1 | tail -1
setsid nohup bash -c 'ros2 launch rulesync_demo rulesync_sim.launch.py' > /tmp/bench_nodes.log 2>&1 < /dev/null &
sleep 8
setsid nohup python3 -m topopatch_ros.scan_stub > /tmp/bench_scan.log 2>&1 < /dev/null &
sleep 3
echo BENCH_STACK_READY
