#!/bin/bash
# Minimal single-world bringup for actuator diagnosis (no manager/driver).
SIM_DIR=/opt/ros/lyrical/share/nav2_minimal_tb3_sim
export GZ_SIM_RESOURCE_PATH=$SIM_DIR/models:${GZ_SIM_RESOURCE_PATH:-}
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export ROS_DISABLE_ROS2_DAEMON=1
set +u
source /opt/ros/lyrical/setup.bash
source /home/yule/rulesync_ws/install/setup.bash
set -u
WORLD=${1:-open_3d}
SX=${2:-10.0}; SY=${3:-10.0}; SZ=${4:-3.0}
LOG=/home/yule/rulepatch/3d/logs
setsid nohup xvfb-run -a gz sim -r -s /home/yule/rulepatch/3d/worlds/$WORLD.sdf > $LOG/diag_sim.log 2>&1 < /dev/null &
sleep 12
gz sim --versions 2>/dev/null | head -1 || true
setsid nohup ros2 launch nav2_minimal_tb3_sim spawn_tb3.launch.py \
  robot_name:=r2i_quad robot_sdf:="/home/yule/R2I-RL-checkout/rulesync3d/worlds/r2i_quad.sdf" \
  x_pose:=$SX y_pose:=$SY z_pose:=$SZ > $LOG/diag_spawn.log 2>&1 < /dev/null &
sleep 8
tail -3 $LOG/diag_spawn.log
setsid nohup ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -p config_file:=/tmp/quad_bridge.yaml -p use_sim_time:=True \
  -p expand_gz_topic_names:=True > $LOG/diag_bridge.log 2>&1 < /dev/null &
sleep 6
echo "diag bringup $WORLD done"
