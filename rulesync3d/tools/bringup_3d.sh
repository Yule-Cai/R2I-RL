#!/bin/bash
# Stage-A 3D bringup: open_3d world + gantry quad + bridges. Leaves stack up.
SIM_DIR=/opt/ros/lyrical/share/nav2_minimal_tb3_sim
export GZ_SIM_RESOURCE_PATH=$SIM_DIR/models:${GZ_SIM_RESOURCE_PATH:-}
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export ROS_DISABLE_ROS2_DAEMON=1
set +u
source /opt/ros/lyrical/setup.bash
source /home/yule/rulesync_ws/install/setup.bash
set -u
WORLD=${1:-open_3d}
LOG=/home/yule/rulepatch/3d/logs
mkdir -p $LOG
for pat in '[g]z-sim-main' '[g]z sim' '[p]arameter_bridge' '[r]os2 run'; do
  pkill -9 -f "$pat" 2>/dev/null || true
done
sleep 2
pkill -9 -f '[f]ast-discovery-server' 2>/dev/null || true
sleep 2
bash -c 'source /opt/ros/lyrical/setup.bash && setsid nohup fastdds discovery -i 0 -l 127.0.0.1 -p 11811 > /tmp/ds_3d.log 2>&1 < /dev/null & echo bg' >/dev/null 2>&1 || true
sleep 4
setsid nohup xvfb-run -a gz sim -r -s /home/yule/rulepatch/3d/worlds/$WORLD.sdf > $LOG/sim_$WORLD.log 2>&1 < /dev/null &
sleep 12
# spawn quad at world spawn via gz service
setsid nohup ros2 launch nav2_minimal_tb3_sim spawn_tb3.launch.py \
  robot_name:=r2i_quad robot_sdf:="/home/yule/R2I-RL-checkout/rulesync3d/worlds/r2i_quad.sdf" \
  x_pose:=1.0 y_pose:=10.0 z_pose:=1.0 > $LOG/spawn_$WORLD.log 2>&1 < /dev/null &
sleep 8
# bridges: joint cmd_vel (ROS->GZ Float64), joint state (GZ->ROS), contacts
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
  -p expand_gz_topic_names:=True \
  > $LOG/bridge_$WORLD.log 2>&1 < /dev/null &
sleep 6
echo "bringup $WORLD done"
