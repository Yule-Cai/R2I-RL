#!/bin/bash
# TopoRulePatch headless Nav2 planning stack for one map (domain 7).
# Kills any previous headless stack first (exclusive domain-7 access),
# then launches map_server + planner_server + lifecycle for CFG map key.
# Usage: bash bringup_topo.sh <mapkey>   # e.g. office-A-s101
# NOTE: no `set -u` — ROS setup.bash trips nounset (AMENT_TRACE_SETUP_FILES).
source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=7
MAPKEY=${1:?usage: bringup_topo.sh <mapkey>}
CFG=/home/yule/rulepatch/cfg/planner_${MAPKEY}.yaml
LOG=/home/yule/rulepatch/logs
test -f "$CFG" || { echo "missing cfg $CFG"; exit 1; }
pkill -f 'nav2_map_server/map_server' 2>/dev/null || true
pkill -f 'nav2_planner/planner_server' 2>/dev/null || true
pkill -f 'nav2_lifecycle_manager/lifecycle_manager' 2>/dev/null || true
pkill -f 'tf2_ros.*static_transform_publisher' 2>/dev/null || true
sleep 3
ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --ros-args -p use_sim_time:=False > $LOG/topo_tf_odom.log 2>&1 &
ros2 run tf2_ros static_transform_publisher --frame-id odom --child-frame-id base_link --x 2 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --ros-args -p use_sim_time:=False > $LOG/topo_tf_base.log 2>&1 &
ros2 run nav2_map_server map_server --ros-args --params-file $CFG > $LOG/topo_map.log 2>&1 &
ros2 run nav2_planner planner_server --ros-args --params-file $CFG > $LOG/topo_planner.log 2>&1 &
sleep 2
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args --params-file $CFG -r __node:=lifecycle_manager_navigation > $LOG/topo_lifecycle.log 2>&1 &
echo "bringup-topo-launched $MAPKEY"
