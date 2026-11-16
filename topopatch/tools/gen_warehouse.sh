#!/bin/bash
# Warehouse teacher generation for the repairable benchmark (frozen pipeline).
# Same generator + thresholds as the M3 campaign; warehouse-specific map
# geometry (res 0.03, origin -15.1,-25). clear 0.5, dist 4-12, seed 101.
# Usage: bash gen_warehouse.sh  (runs ~1-2h, background)
CK=~/R2I-RL-checkout
export ROS_DOMAIN_ID=7
set +u
source /opt/ros/lyrical/setup.bash
set -u
export PYTHONPATH=/home/yule/R2I-RL-checkout:/home/yule/R2I-RL-checkout/rulepatch:/home/yule/R2I-RL-checkout/topopatch_ros:${PYTHONPATH:-}
export PYTHONDONTWRITEBYTECODE=1
MAP_PGM=/opt/ros/lyrical/share/nav2_bringup/maps/warehouse.pgm
MAP_YAML=/opt/ros/lyrical/share/nav2_bringup/maps/warehouse.yaml
for pat in '[m]ap_server' '[p]lanner_server' '[l]ifecycle_manager' '[g]en_topo_dataset'; do
  pkill -9 -f "$pat" 2>/dev/null || true
done
sleep 3
python3 $CK/topopatch/tools/paper_planner_cfg.py --pgm-yaml $MAP_YAML \
  --plugins GridBased --out /tmp/gen_warehouse_planner.yaml
setsid nohup ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom \
  --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 > ~/rulepatch/logs/gen_wh_tf.log 2>&1 < /dev/null &
setsid nohup ros2 run tf2_ros static_transform_publisher --frame-id odom --child-frame-id base_link \
  --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 >> ~/rulepatch/logs/gen_wh_tf.log 2>&1 < /dev/null &
setsid nohup ros2 run nav2_map_server map_server --ros-args \
  --params-file /tmp/gen_warehouse_planner.yaml > ~/rulepatch/logs/gen_wh_map.log 2>&1 < /dev/null &
setsid nohup ros2 run nav2_planner planner_server --ros-args \
  --params-file /tmp/gen_warehouse_planner.yaml > ~/rulepatch/logs/gen_wh_planner.log 2>&1 < /dev/null &
sleep 8
timeout 120 python3 $CK/rulepatch/tools/activate_nav2.py --timeout 100 \
  --nodes map_server,planner_server 2>&1 | tail -1
setsid nohup python3 -m topopatch.tools.gen_topo_dataset --map-id warehouse-r401 \
  --map-pgm $MAP_PGM --map-res 0.03 --map-ox -15.1 --map-oy -25 \
  --clear-m 0.5 --n-target 400 --seed 101 --shard-every 100 \
  --plan-timeout 8 --dist-min 4 --dist-max 12 \
  --out-dir ~/rulepatch/topo_raw_warehouse-r401 \
  > ~/rulepatch/logs/gen_topo_warehouse.log 2>&1 < /dev/null &
echo "[gen-wh] launched"
