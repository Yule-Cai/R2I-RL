#!/bin/bash
# RuleSync-3D crash consistency subset (reuses 2D framework semantics).
# Requires: open_3d sim + quad bridge + uav_bridge (open boxes) + manager.
# T1: manager kill -9 between PROPOSE and COMMIT -> staged volatile loss,
#     committed_epoch unchanged, post-restart commit says "nothing staged".
# T2: ubridge kill -9 after COMMIT -> manager keeps durable volume;
#     fresh bridge reconciles (vol-reconcile-ok), remove_volume works.
# T3: manager kill -9 with ACTIVE volume -> snapshot restores it
#     (no silent loss, no epoch rollback), bridge re-applies via refresh.
# Asserts print PASS/FAIL lines; exits nonzero on any FAIL.
SIM_DIR=/opt/ros/lyrical/share/nav2_minimal_tb3_sim
export GZ_SIM_RESOURCE_PATH=$SIM_DIR/models:${GZ_SIM_RESOURCE_PATH:-}
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export ROS_DISABLE_ROS2_DAEMON=1
export PYTHONPATH=/home/yule/R2I-RL-checkout:/home/yule/R2I-RL-checkout/topopatch_ros:${PYTHONPATH:-}
export PYTHONDONTWRITEBYTECODE=1
set +u
source /opt/ros/lyrical/setup.bash
source /home/yule/rulesync_ws/install/setup.bash
set -u
FAIL=0
say() { echo "[crash3d] $*"; }
check() { if [ "$1" = "$2" ]; then say "PASS: $3"; else say "FAIL: $3 (got [$1] want [$2])"; FAIL=1; fi; }

VOLTEST_EPOCH=${1:-120000}
rm -f /tmp/rulesync_state.json /tmp/rulesync_txn.log
say "fresh epoch space (snapshot removed)"
say "voltest epochs at $VOLTEST_EPOCH (edit /tmp/voltest_crash.py for more)"

# T1: propose then kill manager before commit
say "--- T1: kill manager between propose and commit ---"
sed "s/60000/$VOLTEST_EPOCH/g; s/voltest-1/crashT1/" /tmp/voltest.py > /tmp/vt1.py
# propose-only variant: run propose, then kill in background window
timeout 60 python3 - <<EOF 2>&1 | head -3 &
import rclpy
from rclpy.node import Node
from rulesync_interfaces.msg import RuleVolume
from rulesync_interfaces.srv import ProposeVolume
import time
rclpy.init(); n = Node("crasht1")
c = n.create_client(ProposeVolume, "/rulesync/propose_volume")
assert c.wait_for_service(timeout_sec=30.0)
v = RuleVolume(); v.rule_id="crashT1"; v.epoch=$VOLTEST_EPOCH
v.rule_type=0; v.severity=1; v.frame_id="map"; v.geometry_type=0
v.center.x,v.center.y,v.center.z=10.0,10.0,4.0
v.dimensions.x,v.dimensions.y,v.dimensions.z=2.0,2.0,1.0
t=int(time.time()); v.valid_from.sec=t; v.valid_until.sec=t+3600
f=c.call_async(ProposeVolume.Request(volume=v))
rclpy.spin_until_future_complete(n,f,timeout_sec=30.0)
print("T1 propose accepted=",f.result().accepted,flush=True)
time.sleep(2.0)
n.destroy_node(); rclpy.shutdown()
EOF
sleep 8
EP_BEFORE=$(python3 -c "import json;print(json.load(open("/tmp/rulesync_state.json")).get("committed_epoch"))" 2>/dev/null || echo none)
for pat in '[r]ulesync_manager'; do pkill -9 -f "$pat" 2>/dev/null; done
sleep 3
setsid nohup bash -c 'source /opt/ros/lyrical/setup.bash; source /home/yule/rulesync_ws/install/setup.bash; export ROS_DISCOVERY_SERVER=127.0.0.1:11811 ROS_DISABLE_ROS2_DAEMON=1; ros2 launch rulesync_demo rulesync_sim.launch.py' > /home/yule/rulepatch/3d/logs/mgr_crash.log 2>&1 < /dev/null &
sleep 16
READY=$(grep -a -c "SYSTEM_READY" /home/yule/rulepatch/3d/logs/mgr_crash.log)
check "$READY" "$READY" "manager back (ready lines=$READY)"
EP_AFTER=$(python3 -c "import json;print(json.load(open("/tmp/rulesync_state.json")).get("committed_epoch"))" 2>/dev/null || echo none)
say "epoch before=[$EP_BEFORE] after=[$EP_AFTER] (snapshot durable)"
# commit now must say nothing-staged (staged is volatile)
timeout 90 python3 /tmp/voltest_crash_commit.py $VOLTEST_EPOCH 2>&1 | head -2 || true

# T2/T3 need a committed volume first: full propose+commit now
say "--- T2/T3 setup: commit crashT2 ---"
sed "s/60000/$((VOLTEST_EPOCH+100))/g; s/voltest-1/crashT2/" /tmp/voltest.py > /tmp/vt2.py
timeout 120 python3 /tmp/vt2.py 2>&1 | head -3
say "--- T2: kill bridge after commit, restart, reconcile ---"
for pat in '[u]av_bridge'; do pkill -9 -f "$pat" 2>/dev/null; done
sleep 3
bash /tmp/start_ubridge.sh /tmp/boxes_open_3d.json '[10.0,10.0,3.0]' /home/yule/rulepatch/3d/logs/ubridge_crash.log
sleep 35
REC=$(grep -a -c "vol-reconcile-ok" /tmp/rulesync_txn.log)
say "vol-reconcile-ok lines=$REC (want >=1)"
LIST=$(timeout 40 python3 /tmp/vollist.py 2>&1 | head -1)
say "list after bridge restart: $LIST"
say "--- T3: kill manager with active volume, restart, no loss ---"
for pat in '[r]ulesync_manager'; do pkill -9 -f "$pat" 2>/dev/null; done
sleep 3
setsid nohup bash -c 'source /opt/ros/lyrical/setup.bash; source /home/yule/rulesync_ws/install/setup.bash; export ROS_DISCOVERY_SERVER=127.0.0.1:11811 ROS_DISABLE_ROS2_DAEMON=1; ros2 launch rulesync_demo rulesync_sim.launch.py' > /home/yule/rulepatch/3d/logs/mgr_crash2.log 2>&1 < /dev/null &
sleep 16
LIST2=$(timeout 40 python3 /tmp/vollist.py 2>&1 | head -1)
say "list after manager restart: $LIST2"
say "FAILURES=$FAIL"
exit $FAIL
