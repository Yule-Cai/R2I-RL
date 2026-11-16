# Standalone clone of UavDriver.actuator_selftest (4 pubs, fresh node).
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

qos = QoSProfile(depth=10)
qos.reliability = ReliabilityPolicy.BEST_EFFORT
rclpy.init()
n = Node("selftest_clone")
last = {}
import time as _tt
cmd = {}
for _j in ("jx", "jy", "jz", "jyaw"):
    cmd[_j] = n.create_publisher(Float64, f"/r2i_quad/{_j}/cmd_vel", 10)
    _tt.sleep(4.0)
print("publishers staggered", flush=True)


def cb(msg):
    for a, b in zip(msg.name, msg.position):
        last[a] = float(b)


n.create_subscription(JointState, "/r2i_quad/joint_state", cb, qos)
xyz = None


def pose():
    global xyz
    for _ in range(20):
        rclpy.spin_once(n, timeout_sec=0.1)
        if last:
            xyz = (last["jx"] + 5.0, last["jy"] + 10.0, last["jz"] + 4.0)
            return xyz
    return None


results = {}
for j, sign in (("jx", 1.0), ("jy", 1.0), ("jz", 1.0)):
    p0 = pose()
    t0 = time.time()
    while time.time() - t0 < 12.0:
        rclpy.spin_once(n, timeout_sec=0.1)
        cmd[j].publish(Float64(data=0.6 * sign))
        time.sleep(0.05)
    ax = {"jx": 0, "jy": 1, "jz": 2}[j]
    moved = abs(xyz[ax] - p0[ax]) if xyz and p0 else 0.0
    results[j] = round(moved, 3)
    for c in cmd.values():
        c.publish(Float64(data=0.0))
    time.sleep(0.5)
print(f"clone {results}", flush=True)
n.destroy_node()
rclpy.shutdown()
