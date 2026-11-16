# 4-pub + sub aging probe: command jz +0.6 for 180 s, buckets per 30 s.
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

last = {}
qos = QoSProfile(depth=10)
qos.reliability = ReliabilityPolicy.BEST_EFFORT
rclpy.init()
n = Node("long4")
cmd = {j: n.create_publisher(Float64, f"/r2i_quad/{j}/cmd_vel", 10)
       for j in ("jx", "jy", "jz", "jyaw")}


def cb(msg):
    for a, b in zip(msg.name, msg.position):
        last[a] = float(b)


n.create_subscription(JointState, "/r2i_quad/joint_state", cb, qos)
for _ in range(50):
    rclpy.spin_once(n, timeout_sec=0.0)
    time.sleep(0.01)
    if last:
        break
z0 = last.get("jz", 0.0)
print(f"t=0 jz={z0:.3f}", flush=True)
t_start = time.time()
prev = z0
while time.time() - t_start < 240.0:
    el = time.time() - t_start
    rclpy.spin_once(n, timeout_sec=0.0)
    time.sleep(0.008)
    direction = 0.6 if int(el / 40) % 2 == 0 else -0.6
    for j in ("jx", "jy", "jz", "jyaw"):
        cmd[j].publish(Float64(data=direction if j == "jz" else 0.0))
    if el // 30 != getattr(n, "_b", -1):
        setattr(n, "_b", el // 30)
        cur = last.get("jz", 0.0)
        print(f"t={int(el)} jz={cur:.3f} moved30={abs(cur - prev):+.3f}",
              flush=True)
        prev = cur
for p in cmd.values():
    p.publish(Float64(data=0.0))
n.destroy_node()
rclpy.shutdown()
