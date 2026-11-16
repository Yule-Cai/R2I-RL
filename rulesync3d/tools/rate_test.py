# Factorial: njoints simultaneous pubs at hz. Usage: rate_test.py NJ HZ.
# Reports jz motion over 8 s. Discriminates rate vs simultaneity triggers.
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

NJ, HZ = int(sys.argv[1]), float(sys.argv[2])
last = {}
qos = QoSProfile(depth=10)
qos.reliability = ReliabilityPolicy.BEST_EFFORT
rclpy.init()
n = Node("ratetest")


def cb(msg):
    for a, b in zip(msg.name, msg.position):
        last[a] = float(b)


n.create_subscription(JointState, "/r2i_quad/joint_state", cb, qos)
joints = ["jz", "jx", "jy", "jyaw"][:NJ]
pubs = {j: n.create_publisher(Float64, f"/r2i_quad/{j}/cmd_vel", 10)
        for j in joints}
for _ in range(30):
    rclpy.spin_once(n, timeout_sec=0.1)
    if last:
        break
z0 = last.get("jz", 0.0)
print(f"start jz={z0:.3f} nj={NJ} hz={HZ}", flush=True)
t0 = time.time()
period = 1.0 / HZ
while time.time() - t0 < 8.0:
    rclpy.spin_once(n, timeout_sec=0.0)
    for j in joints:
        pubs[j].publish(Float64(data=0.8 if j == "jz" else 0.0))
    time.sleep(period)
print(f"end jz={last.get('jz', 0.0):.3f} moved={last.get('jz', 0.0) - z0:.3f}",
      flush=True)
for p in pubs.values():
    p.publish(Float64(data=0.0))
n.destroy_node()
rclpy.shutdown()
