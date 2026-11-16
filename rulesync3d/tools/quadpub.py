# Reproduce execute-like load: 4 simultaneous pubs at 25 Hz.
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
n = Node("quadpub")


def cb(msg):
    for a, b in zip(msg.name, msg.position):
        last[a] = float(b)


n.create_subscription(JointState, "/r2i_quad/joint_state", cb, qos)
pubs = {j: n.create_publisher(Float64, f"/r2i_quad/{j}/cmd_vel", 10)
        for j in ("jx", "jy", "jz", "jyaw")}
for _ in range(30):
    rclpy.spin_once(n, timeout_sec=0.1)
    if last:
        break
print(f"start jx={last.get('jx')} jz={last.get('jz')}", flush=True)
t0 = time.time()
while time.time() - t0 < 8.0:
    rclpy.spin_once(n, timeout_sec=0.005)
    pubs["jx"].publish(Float64(data=0.0))
    pubs["jy"].publish(Float64(data=0.0))
    pubs["jz"].publish(Float64(data=0.8))
    pubs["jyaw"].publish(Float64(data=0.0))
    time.sleep(0.035)  # ~25 Hz
print(f"end jx={last.get('jx')} jz={last.get('jz')}", flush=True)
for p in pubs.values():
    p.publish(Float64(data=0.0))
n.destroy_node()
rclpy.shutdown()
