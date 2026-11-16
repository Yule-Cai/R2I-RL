# Set jz to mid-range (closed-loop, gentle): pulse toward 0 until |jz|<0.5.
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
n = Node("setmid")


def cb(msg):
    for a, b in zip(msg.name, msg.position):
        last[a] = float(b)


n.create_subscription(JointState, "/r2i_quad/joint_state", cb, qos)
pub = n.create_publisher(Float64, "/r2i_quad/jz/cmd_vel", 10)
for _ in range(30):
    rclpy.spin_once(n, timeout_sec=0.1)
    if last:
        break
print(f"start jz={last.get('jz')}", flush=True)
t0 = time.time()
while time.time() - t0 < 30.0:
    rclpy.spin_once(n, timeout_sec=0.05)
    z = last.get("jz", 0.0)
    if abs(z) < 0.5:
        break
    pub.publish(Float64(data=-0.5 if z > 0 else 0.5))
    time.sleep(0.1)
pub.publish(Float64(data=0.0))
print(f"mid jz={last.get('jz')}", flush=True)
n.destroy_node()
rclpy.shutdown()
