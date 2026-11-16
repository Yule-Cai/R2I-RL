# Read one joint_state with BEST_EFFORT (bridge publishes best-effort).
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState

rclpy.init()
n = Node("jsread")
got = {}
qos = QoSProfile(depth=10)
qos.reliability = ReliabilityPolicy.BEST_EFFORT
qos.history = HistoryPolicy.KEEP_LAST


def cb(msg):
    got["name"] = list(msg.name)
    got["pos"] = [round(float(p), 3) for p in msg.position]


n.create_subscription(JointState, "/r2i_quad/joint_state", cb, qos)
for _ in range(50):
    rclpy.spin_once(n, timeout_sec=0.2)
    if got:
        break
print(got, flush=True)
n.destroy_node()
rclpy.shutdown()
