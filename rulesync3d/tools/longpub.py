# Publisher aging test: ONE node shuttles jz (+0.8/-0.8 alternating
# every 30 s) for 220 s. Reports |motion| per 20 s bucket; recreates the
# publisher at t=110 s. Aging => early buckets full, later trickle,
# post-recreate recovery. Shuttle avoids joint limits.
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

ST = time.time()
last = {}
qos = QoSProfile(depth=10)
qos.reliability = ReliabilityPolicy.BEST_EFFORT
rclpy.init()
n = Node("longpub")


def cb(msg):
    for a, b in zip(msg.name, msg.position):
        last[a] = float(b)


n.create_subscription(JointState, "/r2i_quad/joint_state", cb, qos)
pub = n.create_publisher(Float64, "/r2i_quad/jz/cmd_vel", 10)


def spin(d):
    t0 = time.time()
    while time.time() - t0 < d:
        rclpy.spin_once(n, timeout_sec=0.05)


spin(3.0)
z0 = last.get("jz", 0.0)
print(f"t=0 jz={z0:.3f}", flush=True)
recreated = False
t_start = time.time()
prev = z0
prev_t = t_start
while time.time() - t_start < 220.0:
    el = time.time() - t_start
    if el >= 110.0 and not recreated:
        try:
            n.destroy_publisher(pub)
        except Exception:
            pass
        pub = n.create_publisher(Float64, "/r2i_quad/jz/cmd_vel", 10)
        recreated = True
        print("--- publisher recreated ---", flush=True)
    direction = 0.8 if int(el / 30) % 2 == 0 else -0.8
    pub.publish(Float64(data=direction))
    spin(0.15)
    # bucket report every 20 s
    if el // 20 != getattr(spin, "b", -1):
        setattr(spin, "b", el // 20)
        cur = last.get("jz", 0.0)
        now = time.time()
        print(f"t={int(el)} jz={cur:.3f} moved20={abs(cur - prev):.3f} "
              f"rec={recreated}", flush=True)
        prev = cur
        prev_t = now
pub.publish(Float64(data=0.0))
n.destroy_node()
rclpy.shutdown()
