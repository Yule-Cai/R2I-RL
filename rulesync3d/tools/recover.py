# Recovery: drive jx back west (out of the east wall), then retest jz.
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
n = Node("recover")


def cb(msg):
    for a, b in zip(msg.name, msg.position):
        last[a] = float(b)


n.create_subscription(JointState, "/r2i_quad/joint_state", cb, qos)
px = n.create_publisher(Float64, "/r2i_quad/jx/cmd_vel", 10)
pz = n.create_publisher(Float64, "/r2i_quad/jz/cmd_vel", 10)


def spin(d):
    t0 = time.time()
    while time.time() - t0 < d:
        rclpy.spin_once(n, timeout_sec=0.05)


spin(3.0)
print(f"start jx={last.get('jx')} jz={last.get('jz')}", flush=True)
t0 = time.time()
while time.time() - t0 < 25.0:  # back west into free room
    px.publish(Float64(data=-1.0))
    pz.publish(Float64(data=0.0))
    rclpy.spin_once(n, timeout_sec=0.05)
    time.sleep(0.1)
print(f"west jx={last.get('jx')} jz={last.get('jz')}", flush=True)
t0 = time.time()
while time.time() - t0 < 8.0:  # climb test in free space
    px.publish(Float64(data=0.0))
    pz.publish(Float64(data=0.8))
    rclpy.spin_once(n, timeout_sec=0.05)
    time.sleep(0.1)
print(f"climb jx={last.get('jx')} jz={last.get('jz')}", flush=True)
px.publish(Float64(data=0.0))
pz.publish(Float64(data=0.0))
n.destroy_node()
rclpy.shutdown()
