# Unwedge + actuator retest: back away (-x) from the slab face, then climb.
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
n = Node("unwedge")


def cb(msg):
    for a, b in zip(msg.name, msg.position):
        last[a] = float(b)


n.create_subscription(JointState, "/r2i_quad/joint_state", cb, qos)
px = n.create_publisher(Float64, "/r2i_quad/jx/cmd_vel", 10)
pz = n.create_publisher(Float64, "/r2i_quad/jz/cmd_vel", 10)
for _ in range(30):
    rclpy.spin_once(n, timeout_sec=0.2)
    if last:
        break
print(f"start jx={last.get('jx')} jz={last.get('jz')}", flush=True)
t0 = time.time()
while time.time() - t0 < 5.0:  # back away from wall
    px.publish(Float64(data=-1.0))
    pz.publish(Float64(data=0.0))
    rclpy.spin_once(n, timeout_sec=0.05)
    time.sleep(0.1)
print(f"after -x jx={last.get('jx')} jz={last.get('jz')}", flush=True)
t0 = time.time()
while time.time() - t0 < 6.0:  # climb in free space
    px.publish(Float64(data=0.0))
    pz.publish(Float64(data=0.8))
    rclpy.spin_once(n, timeout_sec=0.05)
    time.sleep(0.1)
print(f"after +z jx={last.get('jx')} jz={last.get('jz')}", flush=True)
px.publish(Float64(data=0.0))
pz.publish(Float64(data=0.0))
n.destroy_node()
rclpy.shutdown()
