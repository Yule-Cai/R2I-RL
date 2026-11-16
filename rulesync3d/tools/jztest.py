# Manual jz actuator test: command +z velocity, watch joint position.
import sys, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

last = {}


def cb(msg):
    for n, p in zip(msg.name, msg.position):
        last[n] = p


rclpy.init()
n = Node("jztest")
n.create_subscription(JointState, "/r2i_quad/joint_state", cb, 10)
pub = n.create_publisher(Float64, "/r2i_quad/jz/cmd_vel", 10)
time.sleep(1.0)
for _ in range(20):
    rclpy.spin_once(n, timeout_sec=0.2)
z0 = last.get("jz", None)
print(f"jz0={z0}", flush=True)
t0 = time.time()
while time.time() - t0 < 6.0:
    pub.publish(Float64(data=0.8))
    rclpy.spin_once(n, timeout_sec=0.1)
    time.sleep(0.05)
for _ in range(10):
    rclpy.spin_once(n, timeout_sec=0.2)
print(f"jz1={last.get('jz', None)} moved={last.get('jz', 0) - (z0 or 0)}",
      flush=True)
pub.publish(Float64(data=0.0))
n.destroy_node()
rclpy.shutdown()
