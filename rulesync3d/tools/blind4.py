# Blind 4-pub (no subscription): fire jz +0.6 for 8 s.
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

rclpy.init()
n = Node("blind4")
cmd = {j: n.create_publisher(Float64, f"/r2i_quad/{j}/cmd_vel", 10)
       for j in ("jx", "jy", "jz", "jyaw")}
time.sleep(3.0)
t0 = time.time()
while time.time() - t0 < 8.0:
    cmd["jz"].publish(Float64(data=0.6))
    time.sleep(0.15)
cmd["jz"].publish(Float64(data=0.0))
print("blind4 done", flush=True)
n.destroy_node()
rclpy.shutdown()
