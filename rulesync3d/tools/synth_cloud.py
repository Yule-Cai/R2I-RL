# Synthetic room scan -> octomap_server validation (no sim needed).
# Sensor at (10,10,3) (room center); walls/floor sampled in lidar frame.
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def cloud_msg(pts, frame):
    msg = PointCloud2()
    msg.header = Header(frame_id=frame)
    msg.header.stamp.sec = int(time.time())
    msg.height, msg.width = 1, len(pts)
    msg.fields = [PointField(name=a, offset=i * 4,
                             datatype=PointField.FLOAT32, count=1)
                  for i, a in enumerate("xyz")]
    msg.is_bigendian = False
    msg.point_step, msg.row_step = 12, 12 * len(pts)
    msg.is_dense = True
    msg.data = np.asarray(pts, dtype=np.float32).tobytes()
    return msg


def main():
    rclpy.init()
    n = Node("synth_cloud")
    pub = n.create_publisher(PointCloud2, "/synth_cloud", 10)
    rng = np.random.default_rng(0)
    S = np.array([10.0, 10.0, 3.0])
    pts = []
    # 4 boundary walls (y=0/20, x=0/20, z 0..1) + floor patch, in map frame
    for _ in range(600):
        pts.append([rng.uniform(0, 20), 0.0, rng.uniform(0, 1)])
        pts.append([rng.uniform(0, 20), 20.0, rng.uniform(0, 1)])
    for _ in range(400):
        pts.append([0.0, rng.uniform(0, 20), rng.uniform(0, 1)])
        pts.append([20.0, rng.uniform(0, 20), rng.uniform(0, 1)])
    for _ in range(400):
        pts.append([rng.uniform(0, 20), rng.uniform(0, 20), 0.0])
    pts = np.asarray(pts) - S  # to lidar frame
    for _ in range(12):
        m = cloud_msg(pts, "lidar_frame")
        m.header.stamp = n.get_clock().now().to_msg()
        pub.publish(m)
        rclpy.spin_once(n, timeout_sec=0.1)
        time.sleep(0.5)
    print(f"published {len(pts)} pts", flush=True)
    n.destroy_node()
    rclpy.shutdown()


main()
