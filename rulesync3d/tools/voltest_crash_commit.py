# Commit-only (expects failure when nothing staged, e.g. post-crash).
import sys
import rclpy
from rclpy.node import Node
from rulesync_interfaces.srv import CommitVolume

epoch = int(sys.argv[1]) if len(sys.argv) > 1 else 61000
rclpy.init()
n = Node("volcommitonly")
c = n.create_client(CommitVolume, "/rulesync/commit_volume")
assert c.wait_for_service(timeout_sec=30.0)
f = c.call_async(CommitVolume.Request(epoch=epoch))
rclpy.spin_until_future_complete(n, f, timeout_sec=60.0)
r = f.result()
print(f"commit committed={r.committed} reason={r.reason}", flush=True)
n.destroy_node()
rclpy.shutdown()
