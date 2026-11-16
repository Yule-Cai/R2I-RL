# List volumes: prints epoch + active rule_ids.
import rclpy
from rclpy.node import Node
from rulesync_interfaces.srv import ListVolumes

rclpy.init()
n = Node("vollist")
c = n.create_client(ListVolumes, "/rulesync/list_volumes")
assert c.wait_for_service(timeout_sec=30.0)
f = c.call_async(ListVolumes.Request())
rclpy.spin_until_future_complete(n, f, timeout_sec=30.0)
r = f.result()
print(f"epoch={r.epoch} active={[str(v.rule_id) for v in r.active]}",
      flush=True)
n.destroy_node()
rclpy.shutdown()
