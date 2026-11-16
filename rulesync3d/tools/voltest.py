# Manual volume 2PC test: propose+commit one AABB far from the robot.
import sys, time
import rclpy
from rclpy.node import Node
from rulesync_interfaces.msg import RuleVolume
from rulesync_interfaces.srv import ProposeVolume, CommitVolume, ListVolumes


def main():
    rclpy.init()
    n = Node("voltest")
    cp = n.create_client(ProposeVolume, "/rulesync/propose_volume")
    cc = n.create_client(CommitVolume, "/rulesync/commit_volume")
    cl = n.create_client(ListVolumes, "/rulesync/list_volumes")
    for nm, c in (("propose", cp), ("commit", cc), ("list", cl)):
        assert c.wait_for_service(timeout_sec=30.0), nm
    v = RuleVolume()
    v.rule_id = "voltest-1"
    v.epoch = 60000
    v.rule_type = 0
    v.severity = 1
    v.frame_id = "map"
    v.geometry_type = 0
    v.center.x, v.center.y, v.center.z = 10.0, 10.0, 1.5
    v.dimensions.x, v.dimensions.y, v.dimensions.z = 3.0, 3.0, 2.0
    t = time.time(); tsec = int(t)
    v.valid_from.sec = tsec
    v.valid_until.sec = tsec + 3600
    f = cp.call_async(ProposeVolume.Request(volume=v))
    rclpy.spin_until_future_complete(n, f, timeout_sec=30.0)
    r = f.result()
    print(f"propose accepted={r.accepted} reason={r.reason}", flush=True)
    f2 = cc.call_async(CommitVolume.Request(epoch=60000))
    rclpy.spin_until_future_complete(n, f2, timeout_sec=60.0)
    r2 = f2.result()
    print(f"commit committed={r2.committed} reason={r2.reason}", flush=True)
    f3 = cl.call_async(ListVolumes.Request())
    rclpy.spin_until_future_complete(n, f3, timeout_sec=30.0)
    r3 = f3.result()
    print(f"list epoch={r3.epoch} active={len(r3.active)}", flush=True)
    n.destroy_node()
    rclpy.shutdown()


main()
