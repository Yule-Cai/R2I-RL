# Isolate manager vote-client failure: pattern A (manager-like: pool-thread
# call_async + poll done) vs pattern B (CLI-like: wait + spin_until_future).
import sys, time
from concurrent.futures import ThreadPoolExecutor
import rclpy
from rclpy.node import Node
from rulesync_interfaces.srv import BridgeVoteVolume as BVV

MODE = sys.argv[1] if len(sys.argv) > 1 else "A"
rclpy.init()
n = Node("votepat")
cli = n.create_client(BVV, "/rulesync/uav_bridge_vote_volume")
pool = ThreadPoolExecutor(max_workers=2)
import threading
spin_t = threading.Thread(target=rclpy.spin, args=(n,), daemon=True)
spin_t.start()
time.sleep(2.0)
print(f"ready={cli.service_is_ready()}", flush=True)


def body():
    fut = cli.call_async(BVV.Request(volume_rules=[], epoch=1, op="prepare"))
    t0 = time.monotonic()
    while not fut.done() and time.monotonic() - t0 < 15.0:
        time.sleep(0.05)
    return fut.done(), (fut.result() if fut.done() else None)


if MODE == "A":
    f = pool.submit(body)
    done, res = f.result(timeout=30.0)
    print(f"A done={done} res={res}", flush=True)
elif MODE == "C":
    from rulesync_interfaces.msg import RuleVolume
    v = RuleVolume()
    v.rule_id = "pat-c"
    v.epoch = 42
    v.frame_id = "map"
    v.geometry_type = 0
    v.center.x, v.center.y, v.center.z = 10.0, 10.0, 1.5
    v.dimensions.x, v.dimensions.y, v.dimensions.z = 3.0, 3.0, 2.0
    v.valid_from.sec = 1
    v.valid_until.sec = 3601

    def body_c():
        fut = cli.call_async(BVV.Request(volume_rules=[v], epoch=42,
                                         op="prepare"))
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < 15.0:
            time.sleep(0.05)
        return fut.done(), (fut.result() if fut.done() else None)
    f = pool.submit(body_c)
    done, res = f.result(timeout=30.0)
    print(f"C done={done} res={res}", flush=True)
else:
    assert cli.wait_for_service(timeout_sec=15.0)
    fut = cli.call_async(BVV.Request(volume_rules=[], epoch=1, op="prepare"))
    rclpy.spin_until_future_complete(n, fut, timeout_sec=20.0)
    print(f"B done={fut.done()} res={fut.result()}", flush=True)
n.destroy_node()
rclpy.shutdown()
