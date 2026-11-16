"""Synchronous volume-vote helper (one-shot process).

Mirrors vote_once.py for RuleVolume 2PC rounds: in-process service vote
collection from the long-lived manager process proved unreliable in this
environment (requests arrive, responses never complete the client's
future), while fresh short-lived processes succeed deterministically
(verified: pool-thread/spin patterns work from a fresh node, fail from
the manager node regardless of client age or creating thread).

Prints: VOTE=<int> REASON=<str>.
"""
import sys
import time
_BOOT = time.monotonic()

import rclpy
from rulesync_interfaces.srv import BridgeVoteVolume
from rulesync_interfaces.msg import RuleVolume
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Time


def main():
    import json
    timings = {"imports_s": time.monotonic() - _BOOT}
    argv = sys.argv[1:]
    service, epoch, op, vols_json = argv[0], argv[1], argv[2], argv[3]
    txn_id = argv[4] if len(argv) > 4 else ""
    target_hash = argv[5] if len(argv) > 5 else ""
    vols_data = json.loads(vols_json)
    start = time.monotonic()
    rclpy.init()
    n = rclpy.create_node("vote_volume_once")
    cli = n.create_client(BridgeVoteVolume, service)
    timings["init_client_s"] = time.monotonic() - start
    start = time.monotonic()
    if not cli.wait_for_service(timeout_sec=10.0):
        print("VOTE=2 REASON=no-service")
        return
    timings["discovery_s"] = time.monotonic() - start
    start = time.monotonic()
    vols = []
    for vd in vols_data:
        v = RuleVolume()
        v.rule_id = vd["rule_id"]
        v.epoch = int(vd["epoch"])
        v.rule_type = int(vd.get("rule_type", 0))
        v.severity = int(vd.get("severity", 1))
        v.frame_id = vd.get("frame_id", "map")
        v.geometry_type = int(vd["geometry_type"])
        c = vd.get("center", [0, 0, 0])
        v.center.x, v.center.y, v.center.z = (float(x) for x in c)
        d = vd.get("dims", [0, 0, 0])
        v.dimensions.x, v.dimensions.y, v.dimensions.z = (
            float(x) for x in d)
        v.vertices = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                      for p in vd.get("vertices", [])]
        # Validity MUST round-trip from the coordinator (hash covers it);
        # fabricating fresh stamps here would break target-hash agreement.
        vf = vd.get("valid_from", [0, 0])
        vu = vd.get("valid_until", [3600, 0])
        v.valid_from = Time(sec=int(vf[0]), nanosec=int(vf[1]))
        v.valid_until = Time(sec=int(vu[0]), nanosec=int(vu[1]))
        vols.append(v)
    req = BridgeVoteVolume.Request()
    req.volume_rules = vols
    req.epoch = int(epoch)
    req.op = op
    try:
        req.txn_id = str(txn_id)
        req.target_hash = str(target_hash)
    except Exception:
        pass
    timings["request_build_s"] = time.monotonic() - start
    start = time.monotonic()
    fut = cli.call_async(req)
    t0 = time.monotonic()
    while rclpy.ok() and not fut.done() and time.monotonic() - t0 < 25.0:
        rclpy.spin_once(n, timeout_sec=1.0)
    if fut.done():
        print(f"VOTE={int(fut.result().vote)} REASON={fut.result().reason}",
              flush=True)
    else:
        print("VOTE=2 REASON=helper-timeout")
    timings["service_s"] = time.monotonic() - start
    start = time.monotonic()
    n.destroy_node()
    rclpy.shutdown()
    timings["shutdown_s"] = time.monotonic() - start
    timings["total_s"] = time.monotonic() - _BOOT
    print("TIMING=" + json.dumps(timings, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
