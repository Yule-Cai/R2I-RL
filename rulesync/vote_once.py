"""Synchronous bridge-vote helper (one-shot process).

Rationale (documented): in-process service/topic vote collection from the
long-lived manager process proved unreliable in this environment, while
fresh short-lived processes succeed deterministically. The manager therefore
delegates each vote round to this helper. Prints: VOTE=<int> REASON=<str>.
"""
import sys
import time

import rclpy
from rulesync_interfaces.srv import BridgeVote
from rulesync_interfaces.msg import RuleSpec
from geometry_msgs.msg import Point32
from builtin_interfaces.msg import Time


def main():
    import json
    argv = sys.argv[1:]
    service, epoch, op, rules_json = argv[0], argv[1], argv[2], argv[3]
    txn_id = argv[4] if len(argv) > 4 else ""
    target_hash = argv[5] if len(argv) > 5 else ""
    rules_data = json.loads(rules_json)
    rclpy.init()
    n = rclpy.create_node("vote_once")
    cli = n.create_client(BridgeVote, service)
    if not cli.wait_for_service(timeout_sec=10.0):
        print("VOTE=2 REASON=no-service")
        return
    rules = []
    for rd in rules_data:
        r = RuleSpec()
        r.rule_id = rd["rule_id"]
        r.epoch = int(rd["epoch"])
        r.rule_type = int(rd["rule_type"])
        r.severity = int(rd.get("severity", 1))
        r.frame_id = rd.get("frame_id", "map")
        r.polygon.points = [Point32(x=float(p[0]), y=float(p[1]),
                                    z=float(p[2]) if len(p) > 2 else 0.0)
                            for p in rd.get("points", [])]
        r.speed_limit = float(rd.get("speed_limit", 0.0))
        # Validity MUST round-trip from the coordinator (hash covers it);
        # fabricating fresh stamps here would break target-hash agreement.
        vf = rd.get("valid_from", [0, 0])
        vu = rd.get("valid_until", [3600, 0])
        try:
            r.valid_from = Time(sec=int(vf[0]), nanosec=int(vf[1]))
            r.valid_until = Time(sec=int(vu[0]), nanosec=int(vu[1]))
        except Exception:
            r.valid_from = Time(sec=0)
            r.valid_until = Time(sec=3600)
        rules.append(r)
    req = BridgeVote.Request()
    req.staged_rules = rules
    req.epoch = int(epoch)
    req.op = op
    try:
        req.txn_id = str(txn_id)
        req.target_hash = str(target_hash)
    except Exception:
        pass
    fut = cli.call_async(req)
    t0 = time.time()
    while rclpy.ok() and not fut.done() and time.time() - t0 < 25.0:
        rclpy.spin_once(n, timeout_sec=1.0)
    if fut.done():
        print(f"VOTE={int(fut.result().vote)} REASON={fut.result().reason}",
              flush=True)
    else:
        print("VOTE=2 REASON=helper-timeout")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
