# Isolation repro for audit cp (propose_volume) discovery failures.
# Runs N trials back-to-back against the LIVE system: init node, wait for
# list_volumes + bridge-state + propose_volume (timed), report per-trial
# latencies. Distinguishes transient discovery sickness (scattered slow
# trials) from systematic propose blindness (cp always slow/failing while
# cl/cb fast). Read-only: creates NO staging, sends NO requests.
import sys
import time

import rclpy
from rclpy.node import Node

TRIALS = int(sys.argv[1]) if len(sys.argv) > 1 else 5


def wait_timed(node, svc_type, name, timeout_each=30.0, attempts=3,
               fresh_each=False):
    from rulesync_interfaces.srv import (
        ListVolumes, BridgeVolumeState, ProposeVolume)
    typ = {"list": ListVolumes, "state": BridgeVolumeState,
           "propose": ProposeVolume}[svc_type]
    lat = []
    cli = None
    for _ in range(attempts):
        if fresh_each or cli is None:
            if cli is not None:
                try:
                    node.destroy_client(cli)
                except Exception:
                    pass
            cli = node.create_client(typ, name)
        t0 = time.monotonic()
        ok = cli.wait_for_service(timeout_sec=timeout_each)
        lat.append(round(time.monotonic() - t0, 1))
        if ok:
            return True, lat
    return False, lat


def main():
    import os as _os
    rclpy.init()
    try:
        for i in range(TRIALS):
            # even trials: reuse client across attempts (audit-like);
            # odd trials: fresh client per attempt.
            fresh = bool(i % 2)
            n = Node(f"disco_probe_{_os.getpid()}_{i}")
            r = {}
            for svc, name in (("list", "/rulesync/list_volumes"),
                              ("state", "/rulesync/uav_bridge_state"),
                              ("propose", "/rulesync/propose_volume")):
                ok, lat = wait_timed(n, svc, name, fresh_each=fresh)
                r[svc] = f"{'OK' if ok else 'FAIL'}{lat}"
            print(f"trial {i} fresh={fresh}: {r}", flush=True)
            n.destroy_node()
            time.sleep(2)
    finally:
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


main()
