"""Ordered Nav2 lifecycle activation (M1 runbook automation).

Polls get_state and activates inactive nodes in dependency order:
map -> localization -> planner/controller/smoother/behavior -> bt/others.
Usage: python3 activate_nav2.py [--timeout 120]
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import Transition

ORDER = ["map_server", "amcl", "planner_server", "controller_server",
         "smoother_server", "behavior_server", "bt_navigator",
         "waypoint_follower", "velocity_smoother", "collision_monitor",
         "docking_server", "route_server", "gps_waypoint_follower"]


def get_state(node, name, timeout=5.0):
    cli = node.create_client(GetState, f"/{name}/get_state")
    if not cli.wait_for_service(timeout_sec=timeout):
        return "missing"
    fut = cli.call_async(GetState.Request())
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    return fut.result().current_state.label.lower() if fut.result() else "noresp"


def activate(node, name, timeout=10.0) -> bool:
    cli = node.create_client(ChangeState, f"/{name}/change_state")
    if not cli.wait_for_service(timeout_sec=timeout):
        return False
    req = ChangeState.Request()
    req.transition.id = Transition.TRANSITION_ACTIVATE
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    return bool(fut.result().success) if fut.result() else False


def configure(node, name, timeout=10.0) -> bool:
    cli = node.create_client(ChangeState, f"/{name}/change_state")
    if not cli.wait_for_service(timeout_sec=timeout):
        return False
    req = ChangeState.Request()
    req.transition.id = Transition.TRANSITION_CONFIGURE
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    return bool(fut.result().success) if fut.result() else False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--timeout", type=float, default=120.0)
    ap.add_argument("--nodes", default="",
                    help="comma-separated subset to activate (default: all)")
    a = ap.parse_args()
    wanted = [n.strip() for n in a.nodes.split(",") if n.strip()] or None
    order = [n for n in ORDER if wanted is None or n in wanted]
    need = ["planner_server", "controller_server", "bt_navigator", "amcl"]
    need = [n for n in need if wanted is None or n in wanted]
    rclpy.init()
    node = Node("rp_activate_nav2")
    try:
        t0 = time.time()
        pending = list(order)
        while pending and time.time() - t0 < a.timeout:
            name = pending[0]
            st = get_state(node, name, timeout=3.0)
            if st == "missing":
                print(f"[activate] {name}: missing, skip", flush=True)
                pending.pop(0)
            elif st == "active":
                pending.pop(0)
            elif st == "unconfigured":
                ok = configure(node, name)
                print(f"[activate] {name}: {st} -> configure={ok}", flush=True)
                time.sleep(1.0)
            elif st == "inactive":
                ok = activate(node, name)
                print(f"[activate] {name}: {st} -> activate={ok}", flush=True)
                time.sleep(1.0)
            else:
                print(f"[activate] {name}: {st}, wait", flush=True)
                time.sleep(2.0)
        states = {n: get_state(node, n, timeout=3.0) for n in order}
        print("[activate] final:", states, flush=True)
        assert all(states.get(n) == "active" for n in need), states
        print("[activate] core nav2 ACTIVE", flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
