"""Set a node parameter (robust replacement for `ros2 param set`).

Usage: python3 rp_params.py <node> <param> <true|false|int>
Prints True on success. Exits 0 always (caller greps output).
"""
import sys

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as PM


def main():
    tgt, name, val = sys.argv[1], sys.argv[2], sys.argv[3]
    rclpy.init()
    n = Node("rp_paramset")
    try:
        cli = n.create_client(SetParameters, f"/{tgt}/set_parameters")
        if not cli.wait_for_service(timeout_sec=10.0):
            print("node-missing", flush=True)
            return
        req = SetParameters.Request()
        pm = PM()
        pm.name = name
        if val.lstrip("-").isdigit():
            pm.value.type = 2
            pm.value.integer_value = int(val)
        else:
            pm.value.type = 1
            pm.value.bool_value = val.lower() in ("true", "1")
        req.parameters = [pm]
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(n, fut, timeout_sec=10.0)
        ok = bool(fut.result().results[0].successful) if fut.result() else False
        print(str(ok), flush=True)
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
