"""Programmatic Nav2 planning helper (M1 blocker fix #1).

ComputePathToPose action wrapper usable from any node. No BT, no sim needed.
"""
import time

from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose


def make_pose(x, y, frame="map"):
    m = PoseStamped()
    m.header.frame_id = frame
    m.pose.position.x = float(x)
    m.pose.position.y = float(y)
    m.pose.orientation.w = 1.0
    return m


class Nav2Planner:
    def __init__(self, node, action="compute_path_to_pose",
                 planner_id="GridBased"):
        self.node = node
        self.planner_id = planner_id
        self.ac = ActionClient(node, ComputePathToPose, action)

    def wait(self, timeout=60.0) -> bool:
        return self.ac.wait_for_server(timeout_sec=timeout)

    def plan(self, start_xy, goal_xy, timeout=30.0):
        """Returns (path_xy_list | None, wall_seconds)."""
        goal = ComputePathToPose.Goal()
        goal.start = make_pose(*start_xy)
        goal.goal = make_pose(*goal_xy)
        goal.planner_id = self.planner_id
        goal.use_start = True
        t0 = time.time()
        fut = self.ac.send_goal_async(goal)
        import rclpy
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return None, time.time() - t0
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=timeout)
        res = rf.result()
        dt = time.time() - t0
        if res is None:
            return None, dt
        path = [(p.pose.position.x, p.pose.position.y)
                for p in res.result.path.poses]
        return (path if len(path) >= 2 else None), dt
