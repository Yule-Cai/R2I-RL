# OctoMap-vs-oracle evaluation: occupied centers -> grid @res, compared
# against ground-truth boxes. Reports precision/recall/F1 of occupied
# voxels + free-space coverage + saves both grids. Frame: map.
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.qos import HistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--boxes", required=True, help="oracle AABB json")
    ap.add_argument("--res", type=float, default=0.2)
    ap.add_argument("--topic", default="/octomap_point_cloud_centers")
    ap.add_argument("--out", default="/home/yule/rulepatch/3d/octomap_eval.npz")
    ap.add_argument("--wait-s", type=float, default=20.0)
    a = ap.parse_args()
    rclpy.init()
    n = Node("octomap_eval")
    got = {}
    q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   history=HistoryPolicy.KEEP_LAST)

    def cb(m):
        pts = np.asarray(list(pc2.read_points(m, field_names=("x", "y",
                                                              "z"),
                                              skip_nans=True)))
        got["pts"] = pts

    n.create_subscription(PointCloud2, a.topic, cb, q)
    t0 = time.time()
    while time.time() - t0 < a.wait_s and "pts" not in got:
        rclpy.spin_once(n, timeout_sec=0.3)
    if "pts" not in got:
        print("EVAL: no centers received", flush=True)
        return
    import ast
    boxes = ast.literal_eval(open(a.boxes).read())
    from rulesync3d.rulesync3d.worlds_3d import BOUNDS
    from rulesync3d.rulesync3d.planners_3d import voxelize_bounds
    res = float(a.res)
    sens_grid, org = voxelize_bounds(
        [(p[0] - res / 2, p[1] - res / 2, p[2] - res / 2,
          p[0] + res / 2, p[1] + res / 2, p[2] + res / 2)
         for p in got["pts"]], res, BOUNDS, margin=0.0)
    or_grid, _ = voxelize_bounds(boxes, res, BOUNDS, margin=0.0)
    tp = int((sens_grid & or_grid).sum())
    fp = int((sens_grid & ~or_grid).sum())
    fn = int((~sens_grid & or_grid).sum())
    prec = tp / max(1, tp + fp)
    rec = tp / max(1, tp + fn)
    print(f"EVAL: sens_occ={int(sens_grid.sum())} "
          f"oracle_occ={int(or_grid.sum())} tp={tp} fp={fp} fn={fn} "
          f"prec={prec:.3f} rec={rec:.3f} "
          f"f1={2*prec*rec/max(1e-9, prec+rec):.3f}", flush=True)
    np.savez_compressed(a.out, sens=sens_grid, oracle=or_grid,
                        origin=np.asarray(org), res=res)
    print(f"EVAL: wrote {a.out}", flush=True)
    n.destroy_node()
    rclpy.shutdown()


main()
