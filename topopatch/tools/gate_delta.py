"""Old-vs-new gate delta on the frozen offline regression set.

Old = point-in-polygon sampling KEEP_OUT check (pre-audit logic).
New = exact segment-polygon intersection (imported gate_full_path).
Reports monotonicity (new accept => old accept must hold everywhere)
and the accept->reject delta attributable to closing the segment hole.
"""
import numpy as np

from rulepatch.rp.extract import load_occ_grid, make_occ_fn
from rulepatch.rp.geom import point_in_polygon
from topopatch.topo.extract_topo import load_topo_split_items
from topopatch.topo.gate import gate_full_path


def old_keepout_violated(path, polys):
    p = np.asarray(path, dtype=np.float64)
    for xy in p:
        for q in polys:
            if point_in_polygon(xy, np.asarray(q, dtype=np.float64)):
                return True
    return False


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", nargs="+", default=[])
    ap.add_argument("--pgms", nargs="*", default=[])
    a = ap.parse_args()
    n_paths = n_old_acc = n_new_acc = 0
    acc_rej = rej_acc = 0
    viol_old_miss = viol_new_miss = 0
    seg_hole = 0
    for di, pgm in zip(a.data, a.pgms):
        occ = load_occ_grid(pgm, res_out=0.05, res_in=0.05,
                            ox=-10.0, oy=-10.0)
        occ_fn = make_occ_fn(occ)
        w, h = occ["w"], occ["h"]
        bounds = (-10.0, -10.0, -10.0 + w * 0.05, -10.0 + h * 0.05)
        for s in load_topo_split_items(di, "test"):
            polys = [np.asarray(s["polys"][k], dtype=np.float64)
                     for k in range(int(
                         np.asarray(s["polys_n"]).reshape(-1)[0]))]
            for key in ("repl_full", "orig_full"):
                path = np.asarray(s[key], dtype=np.float64)
                n_paths += 1
                go_new, rs_new = gate_full_path(
                    path, polys, occ_fn, bounds)
                old_viol = old_keepout_violated(path, polys)
                # old full verdict: replicate old gate (bounds+bad-rule
                # identical; only KEEP_OUT differs; collision identical
                # code path via same occ sampling)
                if rs_new in ("gate:bad-path", "gate:out-of-bounds",
                              "gate:bad-rule", "gate:collision"):
                    go_old = False
                else:
                    go_old = not old_viol
                    if rs_new == "gate:KEEP_OUT-violation" and not old_viol:
                        seg_hole += 1
                n_old_acc += int(go_old)
                n_new_acc += int(go_new)
                if go_old and not go_new:
                    acc_rej += 1
                if not go_old and go_new:
                    rej_acc += 1
                # violator = any sample inside (adversarial ground truth
                # used by replay_gate); both gates must block all of them
                if old_viol:
                    viol_old_miss += int(go_old)
                    viol_new_miss += int(go_new)
    print(f"paths={n_paths} old_accept={n_old_acc} new_accept={n_new_acc}")
    print(f"accept->reject delta={acc_rej} reject->accept={rej_acc} "
          "(must be 0 by construction)")
    print(f"segment-hole cases (new blocks, old passes)={seg_hole}")
    print(f"sample-violators executed: old={viol_old_miss} new={viol_new_miss} "
          "(both must be 0)")


if __name__ == "__main__":
    main()
