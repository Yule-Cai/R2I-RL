"""Freeze protocol (milestone §14): hash the full test configuration BEFORE
running the final TEST evaluation, then verify it inside evaluate runs.

freeze.py write: collects map pgm hashes (topo_maps/*.pgm), extracted
split MANIFESTs (data dirs), model config (train config.json), verifier
constants, operating point (tau) -> freeze.json {hashes, tau, stamp}.

evaluate_topo --freeze freeze.json: asserts current map pgm hash +
extracted split sha match the frozen values, and uses the frozen tau
(overrides --tau). Mismatch -> hard error, no silent re-evaluation.

Usage:
  python3 -m topopatch.topo.freeze write --maps <topo_maps> \
      --data-dev <extracted_train_parent> --data-test <extracted_test> \
      --train-cfg <config.json> --op <operating_point.json> \
      --method model:full --out freeze.json
"""
import argparse
import glob
import hashlib
import json
import os

import numpy as np


def sha_file(path, n=1 << 20):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while True:
            b = f.read(n)
            if not b:
                break
            h.update(b)
    return h.hexdigest()[:16]


VERIFIER_CONSTANTS = {
    "DIV_THRESH": None, "REJOIN_TOL": None, "MIN_PATCH_STRAIGHT": None,
    "MAX_TURN_DEG": None, "MIN_SPACING_M": None, "MAX_SPACING_M": None,
}


def _verifier_consts():
    import rulepatch.rp as R
    import rulepatch.rp.verifier as V
    import inspect
    consts = {}
    for k in ("DIV_THRESH", "REJOIN_TOL", "MIN_PATCH_STRAIGHT"):
        consts[k] = getattr(R, k, "?")
    src = inspect.getsource(V.verify_dense)
    import re
    m = re.search(r"max_turn[^,]*,\s*([0-9.]+)", src)
    consts["MAX_TURN_DEG"] = float(m.group(1)) if m else "?"
    return consts


def cmd_write(a):
    maps = {}
    for pgm in sorted(glob.glob(os.path.join(a.maps, "*.pgm"))):
        maps[os.path.basename(pgm)] = sha_file(pgm)
    data = {}
    roots = (a.data_dev or []) + a.data_test
    for root in roots:
        tag = os.path.basename(root.rstrip("/"))
        for fp in sorted(glob.glob(os.path.join(root, "*.npz"))):
            data[f"{tag}/{os.path.basename(fp)}"] = {
                "sha": sha_file(fp), "bytes": os.path.getsize(fp)}
    train_cfg = json.load(open(a.train_cfg))
    op_all = json.load(open(a.op))
    ops = {}
    for m in a.method:
        o = op_all.get(m, {})
        ops[m] = {"tau": o.get("tau"),
                  "val_all_case_success": o.get("all_case_success"),
                  "val_coverage": o.get("coverage")}
    freeze = {
        "maps": maps,
        "data_files": data,
        "train_cfg": {k: train_cfg.get(k) for k in
                      ("m_hyp", "seed", "epochs", "batch", "lr",
                       "margin_lambda", "occ_lambda", "curv_lambda",
                       "best_val", "n_params", "repair_w", "span_w",
                       "cx_w", "repair_pos_weight", "no_topo")},
        "verifier": _verifier_consts(),
        "operating_points": ops,
        "topo_consts": _topo_consts(),
        "scope_consts": _scope_consts(),
    }
    blob = json.dumps(freeze, sort_keys=True).encode()
    freeze["freeze_sha"] = hashlib.sha256(blob).hexdigest()[:16]
    json.dump(freeze, open(a.out, "w"), indent=1)
    print(f"[freeze] {a.out} sha={freeze['freeze_sha']} "
          f"ops={json.dumps(ops)}")
    return freeze


def _scope_consts():
    import topopatch.topo.repair_scope as S
    return {"SPAN_MAX": S.SPAN_MAX, "DETOUR_MAX": S.DETOUR_MAX,
            "SPAN_WINDOW": S.SPAN_WINDOW,
            "COMPLEX_EDGES": list(S.COMPLEX_EDGES)}


def _topo_consts():
    import topopatch.topo.labels as L
    return {"DEADBAND_M": L.DEADBAND_M, "TIGHT_M": L.TIGHT_M,
            "MIN_SPAN_M": L.MIN_SPAN_M}


def check_freeze(freeze_path, pgm_path, data_dir, split):
    """Called by evaluate_topo --freeze. Returns frozen tau. Raises on
    mismatch. Keys are <ext_dir_basename>/<split>.npz (exact)."""
    fr = json.load(open(freeze_path))
    pgm_sha = sha_file(pgm_path)
    want_maps = fr.get("maps", {})
    base = os.path.basename(pgm_path)
    if base in want_maps and want_maps[base] != pgm_sha:
        raise SystemExit(
            f"[freeze] MISMATCH pgm {base}: frozen={want_maps[base]} "
            f"current={pgm_sha}")
    key = f"{os.path.basename(os.path.normpath(data_dir))}/{split}.npz"
    frozen = fr.get("data_files", {})
    if key in frozen:
        cur = sha_file(os.path.join(data_dir, f"{split}.npz"))
        if frozen[key]["sha"] != cur:
            raise SystemExit(
                f"[freeze] MISMATCH {key}: frozen={frozen[key]['sha']} "
                f"current={cur}")
    else:
        raise SystemExit(f"[freeze] {key} not in frozen data_files")
    # M4: TEST runs record full score distributions at tau=-inf; the
    # per-method VAL operating points (frozen above) are applied in
    # analysis (exact recompute from records). Returns -inf.
    print(f"[freeze] ok sha={fr.get('freeze_sha')} tau=-inf "
          f"(per-method ops in analysis)")
    return float("-inf")


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    w = sub.add_parser("write")
    w.add_argument("--maps", required=True)
    w.add_argument("--data-dev", nargs="*", default=None)
    w.add_argument("--data-test", nargs="+", required=True)
    w.add_argument("--train-cfg", required=True)
    w.add_argument("--op", required=True)
    w.add_argument("--method", nargs="+",
                   default=["model:full"],
                   help="methods whose VAL operating points to freeze")
    w.add_argument("--out", required=True)
    a = ap.parse_args()
    if a.cmd == "write":
        cmd_write(a)


if __name__ == "__main__":
    main()
