"""Run topo extraction for one map + write manifest.

Usage:
  PYTHONPATH=<repo> python3 topopatch/tools/run_topo_extract.py \
      --data <raw_shards_dir> --pgm <map.pgm> --res-in 0.05 \
      --ox -10 --oy -10 --split train|test --out <extracted_dir>

Split assignment is MAP-LEVEL: run once per map with --split train (uid-hash
val subdivision inside) or --split test (all test). Writes {split}.npz +
META.json + MANIFEST.json.
"""
import argparse
import glob
import json
import os

from rulepatch.rp.dataset import sha256_file
from topopatch.topo.extract_topo import run_topo_extraction


def load_topo_split_items(data_dir, split):
    from topopatch.topo.extract_topo import load_topo_split_items as _l
    return _l(data_dir, split)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True)
    ap.add_argument("--pgm", required=True)
    ap.add_argument("--res-in", type=float, default=0.05)
    ap.add_argument("--ox", type=float, default=-10.0)
    ap.add_argument("--oy", type=float, default=-10.0)
    ap.add_argument("--split", required=True, choices=["train", "val", "test"])
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    from PIL import Image
    w, h = Image.open(a.pgm).size
    bounds = (a.ox, a.oy, a.ox + w * a.res_in, a.oy + h * a.res_in)
    out = run_topo_extraction(a.data, a.pgm, bounds, a.out, a.split,
                              res_in=a.res_in, ox=a.ox, oy=a.oy)
    manifest = {"pgm_sha256": sha256_file(a.pgm),
                "res_in": a.res_in, "ox": a.ox, "oy": a.oy,
                "bounds": bounds, "extract": out, "files": {}}
    for fp in sorted(glob.glob(os.path.join(a.out, "*.npz"))):
        manifest["files"][os.path.basename(fp)] = {
            "bytes": os.path.getsize(fp), "sha256": sha256_file(fp)}
    with open(os.path.join(a.out, "MANIFEST.json"), "w") as f:
        json.dump(manifest, f, indent=1)
    print(json.dumps(manifest, indent=1)[:1500])


if __name__ == "__main__":
    main()
