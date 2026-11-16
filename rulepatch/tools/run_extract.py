"""Run A/B patch extraction + write dataset manifest with checksums.

Usage (опрос: any machine with the shards + map pgm):
  PYTHONPATH=rulepatch python3 rulepatch/tools/run_extract.py \
      --data ~/rulepatch/data --pgm <warehouse.pgm> --res-in 0.03 \
      --ox -15.1 --oy -25.0 --out ~/rulepatch/extracted
Writes: train/val/test.npz, extraction_stats.json, MANIFEST.json, CHECKSUMS.sha256
"""
import argparse
import hashlib
import json
import os

from rp.dataset import sha256_file
from rp.extract import run_extraction


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True)
    ap.add_argument("--pgm", required=True)
    ap.add_argument("--res-in", type=float, default=0.03)
    ap.add_argument("--ox", type=float, default=-15.1)
    ap.add_argument("--oy", type=float, default=-25.0)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    import glob
    shards = sorted(glob.glob(os.path.join(a.data, "shard_*.npz")))
    print(f"[extract] shards: {len(shards)}", flush=True)
    stats = run_extraction(a.data, a.pgm,
                           (a.ox, a.oy, a.ox + 1006 * a.res_in,
                            a.oy + 1674 * a.res_in),
                           a.out, res_in=a.res_in, ox=a.ox, oy=a.oy)
    manifest = {
        "shards": len(shards),
        "stats": stats,
        "files": {},
    }
    split_files = sorted(glob.glob(os.path.join(a.out, "*_????.npz")))
    for fp in shards + split_files + [os.path.join(a.out, "extraction_stats.json")]:
        if os.path.exists(fp):
            manifest["files"][os.path.basename(fp)] = {
                "bytes": os.path.getsize(fp), "sha256": sha256_file(fp)}
    man_path = os.path.join(a.out, "MANIFEST.json")
    json.dump(manifest, open(man_path, "w"), indent=1)
    with open(os.path.join(a.out, "CHECKSUMS.sha256"), "w") as f:
        for name, meta in sorted(manifest["files"].items()):
            f.write(f"{meta['sha256']}  {name}\n")
    print(json.dumps(stats, indent=1))


if __name__ == "__main__":
    main()
