"""Dataset shards: loading, region splits, batching (numpy + torch)."""
import glob
import hashlib
import json
import os

import numpy as np

from . import TEST_REGION_X_MIN, TRAIN_SPLIT, VAL_SPLIT, TEST_SPLIT


def sha256_file(path, chunk=1 << 20) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while True:
            b = f.read(chunk)
            if not b:
                break
            h.update(b)
    return h.hexdigest()


def load_shards(data_dir):
    files = sorted(glob.glob(os.path.join(data_dir, "shard_*.npz")))
    recs = []
    for fp in files:
        z = np.load(fp, allow_pickle=True)
        n = int(z["n"][0]) if "n" in z else len(z["start"])
        uids = [str(u) for u in z["uids"]] if "uids" in z else None
        for i in range(n):
            recs.append({
                "shard": os.path.basename(fp), "idx": i,
                "uid": uids[i] if uids is not None else
                f"{os.path.basename(fp)}:{i}",
                "start": z["start"][i], "goal": z["goal"][i],
                "orig": z[f"orig_{i}"], "repl": z[f"repl_{i}"],
                "rect": z[f"rect_{i}"],
                "orig_len": float(z["orig_len"][i]),
                "repl_len": float(z["repl_len"][i]),
            })
    return recs


def load_split(data_dir, split):
    """Load all shard files <split>_*.npz eagerly into a list of dicts."""
    files = sorted(glob.glob(os.path.join(data_dir, f"{split}_*.npz")))
    out = []
    for fp in files:
        z = np.load(fp, allow_pickle=True)
        n = int(z["n"][0])
        uids = [str(u) for u in z["uids"]]
        for i in range(n):
            d = {k: np.array(z[f"{k}_{i}"]) for k in
                 ("bev", "local", "tgt", "center", "A", "B", "rect",
                  "goal", "orig_seg", "repl_seg", "orig_full", "repl_full",
                  "idx")}
            d["uid"] = uids[i]
            out.append(d)
        z.close()
    return out


def write_split_files(items, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    by = {TRAIN_SPLIT: [], VAL_SPLIT: [], TEST_SPLIT: []}
    for it in items:
        by[it["split"]].append(it["uid"])
    paths = {}
    for k, v in by.items():
        p = os.path.join(out_dir, f"{k}.json")
        with open(p, "w") as f:
            json.dump({"count": len(v), "uids": v}, f)
        paths[k] = p
    return paths
