"""Merge per-map extracted splits into one joint train/val dir.

Usage:
  python3 -m topopatch.tools.merge_extracted <out_dir> <in_dir>...
Each in_dir has train.npz val.npz META.json MANIFEST.json (run_topo_extract
output). Concatenates train->train, val->val; writes MERGE.json with per-map
counts + sha of inputs. Schema must match (asserts key sets equal).
"""
import argparse
import glob
import json
import os

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out_dir")
    ap.add_argument("in_dirs", nargs="+")
    a = ap.parse_args()
    os.makedirs(a.out_dir, exist_ok=True)
    merge = {"inputs": {}, "counts": {}}
    for split in ("train", "val"):
        dicts = []
        for d in a.in_dirs:
            fp = os.path.join(d, f"{split}.npz")
            if not os.path.exists(fp):
                continue
            z = np.load(fp, allow_pickle=True)
            # materialize each column ONCE (see load_topo_split_items
            # incident note: per-(item,key) NpzFile indexing re-decodes
            # the column and pins a fresh base per view -> OOM).
            cols = {k: z[k] for k in z.files if k != "n"}
            n = int(np.asarray(z["n"]).reshape(-1)[0])
            merge["inputs"].setdefault(os.path.basename(d.rstrip("/")),
                                       {})[split] = n
            items = []
            for i in range(n):
                it = {}
                for k, v in cols.items():
                    it[k] = v[i] if (v.shape and v.shape[0] == n) else v
                items.append(it)
            dicts.append(items)
            del z, cols
        if not dicts:
            continue
        keys = set(dicts[0][0].keys())
        assert all(set(it.keys()) == keys for ds in dicts for it in ds), \
            "schema mismatch"
        all_items = [it for ds in dicts for it in ds]
        kw = {"n": np.array([len(all_items)])}
        str_keys = ("uid", "map_id", "geoms", "topo_sig", "split")
        for kk in str_keys:
            kw[kk] = np.array([str(it[kk]) for it in all_items])
        for kk in sorted(keys - set(str_keys)):
            try:
                kw[kk] = np.stack([np.asarray(it[kk]) for it in all_items])
            except ValueError:
                kw[kk] = np.array([np.asarray(it[kk], dtype=object)
                                   for it in all_items], dtype=object)
        kw["topo_corr"] = np.array([int(it["topo_corr"]) for it in
                                    all_items], np.int64)
        np.savez_compressed(os.path.join(a.out_dir, f"{split}.npz"), **kw)
        merge["counts"][split] = len(all_items)
    json.dump(merge, open(os.path.join(a.out_dir, "MERGE.json"), "w"),
              indent=1)
    print(json.dumps(merge, indent=1))


if __name__ == "__main__":
    main()
