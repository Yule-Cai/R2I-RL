"""Supervised training for TopoRulePatchNet (torch). No RL/LLM/diffusion.

Usage:
  PYTHONPATH=<repo> python3 -m topopatch.topo.train_topo --data <extracted> \
      --out <run_dir> [--epochs N] [--seed S] [--lr LR] [--init CKPT]
"""
import argparse
import json
import os
import time

import numpy as np
import torch
import torch.utils.data as tud

from topopatch import TOPO_AMBIGUOUS
from topopatch.topo.labels import label_sample
from topopatch.topo.model_topo import (TopoRulePatchNet, count_params,
                                       topo_wta_loss)


def _augment(bev4, local, tgt, dense, polys_n, rule_feats, center, A, B,
             repl_seg, rng):
    """Rigid aug (flips + k*90) + label recompute (side swaps on flips).

    All *_norm arrays are center-relative; world_to_norm callers provide
    norm coords. Rule feats: centroid cols transform, w/h swap on odd
    rotations, area/nverts invariant. Returns (bev, local, tgt, dense,
    polys_n, rule_feats, topo_cls, topo_amb).
    """
    flipx = bool(rng.randint(0, 2))
    flipy = bool(rng.randint(0, 2))
    rot = int(rng.randint(0, 4))

    def xy_t(p):
        q = np.array(p, copy=True, dtype=np.float64)
        if flipx:
            q[..., 0] = -q[..., 0]
        if flipy:
            q[..., 1] = -q[..., 1]
        for _ in range(rot):
            q = np.stack([-q[..., 1], q[..., 0]], axis=-1)
        return q.astype(np.float32)

    b = np.array(bev4, copy=True)
    if flipx:
        b = b[:, :, ::-1]
    if flipy:
        b = b[:, ::-1, :]
    if rot:
        b = np.rot90(b, rot, axes=(1, 2)).copy()
    b = np.ascontiguousarray(b)
    rf = np.array(rule_feats, copy=True, dtype=np.float32)
    cent = xy_t(rf[:, 1:3])
    rf[:, 1:3] = cent
    if rot % 2 == 1:
        rf[:, 3], rf[:, 4] = rf[:, 4].copy(), rf[:, 3].copy()
    local2 = xy_t(local)
    tgt2 = xy_t(tgt)
    dense2 = xy_t(dense)
    polys2 = xy_t(polys_n)
    A2 = tuple(xy_t(np.asarray(A, dtype=np.float64)).tolist())
    B2 = tuple(xy_t(np.asarray(B, dtype=np.float64)).tolist())
    repl2 = xy_t(np.asarray(repl_seg, dtype=np.float64))
    polys2 = xy_t(polys_n)
    # label_sample thresholds are in meters; augmentation works in
    # window-normalized units -> scale back before labeling. Drop
    # zero-padded rule slots (they are not rules).
    from rulepatch.rp import BEV_SIZE_M
    half = BEV_SIZE_M / 2.0
    valid = [np.asarray(polys2[k]) * half
             for k in range(polys2.shape[0])
             if float(np.abs(polys2[k]).sum()) > 1e-9]
    lab = label_sample(tuple((np.asarray(A2) * half).tolist()),
                       tuple((np.asarray(B2) * half).tolist()),
                       np.asarray(repl2) * half, valid)
    return (b, local2, tgt2, dense2, polys2, rf, int(lab["cls"]),
            bool(lab["ambiguous"]))


class TopoSplitDS(tud.Dataset):
    def __init__(self, data_dir, split, aug=False, seed=0):
        from topopatch.topo.extract_topo import load_topo_split_items \
            as _load
        self.items = _load(data_dir, split)
        self.aug = bool(aug)
        self.rng = np.random.RandomState(seed + 2000)
        assert len(self.items) > 0, f"empty split {split}"

    def __len__(self):
        return len(self.items)

    def __getitem__(self, i):
        from rulepatch.rp.bev import add_coords, world_to_norm
        from rulepatch.rp.geom import resample_polyline
        s = self.items[i]
        bev = s["bev"].astype(np.float32) / 255.0
        local = s["local"].astype(np.float32)
        tgt = s["tgt"].astype(np.float32)
        center = tuple(s["center"])
        dense = world_to_norm(resample_polyline(s["repl_seg"], 61),
                              center).astype(np.float32)
        polys_n = world_to_norm(
            np.asarray(s["polys"], dtype=np.float64).reshape(-1, 2),
            center).reshape(3, 12, 2).astype(np.float32)
        pn = np.asarray(s["polys_n"], dtype=np.int64)
        rf = s["rule_feats"].astype(np.float32)
        cls = int(s["topo_cls"][0])
        amb = bool(cls == TOPO_AMBIGUOUS)
        if self.aug:
            (bev, local, tgt, dense, polys_n, rf, cls,
             amb) = _augment(bev, local, tgt, dense, polys_n, rf, center,
                             tuple(s["A"]), tuple(s["B"]), s["repl_seg"],
                             self.rng)
        bev = add_coords(bev)
        # M4 scope GT: rigid-invariant (lengths) -> identical pre/post aug.
        from topopatch.topo.repair_scope import repairability_gt
        _sg = repairability_gt(s["orig_seg"], s["repl_seg"],
                               tuple(s["A"]), tuple(s["B"]))
        return (torch.from_numpy(bev), torch.from_numpy(local),
                torch.from_numpy(tgt), torch.from_numpy(dense),
                torch.from_numpy(polys_n), torch.from_numpy(pn),
                torch.from_numpy(rf),
                torch.tensor(cls, dtype=torch.long),
                torch.tensor(amb, dtype=torch.bool),
                torch.from_numpy(s["bev"][0:1].astype(np.float32) / 255.0),
                torch.tensor(_sg["repairable"], dtype=torch.long),
                torch.tensor(_sg["span_m"], dtype=torch.float32),
                torch.tensor(_sg["complexity"], dtype=torch.long))


def run(data_dir, out_dir, epochs=60, batch=64, lr=3e-4, seed=0,
        workers=4, margin_lambda=2.0, margin=0.0625, occ_lambda=0.5,
        curv_lambda=0.2, init_ckpt=None, no_topo=False):
    os.makedirs(out_dir, exist_ok=True)
    torch.manual_seed(seed)
    np.random.seed(seed)
    dev = torch.device("cpu")
    train_ds = TopoSplitDS(data_dir, "train", aug=True, seed=seed)
    val_ds = TopoSplitDS(data_dir, "val")
    tl = tud.DataLoader(train_ds, batch_size=batch, shuffle=True,
                        num_workers=workers)
    vl = tud.DataLoader(val_ds, batch_size=batch, num_workers=workers)
    model = TopoRulePatchNet().to(dev)
    if init_ckpt:
        _ck = torch.load(init_ckpt, map_location="cpu", weights_only=False)
        model.load_state_dict(_ck["model"])
        print(f"[train] init from {init_ckpt}", flush=True)
    n_params = count_params(model)
    opt = torch.optim.Adam(model.parameters(), lr=lr, weight_decay=1e-4)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, epochs)
    try:
        man = json.load(open(os.path.join(data_dir, "MANIFEST.json")))
        data_hash = man.get("merkle_like", "") or str(
            sorted(man.get("files", {}).keys()))
    except Exception:
        data_hash = "unknown"
    cfg = {"arch": "TopoRulePatchNet", "m_hyp": 4, "n_topo": 3,
           "epochs": epochs, "batch": batch, "lr": lr, "seed": seed,
           "n_params": n_params, "n_train": len(train_ds),
           "n_val": len(val_ds), "in_ch": 6,
           "margin_lambda": margin_lambda, "margin": margin,
           "occ_lambda": occ_lambda, "curv_lambda": curv_lambda,
           "data_hash": data_hash, "no_topo": bool(no_topo),
           "repair_w": 1.0, "span_w": 0.05, "cx_w": 0.5}
    # BCE pos weight from train GT balance (deterministic).
    from topopatch.topo.repair_scope import repairability_gt as _rgt
    _n1 = _n0 = 0
    for _s in train_ds.items:
        _g = _rgt(_s["orig_seg"], _s["repl_seg"], tuple(_s["A"]),
                  tuple(_s["B"]))
        if _g["repairable"] == 1:
            _n1 += 1
        else:
            _n0 += 1
    repair_pos_weight = float(_n0 / max(1, _n1))
    cfg["repair_pos_weight"] = repair_pos_weight
    cfg["train_local_frac"] = float(_n1 / max(1, _n0 + _n1))
    print(f"[train] scope GT: local={_n1} nonlocal={_n0} "
          f"pos_w={repair_pos_weight:.2f}", flush=True)
    topo_w = 0.0 if no_topo else 1.0
    mode_cond = not no_topo
    scope_kw = {"repair_w": 1.0, "span_w": 0.05, "cx_w": 0.5,
                "repair_pos_weight": repair_pos_weight}
    json.dump(cfg, open(os.path.join(out_dir, "config.json"), "w"), indent=1)
    hist = {"train_loss": [], "val_min_mse": [], "val_mode_acc": [],
            "val_mode_acc_hard": [], "lr": []}
    best, best_ep = 1e9, -1
    for ep in range(epochs):
        model.train()
        tl_sum, tl_n = 0.0, 0
        t0 = time.time()
        for batch_t in tl:
            (bev, loc, tgt, dense, polys, pn, rf, cls, amb,
             occ, rep, span, cx) = batch_t
            bev, loc, tgt = bev.to(dev), loc.to(dev), tgt.to(dev)
            dense, polys = dense.to(dev), polys.to(dev)
            pn, rf = pn.to(dev), rf.to(dev)
            cls, amb, occ = cls.to(dev), amb.to(dev), occ.to(dev)
            rep = rep.to(dev)
            span, cx = span.to(dev), cx.to(dev)
            pc, pf, pt, ps = model(bev, loc, rf)
            loss, _ = topo_wta_loss(pc, pf, pt, tgt, dense, cls, amb,
                                    polys, pn, occ,
                                    margin_lambda=margin_lambda,
                                    margin=margin, occ_lambda=occ_lambda,
                                    curv_lambda=curv_lambda,
                                    topo_weight=topo_w,
                                    mode_condition=mode_cond,
                                    scope=ps, tgt_repair=rep,
                                    tgt_span=span, tgt_cx=cx,
                                    **scope_kw)
            opt.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            opt.step()
            tl_sum += float(loss.detach()) * len(bev)
            tl_n += len(bev)
        sched.step()
        model.eval()
        vm_sum, vm_n, va_sum, vah_sum, vah_n = 0.0, 0, 0.0, 0.0, 0
        vr_sum, vs_sum, vcx_sum = 0.0, 0.0, 0.0
        with torch.no_grad():
            for batch_t in vl:
                (bev, loc, tgt, dense, polys, pn, rf, cls, amb,
                 occ, rep, span, cx) = batch_t
                pc, pf, pt, ps = model(bev.to(dev), loc.to(dev),
                                       rf.to(dev))
                _, parts = topo_wta_loss(
                    pc, pf, pt, tgt.to(dev), dense.to(dev),
                    cls.to(dev), amb.to(dev), polys.to(dev),
                    pn.to(dev), occ.to(dev),
                    margin_lambda=margin_lambda, margin=margin,
                    occ_lambda=occ_lambda, curv_lambda=curv_lambda,
                    topo_weight=topo_w, mode_condition=mode_cond,
                    scope={k: (v.to(dev) if torch.is_tensor(v) else v)
                           for k, v in ps.items()},
                    tgt_repair=rep.to(dev), tgt_span=span.to(dev),
                    tgt_cx=cx.to(dev), **scope_kw)
                vm_sum += parts["min_mse"] * len(bev)
                vm_n += len(bev)
                va_sum += parts["mode_acc"] * len(bev)
                hard = parts["mode_acc_hard"]
                if hard == hard:  # not NaN (no hard samples in batch)
                    vah_sum += hard * len(bev)
                    vah_n += len(bev)
                vr_sum += parts["rep_acc"] * len(bev)
                vs_sum += parts["span_mae"] * len(bev)
                vcx_sum += parts["cx_acc"] * len(bev)
        vm = vm_sum / max(1, vm_n)
        hist["train_loss"].append(tl_sum / max(1, tl_n))
        hist["val_min_mse"].append(vm)
        hist["val_mode_acc"].append(va_sum / max(1, vm_n))
        hist["val_mode_acc_hard"].append(vah_sum / max(1, vah_n))
        hist["val_rep_acc"] = hist.get("val_rep_acc", [])
        hist["val_rep_acc"].append(vr_sum / max(1, vm_n))
        hist["val_span_mae"] = hist.get("val_span_mae", [])
        hist["val_span_mae"].append(vs_sum / max(1, vm_n))
        hist["val_cx_acc"] = hist.get("val_cx_acc", [])
        hist["val_cx_acc"].append(vcx_sum / max(1, vm_n))
        hist["lr"].append(sched.get_last_lr()[0])
        if vm < best:
            best, best_ep = vm, ep
            torch.save({"model": model.state_dict(), "cfg": cfg},
                       os.path.join(out_dir, "best.pt"))
            from topopatch.topo.infer_numpy_topo import fuse_topo_state
            fused = fuse_topo_state(model.state_dict())
            fused["m_hyp"] = np.array([4])
            fused["n_ctrl"] = np.array([model.n_ctrl])
            fused["n_topo"] = np.array([model.n_topo])
            np.savez(os.path.join(out_dir, "best_fused.npz"), **fused)
        print(f"[train] ep={ep} loss={hist['train_loss'][-1]:.4f} "
              f"val_min_mse={vm:.5f} best={best:.5f}@{best_ep} "
              f"vacc={hist['val_mode_acc'][-1]:.3f} "
              f"vacch={hist['val_mode_acc_hard'][-1]:.3f} "
              f"repacc={hist['val_rep_acc'][-1]:.3f} "
              f"spanmae={hist['val_span_mae'][-1]:.3f} "
              f"cxacc={hist['val_cx_acc'][-1]:.3f} "
              f"dt={time.time()-t0:.0f}s", flush=True)
    torch.save({"model": model.state_dict(), "cfg": cfg},
               os.path.join(out_dir, "last.pt"))
    json.dump(hist, open(os.path.join(out_dir, "curves.json"), "w"))
    return {"best_val": best, "best_ep": best_ep, "n_params": n_params}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--epochs", type=int, default=60)
    ap.add_argument("--batch", type=int, default=64)
    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--margin-lambda", type=float, default=3.0)
    ap.add_argument("--margin", type=float, default=0.0625)
    ap.add_argument("--occ-lambda", type=float, default=1.0)
    ap.add_argument("--curv-lambda", type=float, default=0.2)
    ap.add_argument("--init", default=None)
    ap.add_argument("--no-topo", action="store_true",
                    help="ablation: disable topology CE + mode conditioning")
    a = ap.parse_args()
    print(json.dumps(run(a.data, a.out, a.epochs, a.batch, a.lr, a.seed,
                         a.workers, a.margin_lambda, a.margin,
                         a.occ_lambda, a.curv_lambda, a.init,
                         no_topo=a.no_topo), indent=1))


if __name__ == "__main__":
    main()
