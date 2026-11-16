"""Supervised training for RulePatchNet (torch). No RL/LLM/diffusion.

Usage:
  PYTHONPATH=rulepatch python3 -m rp.train --data <extracted_dir> \
      --out <run_dir> [--m-hyp 1|3] [--zero-rule] [--epochs N] [--seed S]
"""
import argparse
import json
import os
import time

import numpy as np
import torch
import torch.utils.data as tud

from .model import RulePatchNet, count_params, wta_loss


def augment(bev4, local, tgt, dense, rect, rng):
    """Label-preserving rigid augmentation of the BEV window contents.

    Square window: flips + k*90 rotations about the center. Applied to the
    4 content channels; coords recomputed downstream. All *_norm arrays are
    center-relative so transforms apply directly.
    """
    flipx = bool(rng.randint(0, 2))
    flipy = bool(rng.randint(0, 2))
    rot = int(rng.randint(0, 4))

    def xy_t(p):
        q = p.copy()
        if flipx:
            q[..., 0] = -q[..., 0]
        if flipy:
            q[..., 1] = -q[..., 1]
        for _ in range(rot):
            q = np.stack([-q[..., 1], q[..., 0]], axis=-1)
        return q

    b = bev4.copy()
    if flipx:
        b = b[:, :, ::-1]
    if flipy:
        b = b[:, ::-1, :]
    if rot:
        b = np.rot90(b, rot, axes=(1, 2))
    b = np.ascontiguousarray(b)
    return (b, xy_t(local), xy_t(tgt), xy_t(dense), xy_t(rect))


class SplitDS(tud.Dataset):
    def __init__(self, data_dir, split, zero_rule=False, aug=False, seed=0):
        from .dataset import load_split
        self.items = load_split(data_dir, split)
        self.zero_rule = bool(zero_rule)
        self.aug = bool(aug)
        self.rng = np.random.RandomState(seed + 1000)
        assert len(self.items) > 0, f"empty split {split}"

    def __len__(self):
        return len(self.items)

    def __getitem__(self, i):
        from .model import add_coords
        from .geom import resample_polyline
        from .bev import world_to_norm
        s = self.items[i]
        bev = s["bev"].astype(np.float32) / 255.0
        if self.zero_rule:
            bev = bev.copy()
            bev[2] = 0.0
        local = s["local"].astype(np.float32)
        tgt = s["tgt"].astype(np.float32)
        center = tuple(s["center"])
        dense = world_to_norm(resample_polyline(s["repl_seg"], 61),
                              center).astype(np.float32)
        rect_n = world_to_norm(np.asarray(s["rect"], dtype=np.float64),
                               center).astype(np.float32)
        if self.aug:
            bev, local, tgt, dense, rect_n = augment(
                bev, local, tgt, dense, rect_n, self.rng)
        bev = add_coords(bev)
        return (torch.from_numpy(bev), torch.from_numpy(local),
                torch.from_numpy(tgt), torch.from_numpy(dense),
                torch.from_numpy(rect_n))


def run(data_dir, out_dir, m_hyp=3, zero_rule=False, epochs=40, batch=64,
        lr=3e-4, seed=0, workers=4, margin_lambda=2.0, margin=0.0625,
        init_ckpt=None):
    os.makedirs(out_dir, exist_ok=True)
    torch.manual_seed(seed)
    np.random.seed(seed)
    dev = torch.device("cpu")
    train_ds = SplitDS(data_dir, "train", zero_rule, aug=True, seed=seed)
    val_ds = SplitDS(data_dir, "val", zero_rule)
    tl = tud.DataLoader(train_ds, batch_size=batch, shuffle=True,
                        num_workers=workers)
    vl = tud.DataLoader(val_ds, batch_size=batch, num_workers=workers)
    model = RulePatchNet(m_hyp=m_hyp).to(dev)
    if init_ckpt:
        _ck = torch.load(init_ckpt, map_location="cpu", weights_only=False)
        model.load_state_dict(_ck["model"])
        print(f"[train] init from {init_ckpt}", flush=True)
    n_params = count_params(model)
    opt = torch.optim.Adam(model.parameters(), lr=lr, weight_decay=1e-4)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, epochs)
    cfg = {"m_hyp": m_hyp, "zero_rule": zero_rule, "epochs": epochs,
           "batch": batch, "lr": lr, "seed": seed, "n_params": n_params,
           "n_train": len(train_ds), "n_val": len(val_ds), "in_ch": 6,
           "margin_lambda": margin_lambda, "margin": margin}
    json.dump(cfg, open(os.path.join(out_dir, "config.json"), "w"), indent=1)
    hist = {"train_loss": [], "val_min_mse": [], "lr": []}
    best, best_ep = 1e9, -1
    for ep in range(epochs):
        model.train()
        tl_sum, tl_n = 0.0, 0
        t0 = time.time()
        for bev, loc, tgt, dense, rect in tl:
            bev, loc, tgt = bev.to(dev), loc.to(dev), tgt.to(dev)
            dense, rect = dense.to(dev), rect.to(dev)
            pc, pf = model(bev, loc)
            loss, _ = wta_loss(pc, pf, tgt, tgt_dense=dense, rect=rect,
                               margin_lambda=margin_lambda, margin=margin)
            opt.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            opt.step()
            tl_sum += float(loss) * len(bev)
            tl_n += len(bev)
        sched.step()
        model.eval()
        vm_sum, vm_n = 0.0, 0
        with torch.no_grad():
            for bev, loc, tgt, dense, rect in vl:
                pc, pf = model(bev.to(dev), loc.to(dev))
                _, parts = wta_loss(pc, pf, tgt.to(dev),
                                    tgt_dense=dense.to(dev),
                                    rect=rect.to(dev))
                vm_sum += parts["min_mse"] * len(bev)
                vm_n += len(bev)
        vm = vm_sum / max(1, vm_n)
        hist["train_loss"].append(tl_sum / max(1, tl_n))
        hist["val_min_mse"].append(vm)
        hist["lr"].append(sched.get_last_lr()[0])
        if vm < best:
            best, best_ep = vm, ep
            torch.save({"model": model.state_dict(), "cfg": cfg},
                       os.path.join(out_dir, "best.pt"))
            from .infer_numpy import fuse_state_dict
            fused = fuse_state_dict(model.state_dict())
            fused["m_hyp"] = np.array([m_hyp])
            fused["n_ctrl"] = np.array([model.n_ctrl])
            np.savez(os.path.join(out_dir, "best_fused.npz"), **fused)
        print(f"[train] ep={ep} loss={hist['train_loss'][-1]:.4f} "
              f"val_min_mse={vm:.5f} best={best:.5f}@{best_ep} "
              f"dt={time.time()-t0:.0f}s", flush=True)
    torch.save({"model": model.state_dict(), "cfg": cfg},
               os.path.join(out_dir, "last.pt"))
    json.dump(hist, open(os.path.join(out_dir, "curves.json"), "w"))
    return {"best_val": best, "best_ep": best_ep, "n_params": n_params}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--m-hyp", type=int, default=3)
    ap.add_argument("--zero-rule", action="store_true")
    ap.add_argument("--epochs", type=int, default=40)
    ap.add_argument("--batch", type=int, default=64)
    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--margin-lambda", type=float, default=3.0)
    ap.add_argument("--margin", type=float, default=0.0625)
    ap.add_argument("--init", default=None,
                    help="checkpoint pt to fine-tune from")
    a = ap.parse_args()
    print(json.dumps(run(a.data, a.out, a.m_hyp, a.zero_rule, a.epochs,
                         a.batch, a.lr, a.seed, a.workers, a.margin_lambda,
                         a.margin, a.init), indent=1))


if __name__ == "__main__":
    main()
