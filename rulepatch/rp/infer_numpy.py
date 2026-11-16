"""Numpy-only RulePatchNet forward (for the ROS2 node: no torch onboard).

Matches rp.model.RulePatchNet with BatchNorm folded at export time.
Loads fused weights saved by rp.train (best_fused.npz).
"""
import numpy as np


def _conv_relu(x, w, b):
    """x: (C,H,W), w: (O,C,3,3) stride-2 pad-1 + bias + ReLU -> (O,H/2,W/2)."""
    C, H, W = x.shape
    O = w.shape[0]
    xp = np.pad(x, ((0, 0), (1, 1), (1, 1)), mode="constant")
    H2, W2 = H // 2, W // 2
    s0, s1, s2 = xp.strides
    # window view (H2, W2, C, 3, 3) via striding over even offsets
    shape = (H2, W2, C, 3, 3)
    strides = (2 * s1, 2 * s2, s0, s1, s2)
    win = np.lib.stride_tricks.as_strided(xp, shape=shape, strides=strides)
    y = np.tensordot(win, w, axes=([2, 3, 4], [1, 2, 3]))  # (H2,W2,O)
    y = y.transpose(2, 0, 1) + b[:, None, None]
    np.maximum(y, 0, out=y)
    return y


def _linear(x, w, b):
    return x @ w.T + b


class NumpyPatchNet:
    def __init__(self, npz_path):
        z = np.load(npz_path, allow_pickle=True)
        self.p = dict(z)
        self.m_hyp = int(z["m_hyp"][0]) if "m_hyp" in z else 3
        self.n_ctrl = int(z["n_ctrl"][0]) if "n_ctrl" in z else 6

    def forward(self, bev, local_pts):
        p = self.p
        x = np.asarray(bev, dtype=np.float64)
        for i in range(4):
            x = _conv_relu(x, p[f"enc.{i * 2}.w"], p[f"enc.{i * 2}.b"])
        f = _linear(x.reshape(-1), p["spatial_fc.0.w"], p["spatial_fc.0.b"])
        f = np.maximum(f, 0)
        g = _linear(np.asarray(local_pts, dtype=np.float64).reshape(-1),
                    p["path_mlp.0.w"], p["path_mlp.0.b"])
        g = np.maximum(g, 0)
        g = _linear(g, p["path_mlp.2.w"], p["path_mlp.2.b"])
        g = np.maximum(g, 0)
        h = _linear(np.concatenate([f, g]), p["fuse.0.w"], p["fuse.0.b"])
        h = np.maximum(h, 0)
        ctrls, confs = [], []
        for m in range(self.m_hyp):
            o = _linear(h, p[f"heads.{m}.w"], p[f"heads.{m}.b"])
            ctrls.append(np.tanh(o[:self.n_ctrl * 2]).reshape(self.n_ctrl, 2))
            confs.append(o[self.n_ctrl * 2])
        return np.stack(ctrls), np.array(confs)


def fuse_state_dict(sd):
    """Fold BatchNorm into preceding convs. Returns dict of numpy arrays."""
    out = {}
    # enc layout: enc.{i} = block(conv,bn,relu); conv=enc.{i}.0, bn=enc.{i}.1
    for i in range(4):
        W = sd[f"enc.{i}.0.weight"].detach().numpy().astype(np.float64)
        b = (sd[f"enc.{i}.0.bias"].detach().numpy().astype(np.float64)
             if f"enc.{i}.0.bias" in sd else np.zeros(W.shape[0]))
        bn_w = sd[f"enc.{i}.1.weight"].detach().numpy().astype(np.float64)
        bn_b = sd[f"enc.{i}.1.bias"].detach().numpy().astype(np.float64)
        mu = sd[f"enc.{i}.1.running_mean"].detach().numpy().astype(np.float64)
        va = sd[f"enc.{i}.1.running_var"].detach().numpy().astype(np.float64)
        scale = bn_w / np.sqrt(va + 1e-5)
        out[f"enc.{i * 2}.w"] = (W * scale[:, None, None, None])
        out[f"enc.{i * 2}.b"] = (b - mu) * scale + bn_b
    for key in ("path_mlp.0", "path_mlp.2", "fuse.0", "spatial_fc.0"):
        out[f"{key}.w"] = sd[f"{key}.weight"].detach().numpy().astype(np.float64)
        out[f"{key}.b"] = sd[f"{key}.bias"].detach().numpy().astype(np.float64)
    m = 0
    while f"heads.{m}.weight" in sd:
        out[f"heads.{m}.w"] = sd[f"heads.{m}.weight"].detach().numpy().astype(np.float64)
        out[f"heads.{m}.b"] = sd[f"heads.{m}.bias"].detach().numpy().astype(np.float64)
        m += 1
    return out
