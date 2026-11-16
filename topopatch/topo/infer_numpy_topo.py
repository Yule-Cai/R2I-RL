"""Numpy-only TopoRulePatchNet forward (ROS deployment, no torch).

Mirrors topopatch/topo/model_topo.py with BatchNorm folded at export.
Validated by parity test vs torch (test_infer_numpy_topo.py).
"""
import numpy as np


def _conv_relu(x, w, b):
    """x: (C,H,W), w: (O,C,3,3) stride-2 pad-1 + bias + ReLU."""
    C, H, W = x.shape
    O = w.shape[0]
    xp = np.pad(x, ((0, 0), (1, 1), (1, 1)), mode="constant")
    H2, W2 = H // 2, W // 2
    s0, s1, s2 = xp.strides
    shape = (H2, W2, C, 3, 3)
    strides = (2 * s1, 2 * s2, s0, s1, s2)
    win = np.lib.stride_tricks.as_strided(xp, shape=shape, strides=strides)
    y = np.tensordot(win, w, axes=([2, 3, 4], [1, 2, 3]))
    y = y.transpose(2, 0, 1) + b[:, None, None]
    np.maximum(y, 0, out=y)
    return y


def _linear(x, w, b):
    return x @ w.T + b


class NumpyTopoNet:
    def __init__(self, npz_path):
        z = np.load(npz_path, allow_pickle=True)
        self.p = dict(z)
        self.m_hyp = int(z["m_hyp"][0]) if "m_hyp" in z else 4
        self.n_ctrl = int(z["n_ctrl"][0]) if "n_ctrl" in z else 6
        self.n_topo = int(z["n_topo"][0]) if "n_topo" in z else 3

    def forward(self, bev, local_pts, rule_feats):
        p = self.p
        x = np.asarray(bev, dtype=np.float64)
        for i in range(4):
            x = _conv_relu(x, p[f"enc.{i * 2}.w"], p[f"enc.{i * 2}.b"])
        f = _linear(x.reshape(-1), p["spatial_fc.0.w"],
                    p["spatial_fc.0.b"])
        f = np.maximum(f, 0)
        g = _linear(np.asarray(local_pts, dtype=np.float64).reshape(-1),
                    p["path_mlp.0.w"], p["path_mlp.0.b"])
        g = np.maximum(g, 0)
        g = _linear(g, p["path_mlp.2.w"], p["path_mlp.2.b"])
        g = np.maximum(g, 0)
        r = _linear(np.asarray(rule_feats, dtype=np.float64).reshape(-1),
                    p["rule_mlp.0.w"], p["rule_mlp.0.b"])
        r = np.maximum(r, 0)
        r = _linear(r, p["rule_mlp.2.w"], p["rule_mlp.2.b"])
        r = np.maximum(r, 0)
        h = _linear(np.concatenate([f, g, r]), p["fuse.0.w"],
                    p["fuse.0.b"])
        h = np.maximum(h, 0)
        # softmax for topology probs
        tl = _linear(h, p["topo_head.w"], p["topo_head.b"])
        tl = tl - tl.max()
        topo_probs = np.exp(tl) / np.exp(tl).sum()
        # M4 scope heads (deployment parity with model_topo.py).
        # Old (M3) fused weights lack these keys -> scope None (caller
        # falls back to no-scope selection).
        if "repair_head.w" not in p:
            scope = None
        else:
            rep_logit = float(_linear(h, p["repair_head.w"],
                                      p["repair_head.b"])[0])
            rep_prob = float(1.0 / (1.0 + np.exp(-rep_logit)))
            span_pred = float(_linear(h, p["span_head.w"],
                                      p["span_head.b"])[0])
            cl = _linear(h, p["complexity_head.w"],
                         p["complexity_head.b"])
            cl = cl - cl.max()
            cx_probs = np.exp(cl) / np.exp(cl).sum()
            scope = {"rep_prob": rep_prob, "span_pred": span_pred,
                     "cx_pred": int(np.argmax(cx_probs)),
                     "cx_probs": cx_probs}
        ctrls, confs = [], []
        for m in range(self.m_hyp):
            o = _linear(h, p[f"heads.{m}.w"], p[f"heads.{m}.b"])
            ctrls.append(np.tanh(o[:self.n_ctrl * 2]).reshape(self.n_ctrl, 2))
            confs.append(o[self.n_ctrl * 2])
        return np.stack(ctrls), np.array(confs), topo_probs, scope


def fuse_topo_state(sd):
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
    for key in ("path_mlp.0", "path_mlp.2", "rule_mlp.0", "rule_mlp.2",
                "fuse.0", "spatial_fc.0", "topo_head", "repair_head",
                "span_head", "complexity_head"):
        out[f"{key}.w"] = sd[f"{key}.weight"].detach().numpy().astype(np.float64)
        out[f"{key}.b"] = sd[f"{key}.bias"].detach().numpy().astype(np.float64)
    m = 0
    while f"heads.{m}.weight" in sd:
        out[f"heads.{m}.w"] = sd[f"heads.{m}.weight"].detach().numpy().astype(np.float64)
        out[f"heads.{m}.b"] = sd[f"heads.{m}.bias"].detach().numpy().astype(np.float64)
        m += 1
    return out
