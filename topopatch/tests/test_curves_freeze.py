"""Unit tests for coverage curves + freeze check (no ROS, no torch)."""
import json
import os
import tempfile

import numpy as np

from topopatch.topo.coverage_curves import metrics_at_tau


def _rec(mc, accepted, cl=True, cl_fb=True):
    return {"method": "model:full", "uid": "u", "map_id": "m",
            "max_conf": mc, "accepted": accepted, "reason": "ok",
            "native": {"compliant": True, "valid": True},
            "closed_loop": cl, "closed_loop_fb": cl_fb,
            "len_ratio": 1.1, "len_ratio_fb": 1.4,
            "topo_cls": 1, "n_rules": 1, "geoms": "rect"}


def test_abstain_uses_fallback_branch():
    recs = [_rec(0.9, True, cl=True), _rec(0.1, True, cl=True),
            _rec(0.8, False, cl=False)]  # rejected -> fallback ok
    m = metrics_at_tau(recs, 0.5)
    assert abs(m["coverage"] - 2 / 3) < 1e-9
    assert abs(m["verifier_acceptance_rate"] - 1 / 3) < 1e-9
    assert abs(m["fallback_rate"] - 2 / 3) < 1e-9
    assert abs(m["all_case_success"] - 1.0) < 1e-9
    assert m["native_rule_compliance"] == 1.0


def test_tau_minus_inf_no_abstention():
    recs = [_rec(0.9, True), _rec(0.0, False, cl=False, cl_fb=False)]
    m = metrics_at_tau(recs, float("-inf"))
    assert m["coverage"] == 1.0
    assert abs(m["all_case_success"] - 0.5) < 1e-9


def test_freeze_write_and_check(tmp_path=None):
    from topopatch.topo.freeze import cmd_write, check_freeze

    class A:
        pass
    d = tempfile.mkdtemp()
    os.makedirs(os.path.join(d, "maps"))
    open(os.path.join(d, "maps", "m.pgm"), "wb").write(b"P5fake")
    os.makedirs(os.path.join(d, "test"))
    np.savez_compressed(os.path.join(d, "test", "test.npz"), n=np.array([1]))
    json.dump({"m_hyp": 4, "seed": 0}, open(os.path.join(d, "cfg.json"), "w"))
    json.dump({"model:full": {"tau": 0.3, "all_case_success": 0.9,
                              "coverage": 0.8}},
              open(os.path.join(d, "op.json"), "w"))
    a = A()
    a.maps = os.path.join(d, "maps")
    a.data_dev = None
    a.data_test = [os.path.join(d, "test")]
    a.train_cfg = os.path.join(d, "cfg.json")
    a.op = os.path.join(d, "op.json")
    a.method = ["model:full"]
    a.out = os.path.join(d, "freeze.json")
    fr = cmd_write(a)
    # M4 contract: hashes verified, per-method VAL ops frozen in the
    # file; TEST runs record at tau=-inf, ops applied in analysis.
    assert fr["operating_points"]["model:full"]["tau"] == 0.3
    tau = check_freeze(a.out, os.path.join(d, "maps", "m.pgm"),
                       os.path.join(d, "test"), "test")
    assert tau == float("-inf")
    # tamper with data -> hard error
    np.savez_compressed(os.path.join(d, "test", "test.npz"),
                        n=np.array([2]))
    try:
        check_freeze(a.out, os.path.join(d, "maps", "m.pgm"),
                     os.path.join(d, "test"), "test")
    except SystemExit:
        pass
    else:
        raise AssertionError("tampered data must fail freeze check")
