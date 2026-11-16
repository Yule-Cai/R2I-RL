import numpy as np

from rp.extract import extract_sample


def occ_free(x, y):
    return False


BOUNDS = (-20, -20, 20, 20)


def synth_pair(uid="rp000001"):
    xs = np.linspace(-5, 5, 101)
    orig = np.stack([xs, np.zeros_like(xs)], axis=1)
    repl = np.stack([xs, np.where((xs > -2) & (xs < 2),
                                  1.0 * np.sin((xs + 2) / 4 * np.pi), 0.0)], axis=1)
    rect = [(-0.6, -0.6), (-0.6, 0.6), (0.6, 0.6), (0.6, -0.6)]
    return {"uid": uid, "orig": orig, "repl": repl, "rect": rect,
            "goal": (5.0, 0.0), "start": (-5.0, 0.0)}


def test_extract_synth():
    stats: dict = {}
    s, reason = extract_sample(synth_pair(), occ_free, BOUNDS, stats)
    assert s is not None, reason
    assert s["bev"].shape == (4, 128, 128)
    assert s["local"].shape == (32, 2) and s["tgt"].shape == (6, 2)
    assert s["split"] in ("train", "val", "test")


def test_extract_test_region():
    rec = synth_pair("rp000002")
    rec["orig"] = rec["orig"] + np.array([14.0, 10.0])
    rec["repl"] = rec["repl"] + np.array([14.0, 10.0])
    rec["rect"] = [(x + 14.0, y + 10.0) for x, y in rec["rect"]]
    rec["goal"] = (19.0, 10.0)
    s, reason = extract_sample(rec, occ_free, (-20, -20, 20, 20))
    assert s is not None, reason
    assert s["split"] == "test", s["split"]
