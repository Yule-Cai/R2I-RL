"""Unit tests for procedural map families (no ROS, needs PIL+numpy)."""
import os

import numpy as np

from topopatch.topo import maps as M


def test_determinism():
    for fam in ("office", "maze", "narrow"):
        a = M.GENERATORS[fam](7)
        b = M.GENERATORS[fam](7)
        assert np.array_equal(a, b)
        c = M.GENERATORS[fam](8)
        assert not np.array_equal(a, c)


def test_validation_all_families():
    for fam in ("office", "maze", "narrow"):
        for seed in (M.FAMILY_SEEDS[fam + "-A"], M.FAMILY_SEEDS[fam + "-B"]):
            v = M.validate_map(M.GENERATORS[fam](seed))
            assert v["shape_ok"] and v["values_ok"], (fam, seed, v)
            assert v["free_frac_ok"], (fam, seed, v)
            assert v["connectivity_ok"], (fam, seed, v)


def test_write_map_hashes(tmp_path):
    r1 = M.write_map("office-A", M.FAMILY_SEEDS["office-A"], str(tmp_path))
    assert r1["validation"]["connectivity_ok"]
    assert os.path.exists(r1["pgm"]) and os.path.exists(r1["yaml"])
    assert len(r1["sha256_pgm"]) == 64 and len(r1["sha256_yaml"]) == 64
    # byte-identical regeneration
    r2 = M.write_map("office-A", M.FAMILY_SEEDS["office-A"], str(tmp_path))
    assert r1["sha256_pgm"] == r2["sha256_pgm"]
    assert r1["sha256_yaml"] == r2["sha256_yaml"]


def test_splits_disjoint():
    assert not (set(M.DEV_MAPS) & set(M.HELD_MAPS))
    assert "tb3_sandbox" not in M.DEV_MAPS + M.HELD_MAPS
