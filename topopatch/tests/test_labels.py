"""Unit tests for deterministic topology labels (no ROS, no torch)."""
import math

import numpy as np

from topopatch import TOPO_AMBIGUOUS, TOPO_SIDE_NEG, TOPO_SIDE_POS
from topopatch.topo.labels import (detour_side, label_sample,
                                   rule_clearance)


def _arc(A, B, bulge, n=41):
    A = np.asarray(A, float)
    B = np.asarray(B, float)
    t = np.linspace(0, 1, n)
    straight = A[None, :] + t[:, None] * (B - A)[None, :]
    u = (B - A) / np.hypot(*(B - A))
    nrm = np.array([-u[1], u[0]])
    bump = np.sin(t * math.pi) * bulge
    return straight + bump[:, None] * nrm[None, :]


def test_side_pos_over_top():
    A, B = (0.0, 0.0), (6.0, 0.0)
    repl = _arc(A, B, 1.5)
    cls, s, m = detour_side(A, B, repl)
    assert cls == TOPO_SIDE_POS and s > 1.0 and m > 0


def test_side_neg_underneath():
    A, B = (0.0, 0.0), (6.0, 0.0)
    repl = _arc(A, B, -1.5)
    cls, s, m = detour_side(A, B, repl)
    assert cls == TOPO_SIDE_NEG and s < -1.0 and m > 0


def test_deadband_ambiguous():
    A, B = (0.0, 0.0), (6.0, 0.0)
    repl = _arc(A, B, 0.1)
    cls, s, m = detour_side(A, B, repl)
    assert cls == TOPO_AMBIGUOUS and m < 0


def test_short_span_ambiguous():
    A, B = (0.0, 0.0), (0.5, 0.0)
    repl = _arc(A, B, 1.0, n=11)
    cls, _, _ = detour_side(A, B, repl)
    assert cls == TOPO_AMBIGUOUS


def test_label_single_rect():
    A, B = (0.0, 0.0), (6.0, 0.0)
    repl = _arc(A, B, 1.5)
    rect = [(2.0, -0.5), (2.0, 0.5), (4.0, 0.5), (4.0, -0.5)]
    lab = label_sample(A, B, repl, [rect])
    assert lab["cls"] == TOPO_SIDE_POS
    assert not lab["ambiguous"] and not lab["corridor"]
    assert lab["per_rule"][0]["side"] == "P"
    assert lab["per_rule"][0]["clearance_m"] > 0.15
    assert lab["signature"].startswith("S:P|R0:P/")


def test_label_l_shape():
    A, B = (0.0, 0.0), (6.0, 0.0)
    repl = _arc(A, B, -1.8)
    ell = [(2.0, -0.4), (2.0, 0.6), (3.0, 0.6), (3.0, 0.2),
           (4.5, 0.2), (4.5, -0.4)]
    lab = label_sample(A, B, repl, [ell])
    assert lab["cls"] == TOPO_SIDE_NEG
    assert lab["signature"].startswith("S:N|")


def test_corridor_between_two_rules():
    A, B = (0.0, 0.0), (8.0, 0.0)
    # straight-ish path threading between an upper and a lower rect
    t = np.linspace(0, 1, 41)
    repl = np.stack([8 * t, 0.15 * np.sin(t * math.pi)], axis=1)
    upper = [(2.0, 0.8), (2.0, 1.8), (6.0, 1.8), (6.0, 0.8)]
    lower = [(2.0, -1.8), (2.0, -0.8), (6.0, -0.8), (6.0, -1.8)]
    lab = label_sample(A, B, repl, [upper, lower])
    assert lab["corridor"]
    sides = sorted(q["side"] for q in lab["per_rule"])
    assert sides == ["N", "P"]


def test_tight_clearance_ambiguous():
    A, B = (0.0, 0.0), (6.0, 0.0)
    repl = _arc(A, B, 0.62)
    # rect top edge grazes the arc apex (clearance < 0.15)
    rect = [(2.0, -2.0), (2.0, 0.5), (4.0, 0.5), (4.0, -2.0)]
    lab = label_sample(A, B, repl, [rect])
    assert lab["ambiguous"] and lab["cls"] == TOPO_AMBIGUOUS


def test_rule_clearance_values():
    seg = [(0.0, 0.0), (1.0, 0.0)]
    poly = [(2.0, -1.0), (2.0, 1.0), (3.0, 1.0), (3.0, -1.0)]
    assert abs(rule_clearance(seg, poly) - 1.0) < 1e-9


def test_empty_inputs_ambiguous():
    lab = label_sample((0, 0), (0, 0), [], [])
    assert lab["cls"] == TOPO_AMBIGUOUS and lab["ambiguous"]
