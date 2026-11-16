import numpy as np

from rp.geom import (point_in_polygon, resample_polyline, find_anchors,
                     catmull_rom, max_turn_deg, path_length)


def test_pip_square():
    sq = [(0, 0), (0, 1), (1, 1), (1, 0)]
    assert point_in_polygon((0.5, 0.5), sq)
    assert not point_in_polygon((1.5, 0.5), sq)


def test_resample_even():
    p = resample_polyline([(0, 0), (4, 0)], 5)
    assert p.shape == (5, 2)
    assert abs(p[2, 0] - 2.0) < 1e-9
    assert abs(path_length(p) - 4.0) < 1e-9


def test_anchors_detour():
    xs = np.linspace(0, 10, 201)
    orig = list(zip(xs, np.zeros_like(xs)))
    repl = [(x, 0.0 if (x < 4 or x > 6) else 1.2 * np.sin((x - 4) / 2 * np.pi)) for x in xs]
    anc, reason = find_anchors(orig, repl)
    assert anc is not None, reason
    assert anc["jB"] > anc["jA"]
    assert abs(anc["A"][0] - 4.0) < 0.5 and abs(anc["B"][0] - 6.0) < 0.5


def test_anchors_coincide_invalid():
    o = [(float(x), 0.0) for x in range(10)]
    anc, reason = find_anchors(o, list(o))
    assert anc is None


def test_catmull_endpoints():
    c = [(0, 0), (1, 1), (2, 0), (3, 1)]
    d = catmull_rom(c, samples_per_seg=4)
    assert abs(d[0, 0]) < 1e-9 and abs(d[-1, 0] - 3) < 1e-9
    assert max_turn_deg([(0, 0), (1, 0), (2, 0)]) < 0.01
