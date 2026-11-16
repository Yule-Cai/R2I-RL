"""Procedural navigation map families (numpy + PIL only).

Conventions match Nav2 bringup maps (verified against tb3_sandbox):
  384x384 L-mode pgm, res 0.05, origin (-10,-10,0). Pixel values:
  0 = wall, 205 = unknown, 254 = free. Row 0 of the image is the TOP
  (world ymax); M2's loader flips so row 0 = ymin — same convention here.

Families (all seeded, deterministic):
  office : border + 3x2 room grid, every shared wall has a door gap.
  maze   : DFS perfect maze on 16x16 coarse cells + loop openings.
  narrow : serpentine passages (alternating end gaps) + chambers.

Map IDs: "<family>-<VARIANT>-s<seed>" (e.g. "office-B-s11").
Splits (frozen in topopatch/config/maps_freeze.json at freeze time):
  DEV  = warehouse, depot, office-A, narrow-A, maze-A (+ tb3_sandbox demo)
  HELD = office-B, maze-B, narrow-B (never trained on, one-shot eval).
"""
import hashlib
import json
import os

import numpy as np

SIZE = 384
RES = 0.05
ORIGIN = (-10.0, -10.0, 0.0)
WALL, UNKNOWN, FREE = 0, 205, 254

DEV_MAPS = ["warehouse", "depot", "office-A", "narrow-A", "maze-A"]
HELD_MAPS = ["office-B", "maze-B", "narrow-B"]

# Family default seeds (variant A/B). Change = new map (new hash).
FAMILY_SEEDS = {
    "office-A": 101, "office-B": 102,
    "maze-A": 201, "maze-B": 202,
    "narrow-A": 301, "narrow-B": 302,
}


def _blank():
    return np.full((SIZE, SIZE), UNKNOWN, dtype=np.uint8)


def _border(g, t=4):
    g[:t, :] = WALL
    g[-t:, :] = WALL
    g[:, :t] = WALL
    g[:, -t:] = WALL
    return g


def _hwall(g, row, c0, c1, gaps, t=4):
    """Horizontal wall band rows [row,row+t) over cols, minus gap intervals."""
    for c in range(c0, c1):
        if any(a <= c < b for a, b in gaps):
            continue
        g[row:row + t, c] = WALL
    return g


def _vwall(g, col, r0, r1, gaps, t=4):
    for r in range(r0, r1):
        if any(a <= r < b for a, b in gaps):
            continue
        g[r, col:col + t] = WALL
    return g


def gen_office(seed):
    rng = np.random.RandomState(seed)
    g = _blank()
    g[:, :] = FREE
    _border(g)
    # 3 cols x 2 rows of rooms; walls with one door gap each.
    for cx in (96, 192, 288):
        gy = int(rng.uniform(80, 280))
        _vwall(g, cx, 48, 336, [(gy, gy + 24)])
    for ry in (128, 256):
        gx = int(rng.uniform(80, 280))
        _hwall(g, ry, 48, 336, [(gx, gx + 24)])
    return g


def gen_maze(seed):
    rng = np.random.RandomState(seed)
    g = _blank()
    g[:, :] = WALL
    n, cell = 16, 24  # 16x16 cells of 24px (1.2m corridors)
    assert n * cell == SIZE
    visited = np.zeros((n, n), bool)
    edges = []  # ((cy,cx),(ny,nx)) tree edges in coarse coords
    stack = [(0, 0)]
    visited[0, 0] = True
    while stack:
        cy, cx = stack[-1]
        opts = []
        if cx > 0 and not visited[cy, cx - 1]:
            opts.append((cy, cx - 1))
        if cx < n - 1 and not visited[cy, cx + 1]:
            opts.append((cy, cx + 1))
        if cy > 0 and not visited[cy - 1, cx]:
            opts.append((cy - 1, cx))
        if cy < n - 1 and not visited[cy + 1, cx]:
            opts.append((cy + 1, cx))
        if not opts:
            stack.pop()
            continue
        ny, nx = opts[rng.randint(len(opts))]
        visited[ny, nx] = True
        edges.append(((cy, cx), (ny, nx)))
        stack.append((ny, nx))
    # render: inset free block per cell (2px wall ring)...
    for cy in range(n):
        for cx in range(n):
            g[cy * cell + 2:(cy + 1) * cell - 2,
              cx * cell + 2:(cx + 1) * cell - 2] = FREE
    # ...open exactly the tree edges (guaranteed connected)...
    for (cy, cx), (ny, nx) in edges:
        if ny == cy:  # horizontal neighbors
            cc = max(cx, nx) * cell
            r0 = cy * cell + 6
            g[r0:r0 + 12, cc - 2:cc + 2] = FREE
        else:  # vertical neighbors
            rr = max(cy, ny) * cell
            c0 = cx * cell + 6
            g[rr - 2:rr + 2, c0:c0 + 12] = FREE
    # ...plus a few seeded loop openings so detours exist.
    for _ in range(14):
        cy, cx = int(rng.randint(0, n - 1)), int(rng.randint(0, n - 1))
        if rng.random() < 0.5 and cx + 1 < n:
            r0 = cy * cell + 6
            g[r0:r0 + 12, (cx + 1) * cell - 2:(cx + 1) * cell + 2] = FREE
        elif cy + 1 < n:
            c0 = cx * cell + 6
            g[(cy + 1) * cell - 2:(cy + 1) * cell + 2, c0:c0 + 12] = FREE
    _border(g)
    return g


def gen_narrow(seed):
    rng = np.random.RandomState(seed)
    g = _blank()
    g[:, :] = FREE
    _border(g)
    # serpentine: 4 horizontal walls with alternating end gaps.
    rows = (70, 140, 210, 280)
    for k, ry in enumerate(rows):
        side = k % 2  # 0: gap at right end, 1: gap at left end
        gw = int(rng.uniform(24, 36))
        if side == 0:
            gaps = [(336 - gw, 336)]
        else:
            gaps = [(48, 48 + gw)]
        _hwall(g, ry, 40, 344, gaps, t=5)
    return g


GENERATORS = {"office": gen_office, "maze": gen_maze, "narrow": gen_narrow}


def flood_fill_fraction(g):
    """Fraction of free cells in the largest 4-connected component."""
    free = (g == FREE)
    if not free.any():
        return 0.0
    seen = np.zeros_like(free, bool)
    best = 0
    for _ in range(4):  # a few restarts to find the dominant component
        cand = np.argwhere(free & ~seen)
        if len(cand) == 0:
            break
        stack = [tuple(cand[0])]
        seen[tuple(cand[0])] = True
        comp = 0
        while stack:
            r, c = stack.pop()
            comp += 1
            if r > 0 and free[r - 1, c] and not seen[r - 1, c]:
                seen[r - 1, c] = True
                stack.append((r - 1, c))
            if r < SIZE - 1 and free[r + 1, c] and not seen[r + 1, c]:
                seen[r + 1, c] = True
                stack.append((r + 1, c))
            if c > 0 and free[r, c - 1] and not seen[r, c - 1]:
                seen[r, c - 1] = True
                stack.append((r, c - 1))
            if c < SIZE - 1 and free[r, c + 1] and not seen[r, c + 1]:
                seen[r, c + 1] = True
                stack.append((r, c + 1))
        best = max(best, comp)
    return float(best) / float(free.sum())


def validate_map(g):
    """Return dict of deterministic checks (no ROS)."""
    vals = set(np.unique(g).tolist())
    free_frac = float((g == FREE).mean())
    conn = flood_fill_fraction(g)
    return {
        "shape_ok": tuple(g.shape) == (SIZE, SIZE),
        "values_ok": vals <= {WALL, UNKNOWN, FREE},
        "free_frac": free_frac,
        "free_frac_ok": 0.15 < free_frac < 0.95,
        "connectivity": conn,
        "connectivity_ok": conn > 0.90,
    }


def sha256_bytes(b: bytes) -> str:
    import hashlib as _h
    return _h.sha256(b).hexdigest()


def write_map(family_variant, seed, out_dir):
    """Generate + write pgm/yaml/meta. Returns record dict with hashes."""
    from PIL import Image
    family = family_variant.split("-")[0]
    g = GENERATORS[family](seed)
    map_id = f"{family_variant}-s{seed}"
    os.makedirs(out_dir, exist_ok=True)
    pgm_path = os.path.join(out_dir, f"{map_id}.pgm")
    Image.fromarray(g).save(pgm_path)
    yaml_text = (f"image: {map_id}.pgm\n"
                 f"resolution: {RES:.6f}\n"
                 f"origin: [{ORIGIN[0]:.6f}, {ORIGIN[1]:.6f}, {ORIGIN[2]:.6f}]\n"
                 f"negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    yaml_path = os.path.join(out_dir, f"{map_id}.yaml")
    with open(yaml_path, "w") as f:
        f.write(yaml_text)
    val = validate_map(g)
    meta = {"map_id": map_id, "family": family,
            "variant": family_variant.split("-", 1)[1], "seed": seed,
            "size": SIZE, "res": RES, "origin": list(ORIGIN),
            "validation": val}
    meta_path = os.path.join(out_dir, f"{map_id}.meta.json")
    with open(meta_path, "w") as f:
        json.dump(meta, f, indent=1)
    with open(pgm_path, "rb") as f:
        pgm_hash = sha256_bytes(f.read())
    return {"map_id": map_id, "family": family, "seed": seed,
            "pgm": pgm_path, "yaml": yaml_path, "meta": meta_path,
            "sha256_pgm": pgm_hash,
            "sha256_yaml": sha256_bytes(yaml_text.encode()),
            "validation": val}
