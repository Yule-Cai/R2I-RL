"""3D world builder: RLE-free SDF worlds from box lists.

Each world: bounds, floor, static box obstacles, spawn pose. Boxes are
(axis-aligned) wall segments; the UAV must change altitude in at least
some successful trajectories (verified per world in tests).
"""
import os

HEADER = """<?xml version="1.0" ?>
<sdf version="1.9">
<world name="{name}">
<physics name="1ms" type="dart"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
<scene><sky><clouds><speed>12</speed></clouds></sky></scene>
<light type="directional" name="sun"><cast_shadows>false</cast_shadows></light>
<ground_plane/>
"""

WALL = """<model name="{name}"><static>true</static><link name="link"><collision name="c"><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry></collision><visual name="v"><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry><material><ambient>0.6 0.6 0.6 1</ambient></material></visual><pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose></link></model>
"""  # noqa: E501

FOOTER = """</world>
</sdf>
"""


def box(cx, cy, cz, sx, sy, sz):
    return (cx, cy, cz, sx, sy, sz)


def write_world(path, name, boxes):
    parts = [HEADER.format(name=name)]
    for i, (cx, cy, cz, sx, sy, sz) in enumerate(boxes):
        parts.append(WALL.format(name=f"wall{i}", x=cx, y=cy, z=cz,
                                 sx=sx, sy=sy, sz=sz))
    parts.append(FOOTER)
    with open(path, "w") as f:
        f.write("".join(parts))
    print(f"wrote {path} ({len(boxes)} boxes)")


# World frames: x,y in [0,20], z in [0,6]. Spawn (1,10,1).
def world_open_3d():
    # empty room: floor + 4 boundary walls (low, fly-over allowed)
    return [
        box(10, 0, 0.5, 20, 0.4, 1.0),
        box(10, 20, 0.5, 20, 0.4, 1.0),
        box(0, 10, 0.5, 0.4, 20, 1.0),
        box(20, 10, 0.5, 0.4, 20, 1.0),
    ]


def world_floating_barrier():
    # partial-width slab x 6..14, z 0..3, full y: the straight low
    # corridor is blocked with no around (bounds), forcing a climb
    # over the top (z > 3) and back down. Genuine forced dz.
    return world_open_3d() + [
        box(10, 10, 1.5, 8, 20, 3.0),
    ]


def world_vertical_corridor():
    # two tall slabs leave a vertical shaft at x 9..11: altitude
    # changes forced through the shaft
    return world_open_3d() + [
        box(5, 10, 3.0, 8, 20, 6.0),
        box(15, 10, 3.0, 8, 20, 6.0),
    ]


def world_multi_level():
    # two stacked slabs with offset gaps: low route (z<2) west side,
    # high route (z>4) east side; forces an altitude switch mid-route
    return world_open_3d() + [
        box(5, 10, 1.0, 8, 20, 2.0),
        box(15, 10, 5.0, 8, 20, 2.0),
    ]


WORLDS = {
    "open_3d": world_open_3d,
    "floating_barrier": world_floating_barrier,
    "vertical_corridor": world_vertical_corridor,
    "multi_level": world_multi_level,
}

BOUNDS = (0.0, 0.0, 0.2, 20.0, 20.0, 6.0)
# Per-world spawns: collision-free AND mission-covering (joints jx/jy
# ±15, jz ±4; spawn±span must cover starts/goals/detours with margin).
SPAWNS = {
    "open_3d": (10.0, 10.0, 3.0),
    "floating_barrier": (5.0, 10.0, 4.0),
    "vertical_corridor": (10.0, 10.0, 3.0),
    "multi_level": (10.0, 10.0, 3.0),
}


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", required=True)
    a = ap.parse_args()
    os.makedirs(a.out_dir, exist_ok=True)
    for name, fn in WORLDS.items():
        write_world(os.path.join(a.out_dir, f"{name}.sdf"), name,
                    fn())


if __name__ == "__main__":
    main()
