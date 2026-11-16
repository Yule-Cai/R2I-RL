"""Launch one B1 quadrotor 3D DEV attempt on the handheld.

Same locking/hash conventions as ``run_hssd_campaign_attempt.py`` for the
frozen 2D runner:

* preflight sha256-pins the prepared scene world, the 5 cm collision map, every
  per-scene asset listed in ``migration_manifest.json``, the 3D case file, the
  DEV runner, the pure-Python ``quad3d`` package, the installed ``rulesync3d``
  runtime modules and the container image id;
* runs exactly one container with a unique name and a reserved ROS domain
  (230-232), ``--network none`` and no automatic retries;
* a concurrent attempt is only allowed with ``--allow-concurrent`` (the
  handheld also runs unrelated 2D qualification containers);
* every attempt gets a fresh output directory, the host log, the launch
  record with all input hashes and the unmodified ``result.json``.

Run from the project root on the handheld:
  python3 run_hssd_quad3d_attempt.py --scene-dir iotj_scene_build/scenes/<scene> \
      --case quad3d_cases/<case>.json --allow-concurrent
Platform qualification (minimal empty world, no scene):
  python3 run_hssd_quad3d_attempt.py --stage platform_selftest --allow-concurrent
"""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import time


ROOT = Path("/home/gamer/Projects/rulesync-iotj-environments-20261028")
IMAGE = "localhost/rulesync-channel-dev:20261104-trace1"
IMAGE_ID = "9130c308592c2ededa0483ff9d76275e4952711610ea6b22cc542f4820ebdc51"
RUNNER = "quad3d_snapshot_dev_20261114_v3/scene_runner_quad3d_v1.py"
RUNNER_SHA = "e1268863c6a6e5eb70dbc5960f3d56d09a80b0501e3a68398393bd1cb51f240d"
MODULE_SHA = {
    "quad3d_snapshot_dev_20261114_v3/quad3d/__init__.py": "ced3401bd6838b93c271a6a2b4dc2501e8e68e1a18053f317b91ab1366eec51c",
    "quad3d_snapshot_dev_20261114_v3/quad3d/authority3d.py": "707dd39da9afff405e30d4b3f3f0e53714fc1469c6958961be9e69c0bdd5c72d",
    "quad3d_snapshot_dev_20261114_v3/quad3d/dry_run_quad3d_v1.py": "9f41e84a59f444a8625e53c9fe14319612b83b4a6134724d93c5dc5c074e3584",
    "quad3d_snapshot_dev_20261114_v3/quad3d/make_quad3d_case_v1.py": "93c91a0311d44aad0e1e494ec9d7b3c1b872b6d1e10f47cfefd5688cc4cb5127",
    "quad3d_snapshot_dev_20261114_v3/quad3d/quad3d_case_schema_v1.json": "9db7b6ff0bb40c13bbdce887d8783ca6d0d7bdcdb898dedc8586c72d2e4b92db",
    "quad3d_snapshot_dev_20261114_v3/quad3d/scene3d.py": "4cd6fcc67343d3dd9596ac1c2bd270d8d3f68357a454336ff5d23b1dbffdec0d",
    "quad3d_snapshot_dev_20261114_v3/quad3d/selftest_quad3d_v1.py": "1e76afba07c67cce5aa8f648a5ba5cd638b3834be67c31692f0e6fd977e1a0fc",
    "quad3d_snapshot_dev_20261114_v3/quad3d/snapshot_history.py": "9d1b594db7f601ef74ad1927885c74ec627a2a6b39e5f454b2938a8bbe1cd866"
}
RULESYNC3D_SHA = {
    "authorization.py":
        "8c6ca852b65b35ec6513c56d6192d0b1fc4b63abc112ad91629e24fb4c491be6",
    "gate_3d.py":
        "abdf76aab0ef5ea1b1b7625f6730df0f7832eaab23799f060933b17c067f3fab",
    "planners_3d.py":
        "8ce2d12b27ba00db6ccbb7ae1b04766c6455486803ddee9b8bd44ac7de27b93b",
    "ruleguard.py":
        "8fd86f7b1fa3887bc12928475d4aab56c3b79fdadddf8e05750d09484e4ee0ff",
}
RESERVED_DOMAINS = (230, 231, 232)

IN_CONTAINER = r"""
import hashlib
import json
from pathlib import Path
import runpy
import sys
from types import SimpleNamespace

sys.path.insert(0, '/experiment/quad3d_snapshot_dev_20261114_v3')


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.sha256(stream.read()).hexdigest()


runner = Path('/experiment/quad3d_snapshot_dev_20261114_v3/scene_runner_quad3d_v1.py')
assert sha(runner) == sys.argv[1], 'runner sha256 mismatch'
for relative, digest in json.loads(sys.argv[2]).items():
    assert sha('/experiment/' + relative) == digest, relative
import rulesync3d.authorization as authorization
import rulesync3d.gate_3d as gate_3d
import rulesync3d.planners_3d as planners_3d
import rulesync3d.ruleguard as ruleguard
installed = {
    'authorization.py': authorization.__file__,
    'gate_3d.py': gate_3d.__file__,
    'planners_3d.py': planners_3d.__file__,
    'ruleguard.py': ruleguard.__file__,
}
for name, digest in json.loads(sys.argv[3]).items():
    assert sha(installed[name]) == digest, 'installed rulesync3d ' + name
args = SimpleNamespace(**json.loads(sys.argv[4]))
for field in ('scene', 'output', 'case'):
    value = getattr(args, field)
    setattr(args, field, Path(value) if value else None)
result = runpy.run_path(str(runner))['run'](args)
raise SystemExit(0 if result['result'] == 'PASS' else 1)
"""


def sha(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def write_json(path, value):
    tmp = Path(path).with_suffix(Path(path).suffix + ".tmp")
    tmp.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")
    tmp.replace(path)


def container(name, arguments, gpu=False, extra_mounts=(), extra_env=()):
    command = [
        "podman", "run", "--rm", "--name", name, "--network", "none",
        "--cpus", "3", "--memory", "3g", "--memory-swap", "3g",
        "--pids-limit", "512", "--timeout", "1200",
        "-v", "/home/gamer/Projects/rulesync-review-20261010:/work:ro",
        "-v", "/home/gamer/Projects/rulesync-submission-20261012:"
              "/reference:ro",
        "-v", f"{ROOT}:/experiment", "-v", f"{ROOT / 'assets'}:/assets:ro",
    ]
    if gpu:
        command += ["--device", "/dev/dri/renderD128"]
    for mount in extra_mounts:
        command += ["-v", mount]
    for value in extra_env:
        command += ["--env", value]
    shell = (
        "source /opt/ros/lyrical/setup.bash && source /work/install/setup.bash"
        " && export PYTHONPATH=/experiment/source:/reference:/work/scripts:"
        "$PYTHONPATH LTTNG_HOME=/tmp/rulesync_lttng_home && "
    )
    return command + ["--entrypoint", "/bin/bash", IMAGE, "-lc",
                      shell + shlex.join(arguments)]


def running_containers():
    try:
        output = subprocess.check_output(
            ["podman", "ps", "--format", "{{.Names}}"], text=True)
    except subprocess.CalledProcessError:
        return []
    return [line.strip() for line in output.splitlines() if line.strip()]


def preflight_scene(scene_dir, case_path):
    scene_id = scene_dir.name
    assert scene_dir.is_dir(), f"scene dir missing: {scene_dir}"
    scene_sdf = scene_dir / f"{scene_id}.sdf"
    migration = json.loads((scene_dir / "migration_manifest.json").read_text())
    assert sha(scene_sdf) == migration["world_sha256"], "scene world sha256"
    map_path = scene_dir / "nav_v1/map.json"
    routes = json.loads((scene_dir / "routes_v1.json").read_text())
    assert sha(map_path) == routes["map_sha256"], "nav_v1 map sha256"
    asset_hashes = {}
    for relative, digest in migration["asset_files_sha256"].items():
        path = scene_dir / "assets" / relative
        assert path.is_file(), f"missing asset {relative}"
        assert sha(path) == digest, f"asset sha256 {relative}"
        asset_hashes[relative] = digest
    case = json.loads(case_path.read_text())
    assert case["scene"] == scene_id, "case scene id"
    assert case["map_sha256"] == routes["map_sha256"], "case map sha256"
    source_case_path = (scene_dir / "recovery_inputs_v1"
                        / case["source_case"]["file"])
    assert source_case_path.is_file(), "source 2D case missing"
    assert sha(source_case_path) == case["source_case"]["sha256"], \
        "source 2D case sha256"
    return {
        "scene_id": scene_id,
        "scene_sdf_sha256": sha(scene_sdf),
        "map_sha256": sha(map_path),
        "migration_manifest_sha256": sha(
            scene_dir / "migration_manifest.json"),
        "routes_v1_sha256": sha(scene_dir / "routes_v1.json"),
        "asset_files": len(asset_hashes),
        "asset_files_sha256": asset_hashes,
        "case_sha256": sha(case_path),
        "source_2d_case_sha256": sha(source_case_path),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=["smoke", "platform_selftest"],
                        default="smoke")
    parser.add_argument("--scene-dir", type=Path)
    parser.add_argument("--case", type=Path)
    parser.add_argument("--seed", type=int, default=53001)
    parser.add_argument("--speed", type=float, default=0.20)
    parser.add_argument("--attempt", type=int, default=1)
    parser.add_argument("--ros-domain", type=int, default=231)
    parser.add_argument("--render-mode", choices=["gpu", "software", "none"],
                        default="gpu")
    parser.add_argument("--tag", default="quad3d_dev_v1")
    parser.add_argument("--allow-concurrent", action="store_true")
    parser.add_argument("--voxel-npz", type=Path)
    parser.add_argument("--occupancy", choices=["projected2d", "voxel"],
                        default="projected2d")
    parser.add_argument("--source-pin-sha", required=True)
    options = parser.parse_args()
    pin_path = Path(__file__).resolve().parent / "source_pin.json"
    assert sha(pin_path) == options.source_pin_sha, "source pin manifest mismatch"
    for name, digest in json.loads(pin_path.read_text())["files"].items():
        assert sha(pin_path.parent / name) == digest, "source pin mismatch: " + name
    assert options.tag.startswith("quad3d_snapshot_"), "new DEV tag required"
    assert not options.allow_concurrent, "snapshot qualification is sequential"
    names = running_containers()
    assert not any(n.startswith("rulesync-") for n in names), "RuleSync container already active"
    mem = {line.split(":")[0]: int(line.split()[1]) for line in Path("/proc/meminfo").read_text().splitlines()}
    assert mem["MemAvailable"] >= 7 * 1024 * 1024, "less than 7 GiB available"
    if options.case:
        case_speed = json.loads(options.case.read_text())["flight"]["speed_xy_mps"]
        assert case_speed == options.speed, "case speed differs from trial speed"
    assert Path(__file__).resolve().parent == ROOT / "quad3d_snapshot_dev_20261114_v3", \
        f"run this launcher from the handheld project root {ROOT}"
    assert options.attempt >= 1
    assert options.ros_domain in RESERVED_DOMAINS, \
        f"DEV attempts reserve ROS domains {RESERVED_DOMAINS}"
    assert options.speed in (0.10, 0.20, 0.25)
    assert 0 <= options.seed < 2 ** 31
    preflight = {
        "runner_sha256": sha(ROOT / RUNNER),
        "runner_expected_sha256": RUNNER_SHA,
        "module_sha256": {name: sha(ROOT / name) for name in MODULE_SHA},
        "module_expected_sha256": MODULE_SHA,
        "image_id": subprocess.check_output(
            ["podman", "image", "inspect", IMAGE, "--format", "{{.Id}}"],
            text=True).strip(),
        "image_expected_id": IMAGE_ID,
        "running_containers": running_containers(),
        "concurrency_allowed": bool(options.allow_concurrent),
    }
    assert preflight["runner_sha256"] == RUNNER_SHA, "runner sha256 mismatch"
    for name, digest in MODULE_SHA.items():
        assert preflight["module_sha256"][name] == digest, \
            f"module sha256 mismatch: {name}"
    assert preflight["image_id"] == IMAGE_ID, "container image id mismatch"
    if options.stage == "smoke":
        assert options.scene_dir is not None and options.case is not None, \
            "--scene-dir and --case are required for the smoke stage"
        scene_dir = options.scene_dir.resolve()
        case_path = options.case.resolve()
        scene = preflight_scene(scene_dir, case_path)
        preflight["scene"] = scene
    else:
        scene_dir = None
        case_path = None
        scene = None

    if options.stage == "smoke":
        output = (scene_dir / "quad3d_runs" / options.tag
                  / f"{case_path.stem}_s{int(round(options.speed * 100)):02d}"
                    f"_seed{options.seed}_{options.occupancy}_a{options.attempt}")
        name = (f"rulesync-quad3d-{scene['scene_id'].replace('hssd_', '')}"
                f"-{case_path.stem.split('_quad3d_')[0].rsplit('_', 1)[-1]}"
                f"-s{int(round(options.speed * 100)):02d}a{options.attempt}"
                f"-seed{options.seed}"
                f"-{options.tag.replace('quad3d_', '')}")
        inside_scene = f"/experiment/{scene_dir.relative_to(ROOT)}"
        inside_case = f"/experiment/{case_path.relative_to(ROOT)}"
        inside_output = f"/experiment/{output.relative_to(ROOT)}"
        extra_mounts = [f"{scene_dir / 'assets'}:/assets/"
                        f"{scene['scene_id']}:ro"]
        extra_env = ["GZ_IP=127.0.0.1"]
        voxel_inside = None
        voxel_inside_host = None
        if options.occupancy == "voxel":
            assert options.voxel_npz is not None, "--voxel-npz required for voxel occupancy"
            voxel = options.voxel_npz.resolve()
            assert voxel.is_file(), f"voxel npz missing: {voxel}"
            assert str(voxel).startswith(str(ROOT)), "voxel npz must live under ROOT"
            assert scene["scene_id"] in voxel.name, "voxel npz scene mismatch"
            voxel_inside = f"/experiment/{voxel.relative_to(ROOT)}"
            voxel_inside_host = voxel
            extra_mounts.append(f"{voxel}:{voxel_inside}:ro")
            extra_env.append(f"RULESYNC_QUAD_VOXEL_NPZ={voxel_inside}")
        arguments = ["python3", "-c", IN_CONTAINER, RUNNER_SHA,
                     json.dumps(MODULE_SHA), json.dumps(RULESYNC3D_SHA),
                     json.dumps({
                         "scene": inside_scene,
                         "scene_id": scene["scene_id"],
                         "case": inside_case,
                         "output": inside_output,
                         "seed": options.seed,
                         "speed": options.speed,
                         "ros_domain": options.ros_domain,
                         "render_mode": options.render_mode,
                         "target_rtf": 1.0,
                         "stage": "smoke",
                         "occupancy": options.occupancy,
                         "voxel_npz": voxel_inside,
                         "voxel_sha256": sha(voxel_inside_host) if voxel_inside else None,
                     })]
    else:
        output = (ROOT / "quad3d_runs" / options.tag
                  / f"platform_selftest_s{int(round(options.speed * 100)):02d}"
                    f"_seed{options.seed}_a{options.attempt}")
        name = (f"rulesync-quad3d-platform-s"
                f"{int(round(options.speed * 100)):02d}a{options.attempt}"
                f"-seed{options.seed}")
        inside_output = f"/experiment/{output.relative_to(ROOT)}"
        arguments = ["python3", "-c", IN_CONTAINER, RUNNER_SHA,
                     json.dumps(MODULE_SHA), json.dumps(RULESYNC3D_SHA),
                     json.dumps({
                         "scene": None,
                         "scene_id": None,
                         "case": None,
                         "output": inside_output,
                         "seed": options.seed,
                         "speed": options.speed,
                         "ros_domain": options.ros_domain,
                         "render_mode": options.render_mode,
                         "target_rtf": 1.0,
                         "stage": "platform_selftest",
                     })]
        extra_mounts = []
        extra_env = ["GZ_IP=127.0.0.1"]
        voxel_inside = None

    names = preflight["running_containers"]
    assert name not in names, f"container {name} is already running"
    if not options.allow_concurrent:
        assert not [item for item in names
                    if item.startswith("rulesync-quad3d-")], \
            "a quad3d attempt is already running; pass --allow-concurrent"
    output.mkdir(parents=True, exist_ok=False)
    command = container(name, arguments, gpu=options.render_mode == "gpu",
                        extra_mounts=extra_mounts,
                        extra_env=extra_env)
    launch = {
        "schema": "rulesync.quad3d.launch.v1",
        "stage": options.stage,
        "arguments": json.loads(arguments[-1]),
        "container_name": name,
        "image": IMAGE,
        "image_id": IMAGE_ID,
        "ros_domain": options.ros_domain,
        "speed_mps": options.speed,
        "seed": options.seed,
        "attempt": options.attempt,
        "render_mode": options.render_mode,
        "preflight": preflight,
        "container_command": command,
        "output": str(output),
        "launched_wall_time": time.strftime("%Y-%m-%dT%H:%M:%S",
                                            time.localtime()),
        "automatic_retries": False,
        "formal_denominator": False,
    }
    write_json(output / "launch.json", launch)
    started = time.monotonic()
    with (output / "launch_host.log").open("w") as log:
        process = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
    result_path = output / "result.json"
    row = {
        "stage": options.stage,
        "container": name,
        "exit_code": process.returncode,
        "wall_s": time.monotonic() - started,
        "result": (json.loads(result_path.read_text())
                   if result_path.is_file() else None),
    }
    write_json(output / "closed.json", {"schema": launch["schema"],
                                        "row": row})
    unchanged = (sha(ROOT / RUNNER) == RUNNER_SHA
                 and all(sha(ROOT / name) == digest
                         for name, digest in MODULE_SHA.items()))
    assert unchanged, "source files changed during the attempt"
    print(json.dumps({"output": str(output),
                      "exit_code": row["exit_code"],
                      "result": (row["result"] or {}).get("result"),
                      "phase": (row["result"] or {}).get("phase"),
                      "case_id": (row["result"] or {}).get("case_id"),
                      "error": (row["result"] or {}).get("error")},
                     indent=2), flush=True)
    raise SystemExit(0 if row["exit_code"] == 0 else 1)


if __name__ == "__main__":
    main()
