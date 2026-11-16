"""Isolated 3D DEV launcher with explicit occupancy/source pin.

Run only after the active formal queue is complete. No concurrent RuleSync
container or <7 GiB memory is admitted. Source mismatch never disables pinning.
Use --source-pin-sha from the reviewed source_pin_v2.json and, for smoke,
--occupancy-mode voxel --voxel-manifest <bound occupancy_manifest.json>.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import sys
import time


ROOT = Path("/home/gamer/Projects/rulesync-iotj-environments-20261028")
IMAGE = "localhost/rulesync-channel-dev:20261104-trace1"
IMAGE_ID = "9130c308592c2ededa0483ff9d76275e4952711610ea6b22cc542f4820ebdc51"
RUNNER = "scene_runner_quad3d_v1.py"
BASE = Path(__file__).resolve().parent
SOURCE_PIN = json.loads((BASE / 'source_pin_v2.json').read_text())
RUNNER_SHA = SOURCE_PIN['files'][RUNNER]
MODULE_SHA = {k:v for k,v in SOURCE_PIN['files'].items() if k.startswith('quad3d/')}


def verify_source_pin(expected):
    from quad3d.voxel_contract import sha as digest, member
    if digest(BASE / 'source_pin_v2.json') != expected:
        raise ValueError('source pin hash mismatch')
    for relative, value in SOURCE_PIN['files'].items():
        if digest(member(BASE, relative)) != value:
            raise ValueError('DEV source hash mismatch: ' + relative)
    return True


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

sys.path.insert(0, '/experiment/quad3d_voxel_dev_20261108_v2')


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.sha256(stream.read()).hexdigest()


runner = Path('/experiment/quad3d_voxel_dev_20261108_v2/scene_runner_quad3d_v1.py')
assert sha(runner) == sys.argv[1], 'runner sha256 mismatch'
for relative, digest in json.loads(sys.argv[2]).items():
    assert sha('/experiment/quad3d_voxel_dev_20261108_v2/' + relative) == digest, relative
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
        " && export PYTHONPATH=/experiment/quad3d_voxel_dev_20261108_v2:"
        "/experiment/pylibs:/reference:/work/scripts:"
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
    parser.add_argument("--tag", default="quad3d_voxel_dev_v2")
    parser.add_argument('--source-pin-sha', required=True)
    parser.add_argument('--occupancy-mode', choices=['voxel', 'projected_2d'])
    parser.add_argument('--voxel-manifest', type=Path)
    options = parser.parse_args()
    assert BASE == ROOT / "quad3d_voxel_dev_20261108_v2", \
        f"run this launcher from the handheld project root {ROOT}"
    assert options.attempt >= 1
    assert options.ros_domain in RESERVED_DOMAINS, \
        f"DEV attempts reserve ROS domains {RESERVED_DOMAINS}"
    assert options.speed in (0.10, 0.20, 0.25)
    assert 0 <= options.seed < 2 ** 31
    verify_source_pin(options.source_pin_sha)
    names = running_containers()
    if any(name.startswith('rulesync-') for name in names):
        raise RuntimeError('DEV Gazebo deferred: another RuleSync container is active')
    memory = dict(line.split(':', 1) for line in Path('/proc/meminfo').read_text().splitlines())
    if int(memory['MemAvailable'].split()[0]) < 7 * 1024 * 1024:
        raise RuntimeError('DEV Gazebo deferred: requires 7 GiB available memory')
    preflight = {
        "runner_sha256": sha(BASE / RUNNER),
        "runner_expected_sha256": RUNNER_SHA,
        "module_sha256": {name: sha(BASE / name) for name in MODULE_SHA},
        "module_expected_sha256": MODULE_SHA,
        "image_id": subprocess.check_output(
            ["podman", "image", "inspect", IMAGE, "--format", "{{.Id}}"],
            text=True).strip(),
        "image_expected_id": IMAGE_ID,
        "running_containers": running_containers(),
        "concurrency_allowed": False,
        "source_pin_sha256": options.source_pin_sha,
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
        if options.occupancy_mode not in ('voxel', 'projected_2d'):
            raise ValueError('smoke requires explicit --occupancy-mode')
        if options.occupancy_mode == 'voxel':
            if options.voxel_manifest is None:
                raise ValueError('voxel mode requires --voxel-manifest')
            from quad3d.voxel_contract import validate_manifest
            _, binding = validate_manifest(options.voxel_manifest, scene_dir, ROOT)
            preflight['occupancy_binding'] = binding
            inside_manifest = '/experiment/' + str(options.voxel_manifest.resolve().relative_to(ROOT))
        elif options.voxel_manifest is not None:
            raise ValueError('projected_2d forbids --voxel-manifest')
    else:
        scene_dir = None
        case_path = None
        scene = None

    if options.stage == "smoke":
        output = (scene_dir / "quad3d_runs" / options.tag
                  / f"{case_path.stem}_s{int(round(options.speed * 100)):02d}"
                    f"_seed{options.seed}_a{options.attempt}")
        name = (f"rulesync-quad3d-{scene['scene_id'].replace('hssd_', '')}"
                f"-{case_path.stem.split('_quad3d_')[0].rsplit('_', 1)[-1]}"
                f"-s{int(round(options.speed * 100)):02d}a{options.attempt}"
                f"-{options.tag.replace('quad3d_', '')}")
        inside_scene = f"/experiment/{scene_dir.relative_to(ROOT)}"
        inside_case = f"/experiment/{case_path.relative_to(ROOT)}"
        inside_output = f"/experiment/{output.relative_to(ROOT)}"
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
                     })]
        extra_mounts = [f"{scene_dir / 'assets'}:/assets/"
                        f"{scene['scene_id']}:ro"]
    else:
        output = (ROOT / "quad3d_runs" / options.tag
                  / f"platform_selftest_s{int(round(options.speed * 100)):02d}"
                    f"_seed{options.seed}_a{options.attempt}")
        name = (f"rulesync-quad3d-platform-s"
                f"{int(round(options.speed * 100)):02d}a{options.attempt}")
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

    names = preflight["running_containers"]
    assert name not in names, f"container {name} is already running"
    output.mkdir(parents=True, exist_ok=False)
    env = ['GZ_IP=127.0.0.1']
    if options.stage == 'smoke':
        env += ['RULESYNC_QUAD_OCCUPANCY_MODE=' + options.occupancy_mode]
        if options.occupancy_mode == 'voxel':
            env += ['RULESYNC_QUAD_VOXEL_MANIFEST=' + inside_manifest]
    command = container(name, arguments, gpu=options.render_mode == "gpu",
                        extra_mounts=extra_mounts,
                        extra_env=env)
    launch = {
        "schema": "rulesync.quad3d.launch.v2",
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
    unchanged = (sha(BASE / RUNNER) == RUNNER_SHA
                 and all(sha(BASE / name) == digest
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
