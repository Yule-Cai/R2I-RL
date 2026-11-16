"""Run one baseline-arm attempt (recovery stage) on a high-fidelity HSSD scene.

Same preflight hash checks as the campaign launcher. The runner is the
baseline DEV variant scene_runner_baseline_v1.py (sha recorded per launch),
whose only differences from the frozen runner are gated by RULESYNC_BASELINE:
  * full     - unchanged
  * no_guard - RuleGuard monitor process not started
  * no_gate  - gate node started with rho=0.0 (no inflation check)
Concurrent attempts are allowed with --allow-concurrent.
"""
import argparse
import json
from pathlib import Path
import subprocess

import run_native_furnished_recovery_dev_v2 as base


IN_CONTAINER = """
import hashlib
from pathlib import Path
import runpy
import sys
from types import SimpleNamespace
import json

runner = Path('/experiment/scene_runner_baseline_v1.py')
assert hashlib.sha256(runner.read_bytes()).hexdigest() == sys.argv[1]
args = SimpleNamespace(**json.loads(sys.argv[2]))
for field in ('scene', 'output', 'case'):
    value = getattr(args, field)
    setattr(args, field, Path(value) if value else None)
result = runpy.run_path(str(runner))['run'](args)
raise SystemExit(0 if result['result'] == 'PASS' else 1)
"""


def main(case, attempt, speed, seed, ros_domain, allow_concurrent, tag, baseline, arm_env=''):
    root = Path(__file__).resolve().parent
    prepared = root / "nav_v1"
    migration = json.loads((prepared / "migration_manifest.json").read_text())
    projection = json.loads((prepared / "projection.json").read_text())
    scene = f"hssd_{migration['scene']}"
    assert base.sha(prepared / f"{scene}.sdf") == migration["world_sha256"]
    assert base.sha(prepared / "map.json") == projection["map_sha256"]
    assert projection["world_sha256"] == migration["source_capture_world_sha256"]
    assert all(base.sha(root / "assets" / name) == digest
               for name, digest in migration["asset_files_sha256"].items())
    runner_path = base.ROOT / "scene_runner_baseline_v1.py"
    runner_sha = base.sha(runner_path)
    assert subprocess.check_output(
        ["podman", "image", "inspect", base.IMAGE, "--format", "{{.Id}}"], text=True
    ).strip() == base.IMAGE_ID
    if not allow_concurrent:
        assert not subprocess.check_output(
            ["podman", "ps", "--filter", "name=rulesync", "--format", "{{.Names}}"], text=True
        ).strip()
    assert base.sha(prepared / "map.json") == json.loads(case.read_text())["map_sha256"]

    # frozen-cohort enforcement: refuse to start when inputs or sources drifted
    fm_path = base.ROOT / 'formal_cohort_manifest_v1.json'
    if fm_path.exists():
        fm = json.loads(fm_path.read_text())
        scene_entry = fm.get('scenes', {}).get(scene)
        assert scene_entry is not None, 'scene not in frozen cohort manifest'
        entry = next((x for x in scene_entry
                      if x['name'] == case.name and x['dir'] == str(case.parent.name)), None)
        assert entry is not None, 'case not in frozen cohort manifest'
        assert entry['sha256'] == base.sha(case), 'case sha256 drifted from frozen manifest'
        assert fm['scene_hashes'][scene]['map_sha256'] == projection['map_sha256'], 'map sha256 drifted from frozen manifest'
        runner_name = runner_path.name
        expected = fm.get('sources_sha256', {}).get(runner_name)
        assert expected and expected == runner_sha, 'runner sha256 drifted from frozen manifest'
    output = root / tag / baseline / f"{case.stem}_s{int(round(speed * 100)):02d}_seed{seed}"
    output.mkdir(parents=True, exist_ok=False)
    position = json.loads(case.read_text())["spawn"]
    inside = f"/experiment/{root.relative_to(base.ROOT)}"
    trial = output / "position_1"
    args = dict(scene=f"{inside}/{prepared.name}", output=f"{inside}/{output.relative_to(root)}/{trial.name}",
                case=f"{inside}/{case.relative_to(root)}",
                spawn=position, goal=position, seed=seed, stage="recovery", speed=speed,
                scene_id=scene, geometry_profile="standard", controller_profile="legacy",
                cohort_id=None, concurrency=1, render_mode="gpu", target_rtf=1., ros_domain=ros_domain)
    command = base.container(f"rulesync-base-{baseline}-{case.stem[:14]}-s{int(round(speed * 100)):02d}-{seed}-d{ros_domain}",
                             ["python3", "-c", IN_CONTAINER, runner_sha, json.dumps(args)], gpu=True)
    idx = command.index("--entrypoint")
    extra = ["--env", "GZ_IP=127.0.0.1", "--env", f"RULESYNC_BASELINE={baseline}"]
    if arm_env:
        extra += ["--env", arm_env]
    command[idx:idx] = ["-v", f"{root / 'assets'}:/assets/{scene}:ro"] + extra
    base.write_json(output / "position_1_launch.json", {
        "arguments": args, "image": base.IMAGE_ID, "world": migration["world_sha256"],
        "map": projection["map_sha256"], "runner": runner_sha,
        "case": base.sha(case), "speed": speed, "seed": seed,
    })
    with (output / "position_1_host.log").open("x") as log:
        process = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
    result = trial / "result.json"
    row = {"position": position, "exit_code": process.returncode,
           "result": json.loads(result.read_text()) if result.exists() else None}
    print(json.dumps(row), flush=True)
    base.write_json(output / "closed.json", {"results": [row]})


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", type=Path, required=True)
    parser.add_argument("--attempt", type=int, default=1)
    parser.add_argument("--speed", type=float, default=.20)
    parser.add_argument("--seed", type=int, default=52106)
    parser.add_argument("--ros-domain", type=int, default=220)
    parser.add_argument("--allow-concurrent", action="store_true")
    parser.add_argument("--tag", default="baseline_v1")
    parser.add_argument("--baseline", choices=["full", "geometry_only", "no_guard", "no_gate"], default="full")
    parser.add_argument("--arm-env", default="")
    options = parser.parse_args()
    assert options.attempt >= 1
    assert 0 <= options.ros_domain <= 232
    main(options.case.resolve(), options.attempt, options.speed, options.seed,
         options.ros_domain, options.allow_concurrent, options.tag, options.baseline,
         options.arm_env)
