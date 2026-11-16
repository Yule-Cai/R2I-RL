"""Run matched clean and interrupted snapshot delivery on the two native worlds."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import shlex
import shutil
import subprocess
import time


ROOT = Path("/home/gamer/Projects/rulesync-iotj-environments-20261028")
SOURCE = ROOT / "channel_dev_20261104/source"
OUTPUT = ROOT / "native_furnished_recovery_dev_v2"
IMAGE = "localhost/rulesync-channel-dev:20261104-trace1"
IMAGE_ID = "9130c308592c2ededa0483ff9d76275e4952711610ea6b22cc542f4820ebdc51"
RUNNER_SHA = "a6930ebc0f4b990e9d21cdc7e1da95d17646797e38765b8eaf255076b984a3a1"
CASES = {
    "house": "house_v4/house_route_03_replace.json",
    "warehouse": "warehouse_aligned_v1/warehouse_route_01_replace.json",
}
SOURCES = (
    "nav2_trace_dev_v1.py",
    "command_witness_v1.py",
    "channel_trace_probe_v1.py",
    "scene_runner_channel_dev_v1.py",
    "channel_window_schedule_v1.py",
    "snapshot_response_proxy_v1.py",
    "snapshot_response_proxy_v2.py",
    "channel_guard_observer_v1.py",
    "decode_snapshot_proxy_cdr_v1.py",
)
SEED = 52061


def sha(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def write_json(path, value):
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(value, indent=2) + "\n")
    tmp.replace(path)


def container(name, arguments, schedule=None, gpu=False):
    command = [
        "podman", "run", "--rm", "--name", name, "--network", "none",
        "--cpus", "3", "--memory", "3g", "--memory-swap", "3g",
        "--pids-limit", "512", "--timeout", "1200",
        "-v", "/home/gamer/Projects/rulesync-review-20261010:/work:ro",
        "-v", "/home/gamer/Projects/rulesync-submission-20261012:/reference:ro",
        "-v", f"{ROOT}:/experiment", "-v", f"{ROOT / 'assets'}:/assets:ro",
    ]
    shell = (
        "source /opt/ros/lyrical/setup.bash && source /work/install/setup.bash && "
        "export PYTHONPATH=/experiment/source:/reference:/work/scripts:$PYTHONPATH "
        "LTTNG_HOME=/tmp/rulesync_lttng_home && "
    )
    if gpu:
        command += ["--device", "/dev/dri/renderD128"]
    if schedule:
        shell += f"export RULESYNC_CHANNEL_SCHEDULE={shlex.quote(schedule)} && "
    return command + ["--entrypoint", "/bin/bash", IMAGE, "-lc", shell + shlex.join(arguments)]


def run_trial(job):
    scene, condition, domain = job
    trial = f"{scene}_{condition}"
    inside = f"/experiment/{OUTPUT.name}"
    run = f"{inside}/{trial}"
    source = f"{inside}/source"
    schedule = f"{source}/{condition}.json"
    start = time.monotonic()
    with (OUTPUT / f"{trial}_host.log").open("x") as log:
        status = subprocess.run(container(
            f"rulesync-native-recovery-{trial.replace('_', '-')}-v2",
            [
                "python3", f"{source}/nav2_trace_dev_v1.py",
                "--command-witness", "--output", run, "--",
                "/experiment/channel_dev_20261104/source/scene_runner_channel_dev_v1.py",
                "--scene-id", scene, "--scene", f"/experiment/camera_prepared_v3/{scene}",
                "--case", f"/experiment/tasks/{CASES[scene]}",
                "--speed", ".20", "--seed", str(SEED), "--ros-domain", str(domain),
                "--concurrency", "2", "--output", f"{run}/mission",
                "--stage", "recovery", "--render-mode", "gpu",
                "--controller-profile", "legacy", "--geometry-profile", "standard",
            ],
            schedule=schedule, gpu=True,
        ), stdout=log, stderr=subprocess.STDOUT)
    mission_result = OUTPUT / trial / "mission/result.json"
    return {
        "id": trial, "scene": scene, "condition": condition, "domain": domain,
        "exit_code": status.returncode, "wall_s": time.monotonic() - start,
        "mission": json.loads(mission_result.read_text()) if mission_result.is_file() else None,
    }


def main():
    assert sha(ROOT / "scene_runner_generalization_v1.py") == RUNNER_SHA
    assert subprocess.check_output(
        ["podman", "image", "inspect", IMAGE, "--format", "{{.Id}}"], text=True
    ).strip() == IMAGE_ID
    assert not subprocess.check_output(
        ["podman", "ps", "--filter", "name=rulesync", "--format", "{{.Names}}"], text=True
    ).strip()
    inputs = [ROOT / "scene_runner_generalization_v1.py"]
    for scene, case in CASES.items():
        scene_input = ROOT / "camera_prepared_v3" / scene
        inputs += [scene_input / name for name in (f"{scene}.sdf", "map.json", "camera_reframe.json", "migration_manifest.json")]
        case_path = ROOT / "tasks" / case
        assert json.loads(case_path.read_text())["map_sha256"] == sha(scene_input / "map.json")
        inputs.append(case_path)
    input_hashes = {str(path.relative_to(ROOT)): sha(path) for path in inputs}
    OUTPUT.mkdir()
    source_copy = OUTPUT / "source"
    source_copy.mkdir()
    shutil.copy2(__file__, source_copy)
    for name in SOURCES:
        shutil.copy2(SOURCE / name, source_copy / name)
    for condition in ("clean", "blackout"):
        schedule = json.loads((SOURCE / f"channel_dev_{condition}_v1.json").read_text())
        schedule["seed"] = SEED
        write_json(source_copy / f"{condition}.json", schedule)
    source_hashes = {path.name: sha(path) for path in source_copy.iterdir()}
    batches = [
        [("house", "clean", 219), ("warehouse", "blackout", 220)],
        [("house", "blackout", 221), ("warehouse", "clean", 222)],
    ]
    write_json(OUTPUT / "manifest.json", {
        "scope": "matched full-policy closed-loop recovery DEV in native Paper B worlds",
        "planned": 4, "seed": SEED, "speed_mps": .20, "batches": batches,
        "cases": CASES, "image_id": IMAGE_ID, "source_sha256": source_hashes,
        "input_sha256": input_hashes, "blackout_window_s": [0, 2.5],
        "workers": 2, "per_trial_cpus": 3, "per_trial_memory_gib": 3,
        "automatic_retries": False, "formal_denominator": False,
        "revision": "Use the trace wrapper's registered adapter path after v1 pre-mission rejection.",
    })
    results = []
    with ThreadPoolExecutor(max_workers=2) as pool:
        for batch in batches:
            for result in pool.map(run_trial, batch):
                results.append(result)
                write_json(OUTPUT / "progress.json", {"closed": len(results), "planned": 4, "results": results})
                print(result["id"], result["exit_code"], flush=True)
    for result in results:
        trial = result["id"]
        inside = f"/experiment/{OUTPUT.name}"
        run = f"{inside}/{trial}"
        if (OUTPUT / trial / "mission/snapshot_channel/proxy.jsonl").is_file():
            with (OUTPUT / f"{trial}_decode.log").open("x") as log:
                result["decode_exit_code"] = subprocess.run(container(
                    f"rulesync-native-recovery-decode-{trial.replace('_', '-')}-v2",
                    [
                        "python3", f"{inside}/source/decode_snapshot_proxy_cdr_v1.py",
                        "--root", f"{run}/mission",
                        "--log", "snapshot_channel/proxy.jsonl", "--log", "guard_events.jsonl",
                        "--output", f"{run}/service_cdr_decoded.json",
                    ],
                ), stdout=log, stderr=subprocess.STDOUT).returncode
    assert all(sha(ROOT / path) == digest for path, digest in input_hashes.items())
    assert all(sha(source_copy / name) == digest for name, digest in source_hashes.items())
    assert all(sha(SOURCE / name) == source_hashes[name] for name in SOURCES)
    write_json(OUTPUT / "closed.json", {"closed": len(results), "planned": 4, "results": results})


if __name__ == "__main__":
    main()
