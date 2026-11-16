"""Run one DEV mechanism-pair attempt (external unannounced update / stale candidate).

Runs the DEV runner ``authorization_diagnostic_20261115/scene_runner_diagnostic.py`` against a prepared v2 scene and
a formal-cohort case, but outside the formal queue: the cohort binding is DEV-only
and recorded in the launch/result.  Two monitor arms (full / geometry_only) and two
mechanism modes are supported; results land under mechpairs_runs/<tag>/.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

import sys
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
import run_native_furnished_recovery_dev_v2 as base

MECHP_RUNNER_SHA = "0fa15fc4a7b6d94b45e6079fd78bf75dadd5fe4ccf886b3a0ce1cb037b14dcb1"
MANIFEST = "formal_dualmap_20261108_v2/manifest.json"

IN_CONTAINER = """
import hashlib, json, runpy, sys
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0, '/experiment/authorization_diagnostic_20261115')
sys.path.insert(1, '/experiment')
runner = Path('/experiment/authorization_diagnostic_20261115/scene_runner_diagnostic.py')
assert hashlib.sha256(runner.read_bytes()).hexdigest() == sys.argv[1]
args = SimpleNamespace(**json.loads(sys.argv[2]))
for field in ('scene', 'output', 'case'):
    value = getattr(args, field)
    setattr(args, field, Path(value) if value else None)
result = runpy.run_path(str(runner))['run'](args)
raise SystemExit(0 if result['result'] == 'PASS' else 1)
"""


def main(opt):
    root = base.ROOT
    pin = Path(__file__).resolve().parent/'source_pin.json'
    assert base.sha(pin)==opt.source_pin_sha, 'probe pin hash mismatch'
    for name,digest in json.loads(pin.read_text())['files'].items():
        assert base.sha(pin.parent/name)==digest, name
    names=subprocess.check_output(['podman','ps','--format','{{.Names}}'],text=True).splitlines()
    assert not any(n.startswith('rulesync-') for n in names), 'another RuleSync run is active'
    assert opt.tag.startswith('authprobe_diagnostic_20261115_'), 'new DEV tag required'
    assert opt.mech_mode=='out_of_order' and opt.monitor_arm=='full', 'qualification is full arm only'
    assert base.sha(root/MANIFEST)=='bb749f5d886ef5a0009966ce4d180b3c58e2c1d503213a918cedd94534dfef20'
    mem={ln.split(':')[0]:int(ln.split()[1]) for ln in Path('/proc/meminfo').read_text().splitlines()}
    assert mem['MemAvailable']>=7*1024*1024, 'less than 7GiB available' 
    manifest_path = root / MANIFEST
    m = json.loads(manifest_path.read_text())
    scene = m['scenes'][opt.scene]['prepared']
    asset_root = m['scenes'][opt.scene]['asset_root']
    case = m['cases'][opt.case_id]['path']
    assert (root / scene).is_dir(), f"prepared scene missing: {scene}"
    assert (root / case).is_file(), f"case missing: {case}"
    assert base.sha(root/case)==m['cases'][opt.case_id]['sha256'], 'case differs from frozen input'
    for relative,digest in m['scenes'][opt.scene]['files_sha256'].items():
        assert base.sha(root/relative)==digest, relative
    migration=json.loads((root/scene/'migration_manifest.json').read_text())
    for relative,digest in migration['asset_files_sha256'].items():
        assert base.sha(root/asset_root/relative)==digest, relative
    assert base.sha(root / 'authorization_diagnostic_20261115/scene_runner_diagnostic.py') == MECHP_RUNNER_SHA
    assert subprocess.check_output(
        ["podman", "image", "inspect", base.IMAGE, "--format", "{{.Id}}"],
        text=True).strip() == base.IMAGE_ID
    outdir = (root / 'mechpairs_runs' / opt.tag /
              f"mech_{opt.mech_mode}_{opt.monitor_arm}" /
              f"{opt.case_id}_s{int(round(opt.speed * 100)):02d}_seed{opt.seed}")
    outdir.mkdir(parents=True, exist_ok=False)

    def inside(path):
        return '/experiment/' + str(Path(path).resolve().relative_to(root))

    args = dict(
        scene=inside(scene), scene_id=opt.scene, case=inside(case),
        output=inside(outdir / 'position_1'), seed=opt.seed, speed=opt.speed,
        stage='recovery', geometry_profile='standard', controller_profile='legacy',
        cohort_id=None, concurrency=1, render_mode='gpu', target_rtf=1.,
        ros_domain=opt.ros_domain, arm=opt.monitor_arm,
        experiment_phase='mechanism_pair', source_root='/experiment',
        attempt_key=f"diagnostic-v1|{opt.probe_kind}|{opt.mech_mode}|{opt.monitor_arm}|{opt.case_id}|{opt.speed}|{opt.seed}|{opt.source_pin_sha}",
        cohort_manifest=inside(MANIFEST),
        cohort_sha256=hashlib.sha256(manifest_path.read_bytes()).hexdigest())
    name = (f"rulesync-mech-{opt.mech_mode[:6]}-{opt.monitor_arm[:4]}"
            f"-{opt.scene.replace('hssd_', '')[-6:]}"
            f"-{opt.case_id.split('__')[-1]}"
            f"-s{int(round(opt.speed * 100)):02d}-{opt.seed}"
            f"-d{opt.ros_domain}")
    name=name+"-authdiag"
    assert name not in names
    command = base.container(
        name, ["python3", "-c", IN_CONTAINER, MECHP_RUNNER_SHA, json.dumps(args)],
        gpu=True)
    idx = command.index("--entrypoint")
    command[idx:idx] = ["-v", f"{root / asset_root}:/assets/{opt.scene}:ro",
                        "--env", "GZ_IP=127.0.0.1",
                        "--env", f"RULESYNC_MECH_MODE={opt.mech_mode}",
                        "--env", f"RULESYNC_DYNAMIC_ARM={opt.monitor_arm}",
                        "--env",f"RULESYNC_AUTH_PROBE_KIND={opt.probe_kind}"]
    base.write_json(outdir / "position_1_launch.json", {
        "arguments": args, "image": base.IMAGE_ID, "runner": MECHP_RUNNER_SHA,
        "manifest": MANIFEST, "scene": opt.scene, "case": opt.case_id,
        "case_sha256": base.sha(root / case), "mech_mode": opt.mech_mode,
        "monitor_arm": opt.monitor_arm, "speed": opt.speed, "seed": opt.seed,
        "container": name, "run_from": str(root)})
    with (outdir / "position_1_host.log").open("x") as log:
        process = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
    result = outdir / "position_1/result.json"
    row = {"exit_code": process.returncode,
           "result": json.loads(result.read_text()) if result.exists() else None}
    print(json.dumps({"outdir": str(outdir), "exit_code": process.returncode,
                      "result": (row["result"] or {}).get("result"),
                      "error": (row["result"] or {}).get("error")}), flush=True)
    base.write_json(outdir / "closed.json", {"results": [row]})


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", default="hssd_102815835")
    parser.add_argument("--case-id", default="hssd_102815835__route_02")
    parser.add_argument("--speed", type=float, default=0.20)
    parser.add_argument("--seed", type=int, default=52601)
    parser.add_argument("--ros-domain", type=int, default=230)
    parser.add_argument("--tag", default="mechpair_dev_v1")
    parser.add_argument("--mech-mode", choices=["external_update", "stale_candidate", "out_of_order"],
                        default="external_update")
    parser.add_argument("--monitor-arm", choices=["full", "geometry_only"], default="full")
    parser.add_argument('--probe-kind',choices=['stale_identity','late_authorization'],required=True)
    parser.add_argument('--source-pin-sha',required=True)
    options = parser.parse_args()
    assert 0 <= options.ros_domain <= 232
    main(options)
