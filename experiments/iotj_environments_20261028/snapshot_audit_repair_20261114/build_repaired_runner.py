"""Build an isolated, hash-pinned runner; only the post-hoc audit changes."""
import ast
import hashlib
import json
from pathlib import Path
import shutil

BASE = Path(__file__).resolve().parent
PROJECT = BASE.parent
VARIANT = 'quad3d_snapshot_dev_20261114_v3'


def replace_once(text, old, new):
    assert text.count(old) == 1, old[:100]
    return text.replace(old, new)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    dest = PROJECT / VARIANT
    dest.mkdir(exist_ok=False)
    raw = PROJECT.parents[1] / 'output/snapshot_audit_repair_20261114/raw'
    binding = json.loads((raw / 'bindings.json').read_text())[0]
    source = raw / binding['run'] / 'source'
    (dest / 'quad3d').mkdir()
    for file in source.glob('*.py'):
        target = dest / file.name if file.name.startswith('scene_runner') else dest / 'quad3d' / file.name
        shutil.copyfile(file, target)
    shutil.copyfile(PROJECT / 'quad3d/quad3d_case_schema_v1.json', dest / 'quad3d/quad3d_case_schema_v1.json')
    shutil.copyfile(BASE / 'snapshot_history.py', dest / 'quad3d/snapshot_history.py')
    runner = dest / 'scene_runner_quad3d_v1.py'
    original = runner.read_text()
    updated = replace_once(original, 'from quad3d import authority3d, scene3d',
                           'from quad3d import authority3d, scene3d, snapshot_history')
    old = '''        initial_volumes = self.rules["initial"]["volumes"]
        committed_volumes = (self.rules["initial"]["volumes"]
                             + self.rules["replacement"]["volumes"])'''
    new = '''        history = snapshot_history.replay_snapshots(
            self.root / "authority/wal.jsonl", self.authority._hash)
        initial_snapshot = history[self.initial_admission["epoch"]]
        active_snapshot = history[self.execution["epoch"]]
        if (initial_snapshot["snapshot_hash"] != self.initial_admission["snapshot_hash"]
                or active_snapshot["snapshot_hash"] != self.execution["snapshot_hash"]
                or active_snapshot["snapshot_hash"] != self.authority.snapshot_hash
                or active_snapshot["volumes"] != self.authority.volumes):
            raise RunnerError("audit snapshot identity does not match WAL/admission")
        initial_volumes = initial_snapshot["volumes"]
        committed_volumes = active_snapshot["volumes"]'''
    updated = replace_once(updated, old, new)
    updated = replace_once(updated, '''        audit = {
            "declared_initial_path_clearance"''', '''        audit = {
            "snapshot_semantics": "complete_committed_snapshot_from_WAL",
            "audited_snapshot_volume_ids": {
                str(epoch): [v.get("id") for v in snapshot["volumes"]]
                for epoch, snapshot in history.items()},
            "declared_initial_path_clearance"''')
    runner.write_text(updated)
    # Assert no planning/control/commit/monitor methods were changed.
    before, after = ast.parse(original), ast.parse(updated)
    def methods(tree):
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Mission')
        return {n.name: ast.dump(n, include_attributes=False) for n in cls.body if isinstance(n, ast.FunctionDef)}
    a, b = methods(before), methods(after)
    changed = [name for name in a if a[name] != b[name]]
    assert changed == ['audit'], changed
    launcher = (PROJECT / 'run_hssd_quad3d_attempt.py').read_text()
    launcher = replace_once(launcher, 'RUNNER = "scene_runner_quad3d_v1.py"',
                             f'RUNNER = "{VARIANT}/scene_runner_quad3d_v1.py"')
    launcher = replace_once(launcher, 'RUNNER_SHA = "847a322076e2b2df2ff9dfa0090c1dab478ec6d1cde1f5c02c09d23d945ca72f"',
                             f'RUNNER_SHA = "{sha(runner)}"')
    start, end = launcher.index('MODULE_SHA = {'), launcher.index('RULESYNC3D_SHA = {')
    modules = {f'{VARIANT}/quad3d/{file.name}': sha(file)
               for file in sorted((dest / 'quad3d').iterdir()) if file.is_file()}
    launcher = launcher[:start] + 'MODULE_SHA = ' + json.dumps(modules, indent=4) + '\n' + launcher[end:]
    launcher = replace_once(launcher, "sys.path.insert(0, '/experiment')",
                             f"sys.path.insert(0, '/experiment/{VARIANT}')")
    launcher = replace_once(launcher, "runner = Path('/experiment/scene_runner_quad3d_v1.py')",
                             f"runner = Path('/experiment/{VARIANT}/scene_runner_quad3d_v1.py')")
    launcher = replace_once(launcher, '    options = parser.parse_args()',
                             '''    parser.add_argument("--source-pin-sha", required=True)
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
        assert case_speed == options.speed, "case speed differs from trial speed"''')
    launcher = replace_once(launcher, 'assert Path(__file__).resolve().parent == ROOT,',
                             f'assert Path(__file__).resolve().parent == ROOT / "{VARIANT}",')
    (dest / 'run_hssd_quad3d_attempt.py').write_text(launcher)
    files = {str(file.relative_to(dest)): sha(file) for file in sorted(dest.rglob('*')) if file.is_file()}
    pin = dict(schema='rulesync.snapshot-audit-repair.v3', files=files,
               original_runner_sha256=hashlib.sha256(original.encode()).hexdigest(),
               changed_mission_methods=changed, original_formal_results_untouched=True)
    (dest / 'source_pin.json').write_text(json.dumps(pin, indent=2) + '\n')
    print(json.dumps(dict(variant=str(dest), files=len(files), pin_sha256=sha(dest / 'source_pin.json'))))


if __name__ == '__main__':
    main()
