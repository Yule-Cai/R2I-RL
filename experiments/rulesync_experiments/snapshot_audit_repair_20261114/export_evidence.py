"""Export selected immutable 3D evidence for local snapshot-aware reanalysis."""
import hashlib
import json
from pathlib import Path
import tarfile

ROOT = Path('/home/gamer/Projects/rulesync-iotj-environments-20261028')
DEST = ROOT / 'snapshot_audit_repair_20261114'


def main():
    DEST.mkdir(exist_ok=False)
    matrix = ROOT / 'quad3d_voxel_acceptance_20261108/matrix_final_results_v1.json'
    rows = json.loads(matrix.read_text())
    selected = {str(matrix.relative_to(ROOT)): matrix}
    bindings = []
    for row in rows:
        scene, mode = row['scene'], row['occ']
        runs = ROOT / 'iotj_scene_build/scenes' / scene / 'quad3d_runs' / ('quad3d_matrix_' + mode + '_v1')
        prefix = f"{scene}_route_{row['route']}"
        case_prefix = prefix + ('_s10' if row['speed'] == 0.1 else '') + '_quad3d_v1'
        name = f"{case_prefix}_s{round(row['speed'] * 100):02d}_seed{row['seed']}_{mode}_a1"
        run = runs / name
        assert run.is_dir(), str(run)
        raw = json.loads((run / 'result.json').read_text())
        assert raw['result'] == row['result'], (name, raw['result'], row)
        files = []
        for rel in ['result.json', 'manifest.json', 'launch.json', 'events.jsonl',
                    'trajectory.jsonl', 'guard_events.jsonl', 'audit.json',
                    'authority/wal.jsonl', 'authority/state.json']:
            if (run / rel).exists():
                files.append(run / rel)
        files.extend(f for folder in ['source', 'admissions']
                     for f in (run / folder).rglob('*')
                     if f.is_file() and f.suffix in ('.py', '.json'))
        for file in files:
            selected[str(file.relative_to(ROOT))] = file
        bindings.append(dict(row=row, run=str(run.relative_to(ROOT))))
    assert len(bindings) == 140
    inventory = {name: dict(bytes=file.stat().st_size,
                           sha256=hashlib.sha256(file.read_bytes()).hexdigest())
                 for name, file in sorted(selected.items())}
    (DEST / 'bindings.json').write_text(json.dumps(bindings, indent=2) + '\n')
    (DEST / 'files.json').write_text(json.dumps(inventory, indent=2) + '\n')
    archive = DEST / 'selected_raw_evidence.tar.gz'
    with tarfile.open(archive, 'w:gz') as tar:
        for name, file in sorted(selected.items()):
            tar.add(file, arcname=name, recursive=False)
        tar.add(DEST / 'bindings.json', arcname='bindings.json')
        tar.add(DEST / 'files.json', arcname='files.json')
    receipt = dict(rows=len(bindings), files=len(inventory), bytes=archive.stat().st_size,
                   sha256=hashlib.sha256(archive.read_bytes()).hexdigest())
    (DEST / 'receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
    print(json.dumps(receipt))


if __name__ == '__main__':
    main()
