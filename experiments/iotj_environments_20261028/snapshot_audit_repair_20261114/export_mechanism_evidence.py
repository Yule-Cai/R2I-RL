"""Archive the 360 DEV result/event/source witnesses without changing any run."""
import hashlib
import json
from pathlib import Path
import tarfile

ROOT = Path('/home/gamer/Projects/rulesync-iotj-environments-20261028')
DEST = ROOT / 'mechanism_claim_audit_20261114'


def main():
    DEST.mkdir(exist_ok=False)
    files = {}
    bindings = []
    for group in ['mechpair_matrix_v1', 'outoforder_matrix_v1']:
        results = sorted((ROOT / 'mechpairs_runs' / group).rglob('result.json'))
        assert len(results) == (240 if group == 'mechpair_matrix_v1' else 120)
        for result in results:
            run = result.parent
            data = json.loads(result.read_text())
            bindings.append(dict(run=str(run.relative_to(ROOT)), key=data['runtime_binding']['key']))
            selected = [run / name for name in ['result.json', 'manifest.json', 'events.jsonl',
                                                'guard_events.jsonl', 'wal.jsonl', 'state.json',
                                                'manager_events.jsonl', 'repair_instrumentation.json']]
            selected += list((run / 'source').glob('*'))
            selected += list(run.parent.glob('*launch.json'))
            for file in selected:
                if file.is_file():
                    files[str(file.relative_to(ROOT))] = file
    assert len({b['key'] for b in bindings}) == 360
    inventory = {name: dict(bytes=file.stat().st_size, sha256=hashlib.sha256(file.read_bytes()).hexdigest())
                 for name, file in sorted(files.items())}
    (DEST / 'files.json').write_text(json.dumps(inventory, indent=2) + '\n')
    (DEST / 'bindings.json').write_text(json.dumps(bindings, indent=2) + '\n')
    archive = DEST / 'selected_raw_evidence.tar.gz'
    with tarfile.open(archive, 'w:gz') as tar:
        for name, file in sorted(files.items()):
            tar.add(file, arcname=name, recursive=False)
        for name in ['files.json', 'bindings.json']:
            tar.add(DEST / name, arcname=name)
    receipt = dict(rows=360, files=len(files), bytes=archive.stat().st_size,
                   sha256=hashlib.sha256(archive.read_bytes()).hexdigest())
    (DEST / 'receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
    print(json.dumps(receipt))


if __name__ == '__main__':
    main()
