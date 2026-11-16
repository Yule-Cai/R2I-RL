#!/usr/bin/env python3
"""Fixed, independently restarted board cohort using the existing launcher.

No retries, no temperature-dependent cooling, no daemon or OS thermal changes.
Raw outputs remain in distinct evidence directories even when a launch fails.
Offline audits are deliberately deferred until all motion capture ends.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

from launch_edge_sim import output_path


def save(path, record):
    with path.open('x') as stream:
        json.dump(record, stream, indent=2, allow_nan=False)
        stream.write('\n')


def temperatures():
    return {p.parent.name: int(p.read_text())/1000 for p in
            Path('/sys/class/thermal').glob('thermal_zone*/temp')}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', required=True, type=Path)
    parser.add_argument('--cohort-name', required=True)
    parser.add_argument('--attempts', type=int, default=5)
    parser.add_argument('--inter-run-pause-s', type=float, default=10.)
    args = parser.parse_args()
    if args.attempts != 5 or args.inter_run_pause_s != 10.:
        parser.error('this preregistered cohort is five attempts with fixed 10 s inter-run pause')
    workspace = args.workspace.resolve()
    destination = output_path(workspace, args.cohort_name)
    names = [f'{args.cohort_name}-v{i}' for i in range(1, 6)]
    for name in names:
        output_path(workspace, name)  # Refuse any existing attempt before capture.
    source = workspace/'source'
    files = ['scripts/'+name for name in ('run_edge_task_recovery_cohort.py',
        'launch_edge_sim.py', 'launch_edge_volume_physics.py',
        'benchmark_dynamic_volume_update.py', 'volume_task_recovery.py',
        'audit_edge_task_recovery.py', 'audit_dynamic_volume_numpy.py')]
    if any(not (source/name).is_file() for name in files):
        raise FileNotFoundError('missing cohort source')
    destination.mkdir()
    hashes = {}
    for name in files:
        target = destination/'source'/name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source/name, target)
        hashes[name] = hashlib.sha256(target.read_bytes()).hexdigest()
    commands = [[sys.executable, str(source/'scripts/launch_edge_volume_physics.py'),
        '--workspace', str(workspace), '--run-name', name, '--vote-mode', 'persistent',
        '--recover-task', '--thermal-policy', 'observe-only'] for name in names]
    save(destination/'manifest.json', dict(cohort=args.cohort_name, planned=5,
        schedule=[dict(attempt=i, name=name, command=cmd)
                  for i, (name, cmd) in enumerate(zip(names, commands), 1)],
        inter_run_pause_s=10., thermal_policy='observe-only', thermal_abort_C=None,
        cooling_description='User reports repaired cooling; exact heatsink/fan installation not independently inspected.',
        initial_temperatures_C=temperatures(), source_sha256=hashes,
        pre_run_wall_ns=time.time_ns(), cpu_affinity=sorted(os.sched_getaffinity(0)),
        scope='Board-local ROS/Gazebo 3-D gantry task recovery; simulated motion, not real robot.',
        policy='all attempts retained, no retries; exit zero is not audited task success; OS thermal protection unchanged'))
    records = []
    for i, (name, command) in enumerate(zip(names, commands), 1):
        if any(hashlib.sha256((source/p).read_bytes()).hexdigest() != h for p, h in hashes.items()):
            raise RuntimeError('source changed during cohort; do not mix configurations')
        record = dict(attempt=i, name=name, start_wall_ns=time.time_ns(),
                      start_temperatures_C=temperatures())
        save(destination/f'attempt_{i:02d}_start.json', record)
        print(json.dumps(dict(event='attempt_start', **record)), flush=True)
        with (destination/f'attempt_{i:02d}.log').open('x') as log:
            result = subprocess.run(command, cwd=workspace, stdout=log, stderr=subprocess.STDOUT)
        record.update(exit_code=result.returncode, end_wall_ns=time.time_ns(),
                      end_temperatures_C=temperatures(), output_exists=(workspace/'evidence'/name).is_dir())
        records.append(record)
        save(destination/f'attempt_{i:02d}_completion.json', record)
        print(json.dumps(dict(event='attempt_completed', **record)), flush=True)
        if i < 5:
            time.sleep(10.)
    unchanged = all(hashlib.sha256((source/p).read_bytes()).hexdigest() == h for p, h in hashes.items())
    save(destination/'completion.json', dict(planned=5, attempted=len(records),
        launcher_zero_exit=sum(r['exit_code'] == 0 for r in records), source_unchanged=unchanged,
        attempts=records, note='Capture complete; independent audits still required.'))
    print(json.dumps(dict(event='cohort_capture_completed', attempted=len(records), source_unchanged=unchanged)), flush=True)
    return int(not unchanged or any(r['exit_code'] != 0 for r in records))


if __name__ == '__main__':
    sys.exit(main())
