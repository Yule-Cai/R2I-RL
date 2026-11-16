#!/usr/bin/env python3
"""Bounded board-local Gazebo 3-D authorization attempt, not physical flight.

Reuses the existing gantry benchmark (full arm) rather than a second simulator.
No sensor rendering or GUI; retains gravity/inertia/joints and production 2PC.
"""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from launch_edge_sim import output_path, preflight, check_temperature_limit


def resources(root_pid):
    records = {}
    for path in Path('/proc').glob('[0-9]*/stat'):
        try:
            records[int(path.parent.name)] = path.read_text().rsplit(')', 1)[1].split()
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            pass
    pids = {root_pid}
    while True:
        descendants = {pid for pid, values in records.items() if int(values[1]) in pids}
        if descendants <= pids:
            break
        pids |= descendants
    return dict(at_ns=time.monotonic_ns(), processes=[dict(pid=pid,
        rss_bytes=int(records[pid][21])*os.sysconf('SC_PAGE_SIZE'),
        cpu_ticks=int(records[pid][11])+int(records[pid][12])) for pid in sorted(pids & records.keys())],
        temperatures_mC={p.parent.name: int(p.read_text()) for p in
            Path('/sys/class/thermal').glob('thermal_zone*/temp')},
        meminfo=Path('/proc/meminfo').read_text())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--run-name', required=True)
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--vote-mode', choices=('helper', 'persistent'), default='helper')
    parser.add_argument('--recover-task', action='store_true')
    parser.add_argument('--thermal-policy', choices=('guard', 'observe-only'), default='guard',
        help='guard: historical 85 C abort; observe-only: record temperature without a launcher threshold')
    args = parser.parse_args()
    workspace = args.workspace.resolve()
    output = output_path(workspace, args.run_name)
    thermal_abort_C = 85. if args.thermal_policy == 'guard' else None
    checked = preflight(workspace, thermal_abort_C)
    source = workspace / 'source'
    for relative in ('scripts/benchmark_dynamic_volume_update.py', 'rulesync/vote_volume_once.py',
                     'rulesync3d/worlds/r2i_quad.sdf'):
        if not (source / relative).is_file():
            raise FileNotFoundError(source / relative)
    environment = dict(os.environ, ROS_DOMAIN_ID='59', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
        RMW_IMPLEMENTATION='rmw_fastrtps_cpp', FASTDDS_BUILTIN_TRANSPORTS='UDPv4',
        OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', LC_ALL='C.UTF-8',
        RULESYNC_VOLUME_VOTE_HELPER=str(source / 'rulesync/vote_volume_once.py'),
        RULESYNC_VOLUME_VOTE_MODE=args.vote_mode)
    environment.pop('ROS_DISCOVERY_SERVER', None)
    command = ['bash', '--noprofile', '--norc', '-c',
        'set -e; source "$1"; source "$2"; shift 2; exec "$@"', 'edge3d-launch',
        '/opt/ros/jazzy/setup.bash', str(workspace / 'install/setup.bash'),
        'taskset', '-c', '0-3', 'python3', str(source / 'scripts/benchmark_dynamic_volume_update.py'),
        '--local-domain', '59', '--arms', 'full', '--repetitions', '1',
        '--exposure', '65' if args.recover_task else '12', '--output', str(output)]
    if args.recover_task:
        command.append('--recover-task')
    probe = subprocess.run(['bash', '--noprofile', '--norc', '-c',
        'set -e; source "$1"; exec gz sim --versions', 'gz-preflight',
        '/opt/ros/jazzy/setup.bash'], env=environment, text=True,
        capture_output=True, timeout=30.)
    if probe.returncode:
        raise RuntimeError('Gazebo preflight failed: ' + probe.stderr[-1000:])
    checked['gazebo_versions'] = probe.stdout.strip()
    print(json.dumps(dict(preflight=checked, command=command,
        thermal_policy=args.thermal_policy,
        scope='ARM board-local 3-D Gazebo gantry; not real robot or free flight')), flush=True)
    if args.check:
        return 0
    with (workspace / 'bootstrap/edge-sim-launch.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('another managed edge simulation is running') from None
        process = subprocess.Popen(command, env=environment, cwd=workspace, start_new_session=True)
        samples, aborted, start = [], None, time.monotonic()
        def interrupt(_signum, _frame):
            raise KeyboardInterrupt
        previous = signal.signal(signal.SIGTERM, interrupt)
        try:
            while process.poll() is None:
                row = resources(process.pid)
                samples.append(row)
                try:
                    check_temperature_limit({k: v/1000. for k, v in row['temperatures_mC'].items()},
                                            thermal_abort_C)
                except RuntimeError:
                    aborted = 'missing temperature or sampled >=85 C'
                    break
                available = next(int(line.split()[1]) * 1024 for line in row['meminfo'].splitlines()
                                 if line.startswith('MemAvailable:'))
                if available < 512 * 2**20:
                    aborted = 'system available memory <512 MiB'
                    break
                if time.monotonic() - start > 300.:
                    aborted = '300 s wall-clock attempt budget exceeded'
                    break
                time.sleep(1.)
        except KeyboardInterrupt:
            aborted = 'user/launcher interrupt'
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            # The benchmark owns its independent child groups and cleans them
            # in its finally block. Do not use global pkill on the board.
            process.wait()
            signal.signal(signal.SIGTERM, previous)
        report = dict(exit_code=process.returncode, abort_reason=aborted,
            preflight=checked, command=command,
            elapsed_wall_s=time.monotonic()-start, thermal_abort_C=thermal_abort_C,
            thermal_policy=args.thermal_policy,
            memory_abort_MiB=512, wall_budget_s=300, domain=59, cpu_affinity=[0,1,2,3],
            helper=str(source / 'rulesync/vote_volume_once.py'),
            vote_mode=args.vote_mode,
            task_recovery=args.recover_task,
            launcher_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
        destination = output if output.is_dir() else workspace / 'bootstrap'
        (destination / (args.run_name + '-resources.jsonl')).write_text(
            ''.join(json.dumps(row)+'\n' for row in samples))
        (destination / (args.run_name + '-launch.json')).write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report), flush=True)
        return 1 if aborted or process.returncode else 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (RuntimeError, FileNotFoundError, ValueError, FileExistsError) as exc:
        print(f'edge3d launcher: {exc}', file=sys.stderr)
        sys.exit(1)
