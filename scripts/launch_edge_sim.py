#!/usr/bin/env python3
"""One-shot Orange Pi edge simulation launcher. No daemon or boot-time changes.

Runs the existing scenario with the verified A53/local-UDP profile, then audits
its raw receipts. The advisory lock excludes other launches through this entry
point only; manually launched ROS processes must not share domain 58.
"""
import argparse
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys


def output_path(workspace, name):
    if not re.fullmatch(r'[a-z0-9][a-z0-9_-]{0,63}', name):
        raise ValueError('run name must be 1-64 lowercase letters, digits, underscores or hyphens')
    parent = (workspace / 'evidence').resolve()
    if not parent.is_relative_to(workspace.resolve()):
        raise ValueError('evidence directory must remain inside the workspace')
    output = parent / name
    if output.exists() or output.is_symlink():
        raise FileExistsError(f'will not overwrite evidence: {output}')
    return output


def runtime_command(workspace, output):
    return ['bash', '--noprofile', '--norc', '-c',
        'set -e; source "$1"; source "$2"; shift 2; exec "$@"', 'edge-launch',
        '/opt/ros/jazzy/setup.bash', str(workspace / 'install/setup.bash'),
        'taskset', '-c', '0-3', 'python3',
        str(workspace / 'source/scripts/run_edge_closed_loop.py'),
        '--transport', 'UDPv4', '--output', str(output)]


def check_temperature_limit(temperatures, thermal_abort_C=85.):
    """Observe-only (None) never vetoes a run; OS/hardware protection is untouched."""
    if thermal_abort_C is not None and (
            not temperatures or max(temperatures.values()) >= thermal_abort_C):
        raise RuntimeError(f'missing temperature sensors or board already >={thermal_abort_C:g} C')


def preflight(workspace, thermal_abort_C=85.):
    required = [Path('/opt/ros/jazzy/setup.bash'), workspace / 'install/setup.bash',
        workspace / 'source/scripts/run_edge_closed_loop.py',
        workspace / 'source/scripts/audit_edge_closed_loop.py']
    for path in required:
        if not path.is_file():
            raise FileNotFoundError(path)
    if not hasattr(os, 'sched_getaffinity') or not {0, 1, 2, 3} <= os.sched_getaffinity(0):
        raise RuntimeError('this profile requires available Linux CPU cores 0-3')
    for tool in ('bash', 'taskset', 'python3'):
        if shutil.which(tool) is None:
            raise RuntimeError(f'missing executable: {tool}')
    temperatures = {str(p): int(p.read_text()) / 1000. for p in
        Path('/sys/class/thermal').glob('thermal_zone*/temp')}
    check_temperature_limit(temperatures, thermal_abort_C)
    free = shutil.disk_usage(workspace).free
    if free < 128 * 2**20:
        raise RuntimeError('less than 128 MiB free storage; preserve existing evidence')
    return dict(temperatures_C=temperatures, free_storage_bytes=free,
                thermal_abort_C=thermal_abort_C)


def main():
    source = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path,
        default=source.parent if source.name == 'source' else source)
    parser.add_argument('--check', action='store_true', help='preflight only; do not start ROS nodes')
    parser.add_argument('--thermal-policy', choices=('guard', 'observe-only'), default='guard',
        help='guard: historical 85 C startup veto; observe-only: record temperature without a launcher threshold')
    parser.add_argument('--run-name', default='edge-' + datetime.now(timezone.utc).strftime('%Y%m%d-%H%M%S'))
    args = parser.parse_args()
    workspace = args.workspace.resolve()
    output = output_path(workspace, args.run_name)
    profile = dict(ROS_DOMAIN_ID='58', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
        LC_ALL='C.UTF-8', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
        RMW_IMPLEMENTATION='rmw_fastrtps_cpp', FASTDDS_BUILTIN_TRANSPORTS='UDPv4')
    pre = preflight(workspace, 85. if args.thermal_policy == 'guard' else None)
    command = runtime_command(workspace, output)
    print(json.dumps(dict(preflight=pre, profile=profile, command=command,
        thermal_policy=args.thermal_policy,
        scope='board-local ideal 2-D simulation, not physical robot or full Nav2')), flush=True)
    if args.check:
        return 0
    (workspace / 'bootstrap').mkdir(exist_ok=True)
    with (workspace / 'bootstrap/edge-sim-launch.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('another managed edge simulation is running') from None
        env = dict(os.environ, **profile)
        process = subprocess.Popen(command, env=env, cwd=workspace, start_new_session=True)
        def interrupt(_signum, _frame):
            raise KeyboardInterrupt
        previous = signal.signal(signal.SIGTERM, interrupt)
        try:
            code = process.wait()
        except KeyboardInterrupt:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            # The scenario owns and cleans up its separate child process groups.
            process.wait()
            code = 130
        finally:
            signal.signal(signal.SIGTERM, previous)
        launch = dict(profile=profile, command=command, preflight=pre, exit_code=code,
            thermal_policy=args.thermal_policy,
            launcher_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
        if output.is_dir():
            (output / 'launch.json').write_text(json.dumps(launch, indent=2) + '\n')
        if code:
            return code if code > 0 else 128 - code
        audited = subprocess.run([sys.executable,
            str(workspace / 'source/scripts/audit_edge_closed_loop.py'),
            '--source-root', str(workspace / 'source'), str(output)],
            text=True, capture_output=True)
        if audited.returncode:
            print(audited.stderr, file=sys.stderr)
            return audited.returncode
        # Keep the successful independent audit with the run, not just stdout.
        parsed = json.loads(audited.stdout)
        (output / 'audit.json').write_text(json.dumps(parsed, indent=2) + '\n')
        print(audited.stdout, end='', flush=True)
        return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (ValueError, RuntimeError, FileExistsError, FileNotFoundError) as exc:
        print(f'edge launcher: {exc}', file=sys.stderr)
        sys.exit(1)
