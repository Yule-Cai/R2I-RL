"""Trace one source-bound Nav2 DEV runner with a persistent command witness."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

from channel_trace_probe_v1 import decode


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--command-witness', action='store_true')
    p.add_argument('runner_arguments', nargs=argparse.REMAINDER)
    a = p.parse_args()
    args = a.runner_arguments
    if args and args[0] == '--':
        args = args[1:]
    assert args and args[0] in ('/experiment/scene_runner_generalization_v1.py',
                                '/experiment/channel_dev_20261104/source/scene_runner_channel_dev_v1.py')
    assert '--cohort-id' not in args, 'This is DEV,not another formal1350 attempt'
    assert '--ros-domain' in args
    domain = int(args[args.index('--ros-domain') + 1])
    assert 0 <= domain <= 232
    os.environ['ROS_DOMAIN_ID'] = str(domain)
    a.output.mkdir(parents=True, exist_ok=False)
    root = a.output.resolve()
    Path(os.environ['LTTNG_HOME']).mkdir(parents=True, exist_ok=True)
    session = 'rulesync_nav2_dev_' + str(os.getpid())
    result = dict(scope='one Nav2 instrumentation/channel DEV case;not a formal comparison or1350 replacement',
                  runner_arguments=args, collector_ros_domain_id=domain,
                  wrapper_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  runner_sha256=hashlib.sha256(Path(args[0]).read_bytes()).hexdigest())
    log = (root / 'trace_commands.jsonl').open('x')

    def tool(cli, check=True):
        r = subprocess.run(cli, capture_output=True, text=True, timeout=30)
        log.write(json.dumps(dict(args=cli, returncode=r.returncode, stdout=r.stdout, stderr=r.stderr)) + '\n')
        log.flush()
        if check:
            assert r.returncode == 0, (cli, r.stdout, r.stderr)
        return r

    created, started = False, False
    witness = None
    witness_console = None
    try:
        tool(['lttng', 'create', session, '--output=' + str(root / 'ctf')])
        created = True
        tool(['lttng', 'enable-channel', '-u', '--subbuf-size=64K', '--num-subbuf=4', 'nav2_channel'])
        names = ['rcl_node_init', 'rcl_publisher_init', 'rmw_publisher_init', 'rcl_subscription_init',
                 'rmw_subscription_init', 'rclcpp_publish', 'rclcpp_intra_publish', 'rcl_publish', 'rmw_publish', 'rmw_take', 'rcl_take']
        tool(['lttng', 'enable-event', '-u', '-c', 'nav2_channel', ','.join('ros2:' + n for n in names)])
        tool(['lttng', 'add-context', '-u', '-c', 'nav2_channel', '-t', 'vpid', '-t', 'vtid', '-t', 'procname'])
        tool(['lttng', 'start', session])
        started = True
        if a.command_witness:
            witness_source = Path(__file__).with_name('command_witness_v1.py')
            result['command_witness_source_sha256'] = hashlib.sha256(witness_source.read_bytes()).hexdigest()
            witness_console = (root / 'command_witness.console.txt').open('x')
            witness = subprocess.Popen([sys.executable, str(witness_source), '--root', str(root)],
                                       stdout=witness_console, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + 20
            while not (root / 'command_witness.ready').exists():
                assert witness.poll() is None and time.monotonic() < deadline, 'Witness startup failed'
                time.sleep(.02)
        result['runner_started_monotonic_ns'] = time.monotonic_ns()
        r = subprocess.run([sys.executable, *args], timeout=1000)
        result['runner_exit_code'] = r.returncode
        result['runner_finished_monotonic_ns'] = time.monotonic_ns()
    except Exception as exc:
        result['error'] = repr(exc)
    finally:
        if witness is not None:
            (root / 'command_witness.stop').write_text(str(time.monotonic_ns()))
            try:
                result['command_witness_exit_code'] = witness.wait(timeout=10)
            except subprocess.TimeoutExpired:
                witness.terminate()
                witness.wait(timeout=5)
                result['command_witness_error'] = 'graceful stop timeout'
            witness_console.close()
        if started:
            tool(['lttng', 'stop', session], check=False)
        if created:
            tool(['lttng', 'destroy', session], check=False)
        log.close()
        cg = Path('/sys/fs/cgroup')
        (root / 'cgroup_resources.json').write_text(json.dumps({n: (cg/n).read_text() for n in ['memory.peak', 'memory.events', 'memory.swap.peak', 'cpu.stat'] if (cg/n).exists()}, indent=2) + '\n')
    if created:
        try:
            result['trace'] = decode(root / 'ctf', root / 'native_events.jsonl')
        except Exception as exc:
            result['decode_error'] = repr(exc)
    (root / 'trace_result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    assert 'error' not in result and 'decode_error' not in result
    assert result['runner_exit_code'] == 0
    assert not result['trace']['discarded_messages']
    if a.command_witness:
        assert result.get('command_witness_exit_code') == 0 and 'command_witness_error' not in result


if __name__ == '__main__':
    main()
