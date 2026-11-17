"""Isolated ROS2 three-process instrumentation probe; no robot or rule benchmark."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def role_main(role, root, count):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from geometry_msgs.msg import Twist

    log = (root / (role + '.jsonl')).open('x')

    def emit(kind, **values):
        stamp = time.monotonic_ns()
        log.write(json.dumps(dict(kind=kind, monotonic_ns=stamp, pid=os.getpid(), **values)) + '\n')
        log.flush()

    rclpy.init()
    node = Node('rulesync_trace_probe_' + role)
    received = []
    deadline = time.monotonic() + 35
    emit('node_ready', role=role)
    try:
        if role == 'proposer':
            pub = node.create_publisher(String, '/rulesync_trace_probe/proposal', 32)
            while not (root / 'gate.ready').exists() or pub.get_subscription_count() < 1:
                assert time.monotonic() < deadline, 'Proposal subscription readiness timeout'
                rclpy.spin_once(node, timeout_sec=.05)
            for seq in range(count):
                epoch = 1 if seq < count // 2 else 2
                data = dict(message_id=seq, epoch=epoch, snapshot_hash=hashlib.sha256(f'trace-probe-{epoch}'.encode()).hexdigest())
                msg = String(data=json.dumps(data, sort_keys=True))
                emit('proposal_publish_begin', **data)
                pub.publish(msg)
                emit('proposal_publish_end', **data)
                rclpy.spin_once(node, timeout_sec=.025)
            settle = time.monotonic() + .5
            while time.monotonic() < settle:
                rclpy.spin_once(node, timeout_sec=.02)
        elif role == 'gate':
            pub = node.create_publisher(Twist, '/rulesync_trace_probe/final_cmd', 32)

            def proposal(msg):
                data = json.loads(msg.data)
                emit('proposal_receive', **data)
                command = Twist()
                command.linear.x = .1
                emit('final_publish_begin', **data)
                pub.publish(command)
                emit('final_publish_end', **data)
                received.append(data['message_id'])

            sub = node.create_subscription(String, '/rulesync_trace_probe/proposal', proposal, 32)
            while pub.get_subscription_count() < 1:
                assert time.monotonic() < deadline, 'Final observer readiness timeout'
                rclpy.spin_once(node, timeout_sec=.05)
            (root / 'gate.ready').write_text(str(time.monotonic_ns()))
            while len(received) < count:
                assert time.monotonic() < deadline, 'Gate receive timeout'
                rclpy.spin_once(node, timeout_sec=.05)
            settle = time.monotonic() + .5
            while time.monotonic() < settle:
                rclpy.spin_once(node, timeout_sec=.02)
        else:
            def command(msg):
                emit('final_observer_receive', receipt_index=len(received), v=msg.linear.x, w=msg.angular.z)
                received.append(True)

            sub = node.create_subscription(Twist, '/rulesync_trace_probe/final_cmd', command, 32)
            while len(received) < count:
                assert time.monotonic() < deadline, 'Observer receive timeout'
                rclpy.spin_once(node, timeout_sec=.05)
        emit('role_finished', role=role, received=len(received))
    finally:
        node.destroy_node()
        rclpy.shutdown()
        log.close()


def decode(trace, output):
    import bt2

    clocks, count, discarded = {}, 0, []

    def fields(value):
        if value is None:
            return {}
        result = {}
        for name in value:
            field = value[name]
            try:
                result[name] = int(field)
            except (TypeError, ValueError):
                result[name] = str(field)
        return result

    with output.open('x') as stream:
        for message in bt2.TraceCollectionMessageIterator(str(trace)):
            if not hasattr(message, 'event'):
                if 'Discarded' in type(message).__name__:
                    discarded.append(type(message).__name__)
                continue
            event = message.event
            clock = message.default_clock_snapshot
            cls = clock.clock_class
            clocks[str(cls.name)] = dict(frequency=int(cls.frequency), offset_seconds=int(cls.offset.seconds), offset_cycles=int(cls.offset.cycles))
            row = dict(name=event.name, clock_value=int(clock.value), ns_from_origin=int(clock.ns_from_origin),
                       clock_name=cls.name, context=fields(event.common_context_field), payload=fields(event.payload_field))
            stream.write(json.dumps(row) + '\n')
            count += 1
    return dict(events=count, discarded_messages=discarded, clocks=clocks)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--count', type=int, default=12)
    p.add_argument('--role', choices=['proposer', 'gate', 'observer'])
    a = p.parse_args()
    assert 2 <= a.count <= 100
    if a.role:
        role_main(a.role, a.output, a.count)
        return
    a.output.mkdir(exist_ok=False, parents=True)
    root = a.output.resolve()
    Path(os.environ['LTTNG_HOME']).mkdir(exist_ok=True, parents=True)
    session = 'rulesync_probe_' + str(os.getpid())
    children, handles = [], []
    commands = (root / 'tool_commands.jsonl').open('x')
    result = dict(scope='three-process same-container ROS2 tracing DEV probe;no navigation or physical robot', count=a.count,
                  source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())

    def run(args, check=True):
        done = subprocess.run(args, capture_output=True, text=True, timeout=20)
        commands.write(json.dumps(dict(args=args, returncode=done.returncode, stdout=done.stdout, stderr=done.stderr)) + '\n')
        commands.flush()
        if check:
            assert done.returncode == 0, (args, done.stdout, done.stderr)
        return done

    created, started = False, False
    try:
        run(['lttng', 'create', session, '--output=' + str(root / 'ctf')])
        created = True
        run(['lttng', 'enable-channel', '-u', '--subbuf-size=64K', '--num-subbuf=4', 'probe_channel'])
        run(['lttng', 'enable-event', '-u', '-c', 'probe_channel', 'ros2:*'])
        run(['lttng', 'add-context', '-u', '-c', 'probe_channel', '-t', 'vpid', '-t', 'vtid', '-t', 'procname'])
        run(['lttng', 'start', session])
        started = True
        for role in ['observer', 'gate', 'proposer']:
            handle = (root / (role + '.console.txt')).open('x')
            handles.append(handle)
            children.append(subprocess.Popen([sys.executable, str(Path(__file__).resolve()), '--role', role,
                                               '--output', str(root), '--count', str(a.count)], stdout=handle, stderr=subprocess.STDOUT))
        codes = [child.wait(timeout=45) for child in children]
        result['child_exit_codes'] = codes
        assert codes == [0, 0, 0], codes
    except Exception as exc:
        result['error'] = repr(exc)
    finally:
        for child in children:
            if child.poll() is None:
                child.terminate()
                child.wait(timeout=5)
        for handle in handles:
            handle.close()
        if started:
            run(['lttng', 'stop', session], check=False)
        if created:
            run(['lttng', 'destroy', session], check=False)
        commands.close()
    if created:
        try:
            result['trace'] = decode(root / 'ctf', root / 'native_events.jsonl')
        except Exception as exc:
            result['decode_error'] = repr(exc)
    (root / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    assert 'error' not in result and 'decode_error' not in result
    assert result['trace']['events'] > 0 and not result['trace']['discarded_messages']


if __name__ == '__main__':
    main()
