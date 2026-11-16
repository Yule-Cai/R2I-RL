"""DEV-only publication ledger around the unchanged production guard handlers."""
import argparse
import itertools
import json
import os
from pathlib import Path
import threading
import time


def main():
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rulesync_guard.guard_node import RuleGuardNode
    from rulesync3d.authorization import path_digest
    from rulesync3d.ruleguard import RuleGuard
    from benchmark_dynamic_rule_update import timed_guard_class

    parser = argparse.ArgumentParser()
    parser.add_argument('--role', choices=['guard'], required=True)
    _, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    os.environ['RULESYNC_CHURN_ROLE'] = 'guard'
    root = Path(os.environ['RULESYNC_CHURN_FOLDER'])
    lock = threading.Lock()
    sequence = itertools.count(1)
    context = threading.local()

    def record(event, **fields):
        row = dict(event=event, mono_ns=time.monotonic_ns(),
                   wall_ns=time.time_ns(), pid=os.getpid(), **fields)
        with lock, (root/'guard_publication_ledger.jsonl').open('a') as stream:
            stream.write(json.dumps(row, allow_nan=False)+'\n')
            stream.flush()

    parent = timed_guard_class(RuleGuardNode, RuleGuard)

    class PublicationProxy:
        def __init__(self, publisher, owner):
            self.publisher, self.owner = publisher, owner

        def __getattr__(self, name):
            return getattr(self.publisher, name)

        def publish(self, message):
            pub_id = next(sequence)
            record('path_publish_begin', publication_id=pub_id,
                   active_authorization_id=getattr(context, 'authorization_id', None),
                   points=len(message.poses), digest=path_digest(message),
                   state=str(self.owner._state), token_id=str(self.owner._token_id),
                   execution=list(self.owner._guard.execution_identity() or []),
                   message=message_to_ordereddict(message))
            try:
                result = self.publisher.publish(message)
            except Exception as exc:
                record('path_publish_error', publication_id=pub_id, error=repr(exc))
                raise
            record('path_publish_return', publication_id=pub_id)
            return result

    class DiagnosticGuard(parent):
        def __init__(self):
            super().__init__()
            self._path_pub = PublicationProxy(self._path_pub, self)
            record('diagnostic_ready', scope='instrumentation only; no predicate override')

        def _on_authorization(self, message):
            auth_id = next(sequence)
            context.authorization_id = auth_id
            record('authorization_enter', authorization_id=auth_id,
                   message=message_to_ordereddict(message))
            try:
                return super()._on_authorization(message)
            finally:
                record('authorization_exit', authorization_id=auth_id,
                       state=str(self._state), reason=str(self._reason))
                context.authorization_id = None

    node = DiagnosticGuard()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
