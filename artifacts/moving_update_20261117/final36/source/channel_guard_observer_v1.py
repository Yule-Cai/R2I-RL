"""Instrument real RuleGuard service correlation without changing its checks.

The inspected rclpy Client.call_async implementation is reproduced with a log
inside the existing lock, so the actual RMW request sequence is captured before
any completion callback can remove it. All pending/future behavior is retained.
"""
import base64
import hashlib
import inspect
import os
from pathlib import Path
import sys

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.serialization import serialize_message
from rclpy.task import Future
from rulesync_guard.guard_node import RuleGuardNode
from rulesync3d.ruleguard import RuleGuard
from benchmark_dynamic_rule_update import timed_guard_class, emit


class ObservedGuard(timed_guard_class(RuleGuardNode,RuleGuard)):
    def __init__(self):
        self.request_ids = {}
        super().__init__()
        client = self._list_cli
        original = type(client).call_async
        emit('channel_guard_instrumentation',
             observer_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
             rclpy_client_source_sha256=hashlib.sha256(Path(inspect.getfile(type(client))).read_bytes()).hexdigest(),
             original_call_async_source=inspect.getsource(original))

        def observed_call(request):
            if not isinstance(request,client.srv_type.Request):
                raise TypeError()
            with client._lock:
                with client.handle:
                    sequence_number = client.handle.send_request(request)
                if sequence_number in client._pending_requests:
                    raise RuntimeError(f'Sequence ({sequence_number}) conflicts with pending request')
                future = Future()
                client._pending_requests[sequence_number] = future
                future.add_done_callback(client.remove_pending_request)
                self.request_ids[id(future)] = sequence_number
                emit('channel_snapshot_request_sent',guard_sequence=self._snapshot_request_seq,
                     rmw_request_sequence=int(sequence_number))
            return future
        client.call_async = observed_call

    def _on_snapshot(self,future,request_seq):
        native = self.request_ids.pop(id(future))
        try:
            response = future.result()
            raw = bytes(serialize_message(response))
            emit('channel_snapshot_response_callback',guard_sequence=request_seq,
                 rmw_request_sequence=int(native),current_generation=request_seq==self._snapshot_pending_seq,
                 epoch=int(response.epoch),active_hash=response.active_hash,
                 cdr_sha256=hashlib.sha256(raw).hexdigest(),cdr_base64=base64.b64encode(raw).decode('ascii'))
        except Exception as exc:
            emit('channel_snapshot_response_error',guard_sequence=request_seq,
                 rmw_request_sequence=int(native),error=repr(exc))
        return super()._on_snapshot(future,request_seq)


def main():
    os.environ['RULESYNC_CHURN_ROLE']='guard'
    rclpy.init(args=sys.argv[1:])
    node=ObservedGuard()
    executor=MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':main()
