"""DEV-only ListRules response delivery relay; never changes the backend.

An explicit request-ordinal schedule qualifies the relay, not a formal network
model. Delays and suppression are application response faults, not IP loss.
Cached responses retain every original field, epoch and hash. Application CDR
serializations are preserved as evidence; alignment padding is not payload.
"""
import argparse
import base64
import copy
import hashlib
import heapq
import json
import os
from pathlib import Path
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rclpy.task import Future
from rulesync_interfaces.srv import ListRules


class SnapshotResponseProxy(Node):
    def __init__(self, root, schedule, upstream, downstream):
        super().__init__('rulesync_snapshot_response_proxy_dev')
        self.root = root
        self.schedule = schedule
        self.log = (root / 'proxy.jsonl').open('x')
        self.seq = 0
        self.pending = []
        self.unanswered = []
        self.cache = {}
        self.fatal = None
        self.group = ReentrantCallbackGroup()
        self.client = self.create_client(ListRules, upstream, callback_group=self.group)
        self.server = self.create_service(ListRules, downstream, self.on_request, callback_group=self.group)
        self.create_timer(.005, self.deliver, callback_group=self.group)
        self.event('start', upstream=upstream, downstream=downstream,
                   ros_domain_id=os.environ.get('ROS_DOMAIN_ID'),
                   source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                   schedule=schedule)

    def event(self, event, **fields):
        row = dict(event=event, monotonic_ns=time.monotonic_ns(), **fields)
        self.log.write(json.dumps(row) + '\n')
        self.log.flush()
        return row['monotonic_ns']

    def spec_for(self, seq, source, observed_ns):
        return self.schedule['requests'].get(str(seq), dict(action='forward', delay_s=0.))

    async def on_request(self, request, response):
        self.seq += 1
        seq = self.seq
        # Fail visibly rather than returning a fabricated default response.
        assert seq <= self.schedule['maximum_requests'], 'Explicit bounded DEV request budget exceeded'
        self.event('request', seq=seq)
        source = await self.client.call_async(request)
        assert source is not None
        raw = bytes(serialize_message(source))
        digest = hashlib.sha256(raw).hexdigest()
        observed_ns = self.event('source_response', seq=seq, epoch=int(source.epoch), active_hash=source.active_hash,
                   degraded=bool(source.degraded), cdr_sha256=digest,
                   cdr_base64=base64.b64encode(raw).decode('ascii'))
        self.cache.setdefault(int(source.epoch), (source, raw, seq))
        try:
            spec = self.spec_for(seq, source, observed_ns)
        except Exception as exc:
            self.fatal = repr(exc)
            self.event('selection_error', seq=seq, error=repr(exc))
            raise
        assert spec['action'] in ('forward', 'suppress', 'cached')
        selected, selected_raw, origin = source, raw, seq
        if spec['action'] == 'cached':
            selected, selected_raw, origin = self.cache[int(spec['cache_epoch'])]
        selected = copy.deepcopy(selected)
        delay_ns = int(round(float(spec['delay_s']) * 1e9))
        assert 0 <= delay_ns <= 10**10
        planned_ns = time.monotonic_ns() + delay_ns
        self.event('decision', seq=seq, action=spec['action'], delay_ns=delay_ns,
                   planned_delivery_ns=None if spec['action'] == 'suppress' else planned_ns,
                   origin_seq=origin, selected_epoch=int(selected.epoch), selected_hash=selected.active_hash,
                   selected_cdr_sha256=hashlib.sha256(selected_raw).hexdigest(), **spec.get('_context', {}))
        # A zero-delay response has no scheduled waiting obligation. Return it
        # in this callback instead of relying on another executor timer turn.
        if spec['action'] != 'suppress' and delay_ns == 0:
            self.event('dispatch', seq=seq, planned_delivery_ns=planned_ns,
                       epoch=int(selected.epoch), active_hash=selected.active_hash)
            self.before_delivery(seq, selected)
            self.event('callback_return_ready', seq=seq)
            return selected
        wait = Future()
        if spec['action'] == 'suppress':
            # No invalid/default reply masquerading as packet loss. The caller's
            # timeout is external; this coroutine stays unanswered until teardown.
            self.unanswered.append(wait)
        else:
            heapq.heappush(self.pending, (planned_ns, seq, wait, selected))
        try:
            selected = await wait
            self.event('callback_return_ready', seq=seq)
            return selected
        except Exception as exc:
            self.fatal = repr(exc)
            self.event('callback_error', seq=seq, error=repr(exc))
            raise

    def deliver(self):
        now = time.monotonic_ns()
        while self.pending and self.pending[0][0] <= now:
            planned, seq, future, response = heapq.heappop(self.pending)
            self.event('dispatch', seq=seq, planned_delivery_ns=planned,
                       epoch=int(response.epoch), active_hash=response.active_hash)
            self.before_delivery(seq,response)
            future.set_result(response)

    def before_delivery(self,seq,response):
        pass


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--schedule', type=Path, required=True)
    p.add_argument('--upstream', default='/rulesync/list_rules')
    p.add_argument('--downstream', default='/experiment/channel/list_rules')
    a = p.parse_args()
    a.root.mkdir(parents=True, exist_ok=False)
    schedule = json.loads(a.schedule.read_text())
    assert schedule['schema_version'] == 1 and schedule['scope'] == 'DEV_request_ordinal_qualification'
    rclpy.init()
    node = SnapshotResponseProxy(a.root, schedule, a.upstream, a.downstream)
    try:
        deadline = time.monotonic() + 15
        while not node.client.service_is_ready():
            assert time.monotonic() < deadline
            rclpy.spin_once(node, timeout_sec=.02)
        (a.root / 'ready.json').write_text(json.dumps(dict(ready=True, monotonic_ns=time.monotonic_ns())) + '\n')
        while not (a.root / 'stop').exists():
            assert node.fatal is None, node.fatal
            rclpy.spin_once(node, timeout_sec=.01)
        node.event('stop', requests=node.seq, scheduled_pending=len(node.pending),
                   deliberately_unanswered=len(node.unanswered))
    finally:
        node.log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
