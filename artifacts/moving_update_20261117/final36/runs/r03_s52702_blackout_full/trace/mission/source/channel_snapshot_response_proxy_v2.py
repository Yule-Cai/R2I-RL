"""DEV relay whose impairment window follows the real manager COMMIT log."""
import argparse
import hashlib
import json
from pathlib import Path
import time

import rclpy
from channel_window_schedule_v1 import choose, read_anchor
from snapshot_response_proxy_v1 import SnapshotResponseProxy


class CommitAnchoredProxy(SnapshotResponseProxy):
    def __init__(self, root, schedule, upstream, downstream, commit_events):
        self.commit_events = commit_events
        self.response_seq = {}
        super().__init__(root, schedule, upstream, downstream)
        original_send = self.server.send_response
        def observed_send(response,header):
            seq = self.response_seq.pop(id(response))
            request_id = getattr(header,'request_id',header)
            request_sequence = int(request_id.sequence_number)
            self.event('service_send_begin',seq=seq,rmw_request_sequence=request_sequence)
            original_send(response,header)
            self.event('service_send_returned',seq=seq,rmw_request_sequence=request_sequence)
        self.server.send_response = observed_send
        self.event('commit_window_sources', commit_events=str(commit_events),
                   source_sha256={p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in
                                  [Path(__file__), Path(__file__).with_name('channel_window_schedule_v1.py'),
                                   Path(__file__).with_name('snapshot_response_proxy_v1.py')]})

    def before_delivery(self,seq,response):
        assert id(response) not in self.response_seq
        self.response_seq[id(response)] = seq

    def spec_for(self, seq, source, observed_ns):
        anchor = read_anchor(self.commit_events, int(self.schedule['anchor_epoch']))
        if int(source.epoch) == int(self.schedule['anchor_epoch']):
            assert anchor is not None, 'Post-commit state without its durable hook event'
            assert source.active_hash == anchor['snapshot_hash'], 'Source identity disagrees with COMMIT'
            assert int(anchor['t_ns']) <= observed_ns
        return choose(self.schedule, seq, observed_ns, anchor)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,required=True)
    p.add_argument('--schedule',type=Path,required=True)
    p.add_argument('--commit-events',type=Path,required=True)
    p.add_argument('--upstream',default='/rulesync/list_rules')
    p.add_argument('--downstream',default='/experiment/channel/list_rules')
    a=p.parse_args()
    config=json.loads(a.schedule.read_text())
    assert config['schema_version']==2 and config['scope']=='DEV_commit_anchored_delivery'
    assert config['anchor_epoch']==2 and config['maximum_requests']>=100
    assert 0<=config['window_start_s']<config['window_end_s']<=10
    a.root.mkdir(parents=True,exist_ok=False)
    (a.root/'schedule.json').write_bytes(a.schedule.read_bytes())
    rclpy.init()
    node=CommitAnchoredProxy(a.root,config,a.upstream,a.downstream,a.commit_events)
    try:
        deadline=time.monotonic()+20
        while not node.client.service_is_ready():
            assert time.monotonic()<deadline
            rclpy.spin_once(node,timeout_sec=.02)
        (a.root/'ready.json').write_text(json.dumps(dict(ready=True,monotonic_ns=time.monotonic_ns()))+'\n')
        while not (a.root/'stop').exists():
            assert node.fatal is None,node.fatal
            rclpy.spin_once(node,timeout_sec=.01)
        node.event('stop',requests=node.seq,scheduled_pending=len(node.pending),
                   deliberately_unanswered=len(node.unanswered))
    finally:
        node.log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':main()
