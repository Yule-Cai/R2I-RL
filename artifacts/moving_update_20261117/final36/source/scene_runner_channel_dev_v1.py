"""Source-bound observation and identity-ablation DEV around the frozen runner.

Declared changes: route the actual RuleGuard observation client through the
commit-anchored relay; await matching observed authority before full admission.
Default mode preserves proactive stopping. The explicit five-second external
update mode compares the existing full/geometry-only monitor ablation and then
stops both arms for common readmission. Planner/controller/geometry stay intact.
Cannot run formal1350.
"""
import hashlib
import json
import os
from pathlib import Path
import runpy
import shutil
import subprocess
import sys
import time

import benchmark_nav2_combination as bench


ORIGINAL = Path('/experiment/scene_runner_generalization_v1.py')
ORIGINAL_SHA = 'a6930ebc0f4b990e9d21cdc7e1da95d17646797e38765b8eaf255076b984a3a1'
assert hashlib.sha256(ORIGINAL.read_bytes()).hexdigest() == ORIGINAL_SHA
assert '--cohort-id' not in sys.argv
schedule = Path(os.environ['RULESYNC_CHANNEL_SCHEDULE'])
config = json.loads(schedule.read_text())
assert config['scope'] == 'DEV_commit_anchored_delivery'
source = Path(__file__).resolve().parent
original_processes = bench.base.Processes
original_admit = bench.admit
original_hold = bench.Probe.hold
original_commit = bench.commit_rule
monitor_arm = config.get('monitor_arm','full')
exposure_s = float(config.get('external_update_exposure_s',0.))
assert monitor_arm in ('full','geometry_only') and exposure_s in (0.,5.)


class ChannelProcesses(original_processes):
    def __init__(self, root, env):
        env = dict(env, RULESYNC_DYNAMIC_ARM=monitor_arm)
        super().__init__(root, env)
        self.channel_started = False
        paths = [Path(__file__), source/'snapshot_response_proxy_v1.py',
                 source/'snapshot_response_proxy_v2.py', source/'channel_window_schedule_v1.py', schedule]
        paths.append(source/'channel_guard_observer_v1.py')
        for path in paths:
            shutil.copy2(path, root/'source'/('channel_'+path.name))
        metadata = dict(scope='full-policy channel DEV;not a formal cohort or baseline comparison',
                        immutable_runner_sha256=ORIGINAL_SHA,
                        extra_source_sha256={p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},
                        condition=config['condition'], observation_service='/experiment/channel/list_rules',
                        changes=['RuleGuard manager_service routing only',
                                 'wait for matching independently observed authority before full gate submission'],
                        unchanged=['common proactive update stop','backend','planner/controller/low-level collision monitor',
                                   'task geometry','speed','pose/snapshot expiry and polling','acceptance thresholds'])
        metadata.update(monitor_arm=monitor_arm,external_update_exposure_s=exposure_s)
        if exposure_s:
            metadata['scope']='paired execution-identity ablation under external rule update'
            metadata['changes'] += ['same5s external-update observation window in both arms;no anticipatory stop']
            metadata['unchanged'].remove('common proactive update stop')
            manifest_path=root/'manifest.json'
            manifest=json.loads(manifest_path.read_text())
            manifest['trial']['arm']=monitor_arm
            manifest['mechanism_exposure']=dict(seconds=exposure_s,external_update=True,
                common_stop_at_window_end=True,snapshot_identity_check=monitor_arm=='full')
            manifest_path.write_text(json.dumps(manifest,indent=2)+'\n')
        (root/'channel_adapter.json').write_text(json.dumps(metadata,indent=2)+'\n')

    def start(self, name, args, delay=0.):
        if name == 'guard':
            assert not self.channel_started
            super().start('snapshot_channel', [sys.executable,str(source/'snapshot_response_proxy_v2.py'),
                '--root',str(self.root/'snapshot_channel'),'--schedule',str(schedule),
                '--commit-events',str(self.root/'manager_events.jsonl')], .05)
            self.channel_started = True
            assert args[2:4] == ['--role','guard']
            args = [sys.executable,str(source/'channel_guard_observer_v1.py'),*args[4:],
                    '-p', 'manager_service:=/experiment/channel/list_rules']
        return super().start(name,args,delay)

    def close(self):
        # Keep the relay alive while its real clients shut down. Then request a
        # logged graceful relay stop. Pending intentionally unanswered requests
        # remain classified, never converted to empty responses on shutdown.
        relay = [x for x in self.items if x[0]=='snapshot_channel']
        self.items = [x for x in self.items if x[0]!='snapshot_channel']
        try:
            super().close()
        finally:
            self.items = relay
            if relay and (self.root/'snapshot_channel').exists():
                (self.root/'snapshot_channel/stop').write_text('mission clients closed\n')
                try:
                    for _,process,_ in relay:
                        process.wait(timeout=10.)
                except subprocess.TimeoutExpired:
                    pass
            if relay:
                (self.root/'channel_process_exit.json').write_text(json.dumps(
                    dict(relay_exit_before_forced_cleanup=relay[0][1].poll()))+'\n')
            super().close()
            self.items = []


def observed_full_admit(node,path,epoch,expect=True):
    if expect:
        committed=bench.ready_manager(node,epoch)
        bench.base.wait(node,lambda:node.guard is not None and
            int(node.guard.authority_epoch)==int(committed.epoch) and
            node.guard.authority_hash==committed.active_hash,12.)
        node.event('channel_admission_observation_ready',epoch=int(committed.epoch),
                   snapshot_hash=committed.active_hash)
    return original_admit(node,path,epoch,expect)


def external_update_hold(node,reason):
    if exposure_s and reason=='proactive-update-request':
        node.event('external_update_without_anticipatory_stop',monitor_arm=monitor_arm)
        return
    return original_hold(node,reason)


def measured_commit(node,epoch):
    response=original_commit(node,epoch)
    if exposure_s and epoch==2:
        commits=[json.loads(line) for line in (node.root/'manager_events.jsonl').read_text().splitlines()]
        commits=[r for r in commits if r['event']=='durable_commit' and r['epoch']==2]
        assert len(commits)==1
        start=commits[0]['t_ns'];end=start+round(exposure_s*1e9)
        node.event('revocation_window',commit_ns=start,planned_end_ns=end,
                   seconds=exposure_s,monitor_arm=monitor_arm)
        while time.monotonic_ns()<end:
            bench.spin_for(node,min(.05,(end-time.monotonic_ns())/1e9))
        node.event('revocation_window_end',planned_end_ns=end,monitor_arm=monitor_arm)
        original_hold(node,'measurement-window-end')
    return response


bench.base.Processes=ChannelProcesses
bench.admit=observed_full_admit
bench.Probe.hold=external_update_hold
bench.commit_rule=measured_commit
sys.argv[0]=str(ORIGINAL)
runpy.run_path(str(ORIGINAL),run_name='__main__')
