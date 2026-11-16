#!/usr/bin/env python3
"""Exhaust a finite single-voter 2-D recovery abstraction, not the ROS system.

Uses production selection/recovery predicates and the bridge certificate
predicate. Authority is fixed during one recovery; retained, valid WAL/storage
and trusted components are assumptions. Delayed state replies/ACKs may describe
past bridge state. READY therefore means observed evidence, NOT instantaneous
global agreement. No motion, geometry, fsync internals or DDS timing is modeled.
"""

import argparse
import ast
from collections import Counter, deque
from dataclasses import asdict, dataclass, replace
import hashlib
import importlib.util
import itertools
import json
from pathlib import Path
import platform
from types import SimpleNamespace as N

ROOT = Path(__file__).resolve().parents[1]
IDENTITY = ROOT / 'rulesync/src/rulesync_manager/rulesync_manager/snapshot_identity.py'
MANAGER = ROOT / 'rulesync/src/rulesync_manager/rulesync_manager/manager_node.py'
BRIDGE = ROOT / 'rulesync/src/rulesync_nav2_bridge/rulesync_nav2_bridge/bridge_node.py'
spec = importlib.util.spec_from_file_location('recovery_identity', IDENTITY)
si = importlib.util.module_from_spec(spec)
spec.loader.exec_module(si)
tree = ast.parse(BRIDGE.read_text())
cert_node = next(n for n in tree.body if isinstance(n, ast.FunctionDef)
                 and n.name == 'bridge_cert_ok')
scope = {}
exec(compile(ast.Module(body=[cert_node], type_ignores=[]), str(BRIDGE), 'exec'), scope)
bridge_cert = scope['bridge_cert_ok']


@dataclass(frozen=True)
class Snapshot:
    epoch: int
    payload: tuple
    digest: str
    ids: tuple


def snapshots():
    def ring(epoch, radius):
        r = dict(rule_id='p0', epoch=epoch, rule_type=0, severity=1,
                 frame_id='map', polygon=[[-radius, -radius, 0],
                 [radius, -radius, 0], [radius, radius, 0], [-radius, radius, 0]],
                 speed_limit=0., valid_from=[0, 0], valid_until=[100, 0])
        return (r,)
    result = []
    for epoch, rows in [(0, ()), (1, ring(1, 2)), (2, ring(2, 1)), (3, ())]:
        msgs = [N(**dict(r, polygon=N(points=[N(x=x, y=y, z=z)
                    for x, y, z in r['polygon']]),
                    valid_from=N(sec=0, nanosec=0),
                    valid_until=N(sec=100, nanosec=0))) for r in rows]
        result.append(Snapshot(epoch, rows, si._rule_snapshot_hash(msgs),
                               tuple(sorted(r['rule_id'] for r in rows))))
    return tuple(result)


SNAPS = snapshots()
MUTANTS = ('ignore-wal', 'adopt-participant', 'compare-ids-only',
           'clear-on-timeout', 'ignore-cert-txn', 'ignore-cert-epoch',
           'ignore-cert-hash')


@dataclass(frozen=True)
class Scenario:
    operation: str
    before: int
    proposed: int
    decision: str
    checkpointed: bool = False

    @property
    def checkpoint(self):
        return self.proposed if self.checkpointed else self.before

    @property
    def authority(self):
        return self.proposed if self.decision == 'COMMIT' else self.checkpoint

    def wal(self):
        target = SNAPS[self.proposed]
        return [dict(kind='rule-decision', decision=self.decision,
                     epoch=target.epoch, txn_id='original-transaction',
                     target_hash=target.digest, target_rules=list(target.payload))]


SCENARIOS = tuple(Scenario(op, before, after, decision)
                  for op, before, after in [('ADD', 0, 1), ('RELAX', 1, 2),
                                            ('REMOVE', 1, 3)]
                  for decision in ['PREPARE', 'ABORT', 'COMMIT']) + tuple(
                  Scenario(op, before, after, 'COMMIT', True)
                  for op, before, after in [('ADD', 0, 1), ('RELAX', 1, 2),
                                            ('REMOVE', 1, 3)])


@dataclass(frozen=True)
class State:
    manager_up: bool = False
    bridge_up: bool = True
    link_up: bool = True
    active: int = 0
    staged: int = -1
    view: int = -1
    ready: bool = False
    response: int = -1
    ack: int = -1
    evidence: int = -1


class Violation(Exception):
    def __init__(self, message, event, state):
        super().__init__(message)
        self.event, self.state = event, state


class Model:
    def __init__(self, scenario, mutant='none'):
        self.scenario, self.mutant = scenario, mutant
        self.authority = scenario.authority  # independent specification

    def select(self):
        cp = SNAPS[self.scenario.checkpoint]
        wal = [] if self.mutant == 'ignore-wal' else self.scenario.wal()
        epoch, rows = si.select_durable_target(cp.epoch, list(cp.payload), wal,
                                                'rule-decision')
        found = next((i for i, s in enumerate(SNAPS)
                      if s.epoch == epoch and list(s.payload) == rows), -1)
        if found != self.authority:
            raise ValueError('durable selection differs from retained decision')
        return found

    def initial(self):
        # Include unexpected/orphan proposed participant state even pre-COMMIT.
        # These are hostile recovery inputs, not asserted reachable normal runs.
        return [State(active=i, staged=j)
                for i in sorted({0, self.scenario.before, self.scenario.proposed})
                for j in [-1, self.scenario.proposed]]

    def check(self, state, event):
        if state.manager_up and state.view != self.authority:
            raise Violation('participant/old checkpoint promoted over authority', event, state)
        if state.ready and (not state.manager_up or state.evidence != self.authority):
            raise Violation('READY without complete authority evidence', event, state)

    def steps(self, s):
        a = self.authority
        if s.manager_up:
            yield 'manager-crash', replace(s, manager_up=False, view=-1,
                                           ready=False, evidence=-1, response=-1, ack=-1)
        else:
            try:
                chosen = self.select()
            except ValueError as e:
                raise Violation(str(e), 'manager-restart', s)
            yield 'manager-restart', replace(s, manager_up=True, view=chosen,
                                             ready=False, evidence=-1)
        if s.bridge_up:
            yield 'bridge-crash', replace(s, bridge_up=False, active=0, staged=-1)
        else:
            yield 'bridge-restart', replace(s, bridge_up=True, active=0, staged=-1)
        yield 'link-down' if s.link_up else 'link-up', replace(s, link_up=not s.link_up)
        if s.response != -1:
            yield 'drop-state-reply', replace(s, response=-1)
        if s.ack != -1:
            yield 'drop-ack', replace(s, ack=-1)
        if not s.manager_up:
            return
        yield 'state-timeout', replace(s, response=-1,
                                      ready=self.mutant == 'clear-on-timeout', evidence=-1)
        # Previously generated replies may be delivered after a bridge crash;
        # bridge liveness is needed for generation but NOT receipt below.
        if s.bridge_up and s.link_up and s.response == -1:
            yield 'query-state', replace(s, response=s.active)
        if s.link_up and s.response != -1:
            d, b = SNAPS[a], SNAPS[s.response]
            decision = si.recovery_action(d.epoch, d.digest, d.ids,
                                          True, b.epoch, b.digest, b.ids)
            if self.mutant == 'adopt-participant':
                yield 'receive-state', replace(s, response=-1, view=s.response,
                                               ready=True, evidence=s.response)
            else:
                matched = decision == 'converge'
                if self.mutant == 'compare-ids-only':
                    matched = d.ids == b.ids
                yield 'receive-state', replace(s, response=-1, ready=matched,
                                               evidence=s.response if matched else -1)
        if s.link_up and s.ack != -1:
            yield 'receive-ack', replace(s, ack=-1, ready=True, evidence=s.ack)
        if not (s.bridge_up and s.link_up):
            return
        yield 'replay-prepare', replace(s, staged=a)
        if s.staged == -1:
            yield 'commit-without-prepare', s
            return
        staged = SNAPS[s.staged]
        target = SNAPS[a]
        prepared = (f'refresh-e{staged.epoch}', staged.epoch, staged.digest)
        requested = (f'refresh-e{target.epoch}', target.epoch, target.digest)
        for variant in ['exact', 'txn', 'epoch', 'hash']:
            req = list(requested)
            if variant == 'txn':
                req[0] += '-old'
            elif variant == 'epoch':
                req[1] += 1
            elif variant == 'hash':
                req[2] = '0' * 64
            event = 'replay-commit-' + variant
            ok = si.cert_matches(*prepared, *req)
            if ok != bridge_cert(*prepared, *req):
                raise Violation('manager/bridge certificate parity differs', event, s)
            if self.mutant.startswith('ignore-cert-'):
                ignore = {'txn': 0, 'epoch': 1, 'hash': 2}[self.mutant.rsplit('-', 1)[1]]
                ok = all(x == y for i, (x, y) in enumerate(zip(prepared, req))
                         if i != ignore)
            if ok and tuple(req) != prepared:
                raise Violation('mismatched certificate accepted', event, s)
            if ok:
                if s.staged != a:
                    raise Violation('replay applied non-authoritative target', event, s)
                yield event, replace(s, active=s.staged, staged=-1, ack=s.staged)
            else:
                yield event + '-rejected', s

    def healthy_recovery(self, s):
        """Constructive conditional convergence, NOT fairness/partition liveness."""
        s = replace(s, bridge_up=True, link_up=True, response=-1, ack=-1,
                    active=s.active if s.bridge_up else 0,
                    staged=s.staged if s.bridge_up else -1)
        # Failed bridge restarts with its volatile state cleared.
        # No permanent storage loss, new decisions or competing requests here.
        if not s.manager_up:
            s = next(t for e, t in self.steps(s) if e == 'manager-restart')
        for event in ['query-state', 'receive-state']:
            s = next(t for e, t in self.steps(s) if e == event)
            self.check(s, event)
        if not s.ready:
            for event in ['replay-prepare', 'replay-commit-exact', 'receive-ack']:
                s = next(t for e, t in self.steps(s) if e == event)
                self.check(s, event)
        if not (s.ready and s.view == self.authority and s.active == self.authority):
            raise Violation('healthy fixed-authority recovery did not converge', 'healthy', s)


def explore(scenario, mutant='none', max_states=100000):
    model = Model(scenario, mutant)
    queue = deque(model.initial())
    parents = {s: None for s in queue}
    events = Counter()
    edges, delayed_ready_mismatch, convergence_checks = 0, 0, 0
    s = queue[0]
    try:
        while queue:
            s = queue.popleft()
            model.check(s, 'state-invariant')
            if s.ready and (not s.bridge_up or s.active != model.authority):
                delayed_ready_mismatch += 1  # intentionally permitted observation lag
            if mutant == 'none':
                model.healthy_recovery(s)
                convergence_checks += 1
            for event, t in model.steps(s):
                edges += 1
                events[event] += 1
                model.check(t, event)
                if t not in parents:
                    if len(parents) >= max_states:
                        raise RuntimeError('state budget exceeded: audit is incomplete')
                    parents[t] = (s, event)
                    queue.append(t)
    except Violation as error:
        trace = []
        current = s
        while parents[current] is not None:
            prev, event = parents[current]
            trace.append(dict(event=event, state=asdict(current)))
            current = prev
        trace.reverse()
        return dict(status='counterexample', scenario=asdict(scenario), mutant=mutant,
                    states=len(parents), transitions=edges, reason=str(error),
                    initial=asdict(current), trace=trace,
                    failing_event=error.event, failing_state=asdict(error.state))
    return dict(status='pass', scenario=asdict(scenario), mutant=mutant,
                states=len(parents), transitions=edges,
                constructive_convergence_checks=convergence_checks,
                observed_ready_bridge_mismatch_states=delayed_ready_mismatch,
                event_counts=dict(sorted(events.items())))


def run_campaign(max_states=100000):
    normal = [explore(s, max_states=max_states) for s in SCENARIOS]
    if any(r['status'] != 'pass' for r in normal):
        raise RuntimeError('baseline recovery model counterexample: ' + json.dumps(normal))
    witnesses = []
    for mutant in MUTANTS:
        witness = next((r for s in SCENARIOS
                        if (r := explore(s, mutant, max_states))['status'] == 'counterexample'), None)
        if witness is None:
            raise RuntimeError('mutation survived: ' + mutant)
        witnesses.append(witness)
    return dict(status='pass', scope='finite single-voter 2-D recovery abstraction',
                scenarios=len(normal), states=sum(r['states'] for r in normal),
                transitions=sum(r['transitions'] for r in normal),
                decision_histories_checked=check_decision_histories(),
                mutations_detected=len(witnesses), scenarios_detail=normal,
                mutation_witnesses=witnesses)


def check_decision_histories():
    """Independent latest-COMMIT oracle; up to two ordered records per WAL.

    Each epoch has one valid full payload. Conflicting same-epoch decisions,
    damaged JSON or lost storage are outside the declared storage assumption.
    """
    records = [(i, decision, kind) for i in [1, 2, 3]
               for decision in ['PREPARE', 'ABORT', 'COMMIT']
               for kind in ['rule-decision', 'vol-decision']]
    count = 0
    for cp in range(len(SNAPS)):
        for length in [0, 1, 2]:
            for history in itertools.product(records, repeat=length):
                eligible = [i for i, decision, kind in history
                            if decision == 'COMMIT' and kind == 'rule-decision']
                expected = max([cp] + eligible)
                wal = [dict(kind=kind, decision=decision, epoch=SNAPS[i].epoch,
                            target_rules=list(SNAPS[i].payload))
                       for i, decision, kind in history]
                epoch, rows = si.select_durable_target(
                    SNAPS[cp].epoch, list(SNAPS[cp].payload), wal, 'rule-decision')
                if (epoch, rows) != (SNAPS[expected].epoch, list(SNAPS[expected].payload)):
                    raise RuntimeError('durable decision-history counterexample')
                count += 1
    return count


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--max-states-per-scenario', type=int, default=100000)
    args = parser.parse_args()
    if args.max_states_per_scenario < 1:
        parser.error('positive state budget required')
    args.output_dir.mkdir(parents=True, exist_ok=True)
    manifest = args.output_dir / 'manifest.json'
    if manifest.exists():
        parser.error('choose a fresh output directory')
    sources = [Path(__file__).resolve(), IDENTITY, MANAGER, BRIDGE,
               ROOT / 'rulesync/src/rulesync_manager/test/test_recovery_model.py']
    manifest.write_text(json.dumps(dict(
        python=platform.python_version(), scenarios=[asdict(s) for s in SCENARIOS],
        max_states_per_scenario=args.max_states_per_scenario,
        assumptions=['fixed authority during recovery', 'one trusted coordinator/voter',
                     'valid retained durable storage', 'one abstract pending state reply/ACK',
                     'refresh requests not reordered against concurrent/new transactions'],
        exclusions=['ROS executor/transport/clock', 'fsync/torn writes/storage corruption',
                    'continuous instantaneous global agreement', 'motion/geometry',
                    'multi-participant or simultaneous 2-D/3-D domains'],
        decision_history_domain=dict(max_records=2, snapshot_epochs=[0, 1, 2, 3],
                                     kinds=['rule-decision', 'vol-decision'],
                                     decisions=['PREPARE', 'ABORT', 'COMMIT']),
        source_sha256={str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                       for p in sources}), indent=2) + '\n')
    result = run_campaign(args.max_states_per_scenario)
    (args.output_dir / 'results.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({k: result[k] for k in ['status', 'scenarios', 'states',
                           'transitions', 'decision_histories_checked', 'mutations_detected']}))


if __name__ == '__main__':
    main()
