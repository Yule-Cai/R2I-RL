"""Finite recovery graph + actual manager callback bodies; no ROS required."""

import ast
import importlib.util
from pathlib import Path
import sys
import time
from types import SimpleNamespace as N

import pytest

ROOT = Path(__file__).resolve().parents[4]
SOURCE = ROOT / 'scripts/verify_recovery_state_machine.py'
spec = importlib.util.spec_from_file_location('rulesync_recovery_model_test', SOURCE)
model = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = model
spec.loader.exec_module(model)


def test_complete_finite_graph_and_mutation_witnesses():
    result = model.run_campaign()
    assert result['status'] == 'pass' and result['scenarios'] == 12
    assert result['mutations_detected'] == len(model.MUTANTS)
    assert all(r['constructive_convergence_checks'] == r['states']
               for r in result['scenarios_detail'])
    # Observation lag is explicitly reachable, not assumed away.
    assert any(r['observed_ready_bridge_mismatch_states'] > 0
               for r in result['scenarios_detail'])
    assert all(w['failing_event'] and w['reason'] for w in result['mutation_witnesses'])


def test_state_budget_failure_is_not_reported_as_success():
    with pytest.raises(RuntimeError, match='incomplete'):
        model.explore(model.SCENARIOS[0], max_states=1)


def _messages(snapshot):
    return [N(**dict(r, polygon=N(points=[N(x=x, y=y, z=z)
                     for x, y, z in r['polygon']]),
                     valid_from=N(sec=0, nanosec=0),
                     valid_until=N(sec=100, nanosec=0))) for r in snapshot.payload]


def _callbacks():
    # Execute actual unmodified function bodies; imports and ROS node setup are
    # excluded. The replacement round transport has explicit response outcomes.
    tree = ast.parse(model.MANAGER.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef)
               and n.name == 'RuleManager')
    names = {'_reconcile_compare', '_on_bridge_state', '_reconcile_watchdog',
             'on_commit', 'on_remove', '_rule_begin_txn'}
    nodes = [n for n in cls.body if getattr(n, 'name', '') in names]
    scope = dict(recovery_action=model.si.recovery_action, READY=0, time=time,
                 _rule_snapshot_hash=model.si._rule_snapshot_hash,
                 _rule_op_class=model.si._rule_op_class)
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(model.MANAGER), 'exec'), scope)
    return scope


@pytest.mark.parametrize('scenario', model.SCENARIOS)
@pytest.mark.parametrize('bridge_index', [0, 1, 2, 3, None])
@pytest.mark.parametrize('transport', ['ok', 'prepare-reject', 'commit-reject',
                         'empty-prepare', 'empty-commit', 'raise-prepare', 'raise-commit'])
def test_actual_reconcile_callbacks_require_durable_target_evidence(
        scenario, bridge_index, transport):
    fns = _callbacks()
    target = model.SNAPS[scenario.authority]
    target_msgs = _messages(target)
    cp = model.SNAPS[scenario.checkpoint]
    calls = []
    logs = []

    def round_stub(rules, epoch, operation, timeout, txn_id, digest):
        calls.append(operation)
        assert epoch == target.epoch and digest == target.digest
        assert model.si._rule_snapshot_hash(rules) == target.digest
        assert txn_id == f'refresh-e{target.epoch}'
        if transport == 'raise-' + operation:
            raise RuntimeError('injected transport failure')
        if transport == 'empty-' + operation:
            return {}
        if transport == operation + '-reject':
            return {'bridge': 1}
        return {'bridge': 0}

    node = N(_degraded=True, _reconciling=True, _ready_announced=False,
             active={r.rule_id: r for r in _messages(cp)}, committed_epoch=cp.epoch,
             _durable_rule_target=lambda: (target.epoch, target.digest,
                                           {r.rule_id: r for r in target_msgs}),
             get_parameter=lambda name: N(value=1.), _run_round=round_stub,
             log_txn=lambda **kw: logs.append(kw),
             get_logger=lambda: N(info=lambda *a: None, warning=lambda *a: None))
    node._reconcile_compare = lambda res: fns['_reconcile_compare'](node, res)
    response = None
    if bridge_index is not None:
        bridge = model.SNAPS[bridge_index]
        response = N(active_epoch=bridge.epoch, active_hash=bridge.digest,
                     active_rules=_messages(bridge))
    fns['_on_bridge_state'](node, N(result=lambda: response))
    matched = bridge_index == scenario.authority
    expected = response is not None and (matched or transport == 'ok')
    assert node._degraded is not expected
    if expected:
        assert node.committed_epoch == target.epoch
        assert model.si._rule_snapshot_hash(list(node.active.values())) == target.digest
    if response is None or matched:
        assert not calls
    else:
        assert calls[0] == 'prepare'
        if transport in ['prepare-reject', 'empty-prepare', 'raise-prepare']:
            assert calls == ['prepare']


def test_actual_watchdog_missing_response_keeps_degraded():
    fns = _callbacks()
    node = N(_vol_reconcile_watchdog=lambda: None, _reconciling=True,
             _reconcile_t0=-1e30, _degraded=False, _bstate_fails=0,
             committed_epoch=1, get_logger=lambda: N(warning=lambda *a: None),
             log_txn=lambda **kw: None)
    fns['_reconcile_watchdog'](node)
    assert node._degraded and not node._reconciling


@pytest.mark.parametrize('operation,before,after', [('ADD', 0, 1), ('RELAX', 1, 2),
                                                   ('REMOVE', 1, 3)])
@pytest.mark.parametrize('fault', ['none', 'pre-wal', 'post-wal', 'post-apply',
                                  'post-dispatch', 'prepare-reject', 'lost-ack'])
def test_actual_transaction_bodies_at_crash_boundaries(operation, before, after, fault,
                                                     monkeypatch):
    """Actual caller flow with replaced transport/storage, not process/fsync tests."""
    monkeypatch.setenv('RULESYNC_TEST_COMMIT_PAUSE_S', '0')
    fns = _callbacks()
    old, new = model.SNAPS[before], model.SNAPS[after]
    old_msgs, new_msgs = _messages(old), _messages(new)
    wal, checkpoints, events = [], [], []
    participant = N(active=old_msgs, epoch=old.epoch, staged=None, certificate=None)

    class Crash(Exception):
        pass

    def crash_hook(point, *args):
        events.append(point)
        if fault == point:
            raise Crash(point)

    def round_stub(rules, epoch, op, timeout, txn_id, digest):
        events.append(op)
        if op == 'prepare':
            if fault == 'prepare-reject':
                return {'bridge': 1}
            assert model.si._rule_snapshot_hash(rules) == digest
            participant.staged, participant.certificate = rules, (txn_id, epoch, digest)
        elif op == 'commit':
            assert any(r['decision'] == 'COMMIT' and r['epoch'] == epoch
                       and r['target_hash'] == digest for r in wal)
            assert model.bridge_cert(*participant.certificate, txn_id, epoch, digest)
            participant.active, participant.epoch = participant.staged, epoch
            participant.staged = None
            if fault == 'post-apply':
                raise Crash('post-apply/pre-ACK')
            if fault == 'lost-ack':
                return {}
        elif op == 'abort':
            participant.staged = None
        return {'bridge': 0}

    def wal_stub(record):
        events.append('durable-' + record['decision'])
        wal.append(record)

    node = N(_degraded=False, _rule_txn_n=0, committed_epoch=old.epoch,
             active={r.rule_id: r for r in old_msgs},
             staged={r.rule_id: r for r in new_msgs}, staged_epoch=new.epoch,
             get_parameter=lambda name: N(value=1.),
             log_txn=lambda **kw: None, _run_round=round_stub, _wal_write=wal_stub,
             _vol_crash_hook=crash_hook,
             _rule_to_dict=lambda r: next(dict(d) for s in model.SNAPS
                                         for d in s.payload
                                         if d['rule_id'] == r.rule_id and d['epoch'] == r.epoch))
    node._rule_begin_txn = lambda target, epoch: fns['_rule_begin_txn'](node, target, epoch)
    node._save_state = lambda: checkpoints.append((node.committed_epoch,
                                                   [node._rule_to_dict(r)
                                                    for r in node.active.values()]))
    method = 'on_remove' if operation == 'REMOVE' else 'on_commit'
    request = N(epoch=new.epoch, rule_id='p0')
    if fault in ['pre-wal', 'post-wal', 'post-apply', 'post-dispatch']:
        with pytest.raises(Crash):
            fns[method](node, request, N())
    else:
        fns[method](node, request, N())
    committed = fault not in ['pre-wal', 'prepare-reject']
    cp_epoch, cp_rows = checkpoints[-1] if checkpoints else (old.epoch, list(old.payload))
    restored = model.si.select_durable_target(cp_epoch, cp_rows, wal, 'rule-decision')
    expected = new if committed else old
    assert restored == (expected.epoch, list(expected.payload))
    if 'commit' in events:
        assert events.index('durable-COMMIT') < events.index('commit')
    if fault == 'post-apply':
        assert participant.epoch == new.epoch and not checkpoints
    if fault == 'lost-ack':
        assert node._degraded and participant.epoch == new.epoch
    if fault == 'none':
        assert not node._degraded and participant.epoch == new.epoch
