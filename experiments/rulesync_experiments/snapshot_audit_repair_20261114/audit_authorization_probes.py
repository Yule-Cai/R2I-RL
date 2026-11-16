"""Independently verify full bytes and actual message/guard/recovery witnesses."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import tarfile
from types import SimpleNamespace

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / 'rulesync3d'))
from rulesync3d.authorization import verify_authorization


def objectify(value):
    if isinstance(value, dict):
        return SimpleNamespace(**{k: objectify(v) for k, v in value.items()})
    if isinstance(value, list):
        return [objectify(v) for v in value]
    return value


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', type=Path, required=True)
    args = parser.parse_args()
    archive = args.input / 'two_probes_raw.tar.gz'
    receipt = json.loads((args.input / 'receipt.json').read_text())
    assert hashlib.sha256(archive.read_bytes()).hexdigest() == receipt['sha256']
    with tarfile.open(archive) as tar:
        inventory = json.load(tar.extractfile('files.json'))
    retained = {}
    seen = set()
    with tarfile.open(archive, 'r|gz') as tar:
        for member in tar:
            if member.name == 'files.json':
                continue
            assert member.isfile() and member.name in inventory
            data = tar.extractfile(member).read()
            assert dict(bytes=len(data),sha256=hashlib.sha256(data).hexdigest()) == inventory[member.name]
            seen.add(member.name)
            if member.name.endswith(('.json', '.jsonl')):
                retained[member.name] = data
    assert seen == set(inventory)
    pin_name = 'authorization_probe_20261114/source_pin.json'
    assert hashlib.sha256(retained[pin_name]).hexdigest() == 'f1a72d12c39b8340560a1c5ac8cc66827466e39f29ddfeed3f3132168668f061'
    pin = json.loads(retained[pin_name])
    for name,digest in pin['files'].items():
        assert inventory['authorization_probe_20261114/'+name]['sha256'] == digest
    reviewed=[]
    for kind in ['stale_identity','late_authorization']:
        root = 'mechpairs_runs/authprobe_'+kind+'_20261114_v1/mech_out_of_order_full/hssd_102815835__route_02_s20_seed52601/position_1'
        load = lambda name: json.loads(retained[root+'/'+name])
        result,manifest,injected = load('result.json'),load('manifest.json'),load('injected_authorization.json')
        assert result['result']=='PASS' and result['source_unchanged']
        assert not result['formal_denominator'] and result['independent_goal_error_m'] <= .20
        for i,(original,digest) in enumerate(manifest['source_sha256'].items()):
            assert inventory[root+f'/source/{i:02d}_'+Path(original).name]['sha256']==digest
        events=[json.loads(line) for line in retained[root+'/events.jsonl'].splitlines()]
        captured=[json.loads(line) for line in retained[root+'/authorization_messages.jsonl'].splitlines()]
        sent=next(e for e in events if e['kind']=='actual_authorization_probe_sent')
        finished=next(e for e in events if e['kind']=='actual_authorization_probe_rejected')
        fresh=[x['message'] for x in captured if x['receipt_ns']<sent['receipt_ns'] and x['message']['epoch']==2][-1]
        historical=[x['message'] for x in captured if x['receipt_ns']<sent['receipt_ns'] and x['message']['epoch']==1][0]
        for message in [injected,fresh,historical]:
            m=objectify(message)
            assert verify_authorization(m.path,m.path_digest,m.token_id,m.epoch,m.snapshot_hash)[0]
        assert injected['epoch']==1 and fresh['epoch']==2
        assert any(x['message']==injected and x['receipt_ns']>=sent['receipt_ns'] for x in captured)
        if kind=='stale_identity':
            assert injected['path']==fresh['path'] and injected['path_digest']==fresh['path_digest']
            assert injected['snapshot_hash']==historical['snapshot_hash']
        else:
            assert injected==historical
        before=[e for e in events if e['kind']=='gate_verdict' and e['receipt_ns']<sent['receipt_ns']]
        assert before[-1]['execute'] and before[-1]['epoch']==2 and before[-1]['token_id']==fresh['token_id']
        window=[e for e in events if sent['receipt_ns']<=e['receipt_ns']<=finished['receipt_ns']]
        rejection=next(e for e in window if e['kind']=='guard_state' and e['reason']=='authorization-identity-mismatch')
        assert rejection['authority'][0]==2 and rejection['execution'][0]==0 and rejection['state']==2
        assert not any(e['kind']=='monitor_path' and e['points']>0 for e in window)
        after=[e for e in events if e['receipt_ns']>finished['receipt_ns']]
        gate=next(e for e in after if e['kind']=='gate_verdict')
        release=next(e for e in after if e['kind']=='task_release')
        assert gate['execute'] and gate['epoch']==2 and gate['receipt_ns']<release['receipt_ns']
        assert release['epoch']==2 and release['identity'][0]==2
        reviewed.append(dict(probe_kind=kind,result='PASS',source_and_content_verified=True,
                             actual_published_envelope_observed=True,
                             positive_control_epoch=2,injected_epoch=1,
                             guard_rejection_reason=rejection['reason'],
                             nonempty_forward_during_probe=0,fresh_readmission_before_recovery=True,
                             independent_goal_error_m=result['independent_goal_error_m'],
                             scope='Single-case DEV authorization receiver test before recovery execution; not service-response reordering or a hardware experiment'))
    report=dict(trials=2,verified_files=len(seen),raw_archive_sha256=receipt['sha256'],
                formal_denominator=False,rows=reviewed)
    path=args.input/'independent_review_v1.json'
    assert not path.exists()
    path.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
