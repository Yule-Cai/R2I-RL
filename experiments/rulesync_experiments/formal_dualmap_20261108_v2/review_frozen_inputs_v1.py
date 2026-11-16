"""Independent local cartesian-matrix, lineage and spatial-input review."""
import itertools
import json
from pathlib import Path

from contract_v1 import ARMS,COHORT,load,sha


def main():
    repo=Path(__file__).resolve().parents[3];folder=repo/'output/formal_dualmap_20261108_v2'
    root=folder/'inputs';cohort=root/'formal_dualmap_20261108_v2'
    digest='bb749f5d886ef5a0009966ce4d180b3c58e2c1d503213a918cedd94534dfef20'
    m=load(cohort/'manifest.json',digest)
    old=repo/'output/input_requalification_20261108_v1/pair_v4_disposable'
    qualified=json.loads((repo/'output/input_requalification_20261108_v1/pair_local_review_v4.json').read_text())
    pin=json.loads((old/'input_requalification_20261108_v1/pair_pin_v4.json').read_text());old_jobs={j['tag']:j for j in pin['jobs']}
    assert {(r['scene'],r['route']) for r in qualified['rows'] if r['qualified']}=={(c['scene'],c['route']) for c in m['cases'].values()}
    assert m['seeds']['main']==list(range(52501,52506)) and m['seeds']['secondary']==list(range(52501,52504)) and m['speeds']==[.10,.20,.25]
    assert not set(m['seeds']['main']).intersection(m['seeds']['qualification'])
    for n,h in m['sources_sha256'].items():
        assert sha(root/n)==h
        local=Path(__file__).parent/Path(n).name
        if n.startswith('formal_dualmap_20261108_v2/tools/'):assert sha(local)==h,('Local source differs',n)
    compared=[]
    metadata=['planning_only','cohort_id','formal_eligible','formal_scope','qualification','original_qualified_case_sha256','qualification_report_sha256']
    for cid,c in m['cases'].items():
        j=old_jobs[c['qualification_tag']];a=json.loads((old/j['case']).read_text());b=json.loads((root/c['path']).read_text())
        assert sha(old/j['case'])==c['parent_sha256']==j['case_sha256'] and sha(root/c['path'])==c['sha256']
        assert b['cohort_id']==COHORT and b['planning_only'] is False and b['formal_eligible'] is True
        assert b['qualification_report_sha256']==sha(repo/'output/input_requalification_20261108_v1/pair_local_review_v4.json')
        for k in metadata:a.pop(k,None);b.pop(k,None)
        assert a==b,('Spatial task changed',cid);compared.append(cid)
    assets={}
    for scene,s in m['scenes'].items():
        for n,h in s['files_sha256'].items():assert sha(root/n)==h==sha(old/n),('Qualified scene differs',n)
        migration=json.loads((root/s['prepared']/'migration_manifest.json').read_text())
        assets.update({s['asset_root']+'/'+n:h for n,h in migration['asset_files_sha256'].items()})
    assert assets==m['asset_files_sha256']
    expected=set()
    for cid in m['cases']:
        for arm in ARMS:
            seeds=range(52501,52506) if arm in ('full','native_async') else range(52501,52504)
            expected.update((cid,arm,speed,seed) for speed,seed in itertools.product([.10,.20,.25],seeds))
    actual={(r['case_id'],r['arm'],r['speed'],r['seed']) for r in m['rows']}
    assert len(m['rows'])==len(actual)==len(expected)==2970 and actual==expected
    assert {(r['case_id'],r['arm'],r['speed'],r['seed']) for r in m['preflight_rows']}=={(next(iter(m['cases'])),a,.20,52431) for a in ARMS}
    assert {r['arm'] for r in m['rows'][:6]}==set(ARMS) and {r['seed'] for r in m['rows'][:6]}=={52501}
    reference=repo/'output/input_requalification_20261108_v1/common_disposable/input_requalification_20261108_v1/common_arm_v1/runs/common_full_v1/position_1/manifest.json'
    assert m['runtime_core_sha256']==json.loads(reference.read_text())['source_sha256']
    report=dict(passed=True,manifest_sha256=digest,script_sha256=sha(Path(__file__)),layouts=len(m['scenes']),cases=len(compared),
        planned=len(actual),matrix_per_arm=m['matrix_per_arm'],qualified_spatial_inputs_unchanged=True,all_qualified_inputs_included=True,
        exact_cartesian_coverage=True,independent_seeds=True,runtime_core_binding_unchanged=True,asset_references=len(assets),
        scope='Frozen input/code review, not physical preflight or formal mission result')
    (folder/'input_independent_review_v1.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))


if __name__=='__main__':main()
