"""Freeze the previously declared input qualification, not mission outcomes."""
import json
from pathlib import Path
import subprocess
import sys

from contract_v1 import ARMS,COHORT,atomic,sha

ROOT=Path('/home/gamer/Projects/rulesync-iotj-environments-20261028')
OUT=ROOT/'formal_dualmap_20261108_v2'
STAGE=ROOT/'input_requalification_20261108_v1'


def main():
    assert not (OUT/'manifest.json').exists() and not (OUT/'pin.json').exists()
    assert not subprocess.check_output(['podman','ps','--filter','name=rulesync','--format','{{.Names}}'],text=True).strip()
    old=json.loads((ROOT/'iotj_scene_build/arm_queue_status.json').read_text())
    assert old['phase']=='paused' and old['active']==0 and (ROOT/'iotj_scene_build/maintenance_hold_v2.json').exists()
    evidence=OUT/'evidence';r=json.loads((evidence/'pair_local_review_v4.json').read_text())
    assert (r['planned'],r['reviewed'],r['qualified'],r['qualified_layouts'])==(46,46,45,23)
    assert json.loads((evidence/'common_six_arm_local_review_v1.json').read_text())['independent_checks_passed']==6
    qpin=STAGE/'pair_pin_v4.json';assert sha(qpin)==r['pin_sha256']
    jobs={j['tag']:j for j in json.loads(qpin.read_text())['jobs']}
    selected=sorted([x for x in r['rows'] if x['qualified']],key=lambda x:(x['scene'],x['route']))
    sys.path.insert(0,str(ROOT));import run_native_furnished_recovery_dev_v2 as base
    assert subprocess.check_output(['podman','image','inspect',base.IMAGE,'--format','{{.Id}}'],text=True).strip()==base.IMAGE_ID
    names=['contract_v1.py','launch_v1.py','queue_v1.py','scene_runner_rulesync_formal_v1.py','scene_runner_native_formal_v1.py','freeze_v1.py','build_runners_v1.py']
    sources={str((OUT/'tools'/n).relative_to(ROOT)):sha(OUT/'tools'/n) for n in names}
    sources.update({n:sha(ROOT/n) for n in ['run_native_furnished_recovery_dev_v2.py','scene_runner_generalization_v1.py']})
    cases={};scenes={};assets={}
    for row in selected:
        j=jobs[row['tag']];original=ROOT/j['case'];assert sha(original)==j['case_sha256']
        c=json.loads(original.read_text());assert c['planning_only'] is True
        c.update(planning_only=False,cohort_id=COHORT,formal_eligible=True,formal_scope='dualmap_planning_input_qualified',
            qualification='Independent P0/P2 planning-input qualification; not selected on closed-loop success',
            original_qualified_case_sha256=j['case_sha256'],qualification_report_sha256=sha(evidence/'pair_local_review_v4.json'))
        task=OUT/'cases'/row['scene']/original.name;task.parent.mkdir(parents=True,exist_ok=True)
        with task.open('x') as f:json.dump(c,f,indent=2);f.write('\n')
        cid=row['scene']+'__'+row['route'];assert cid not in cases
        cases[cid]=dict(scene=row['scene'],route=row['route'],name=task.name,path=str(task.relative_to(ROOT)),sha256=sha(task),parent_sha256=j['case_sha256'],qualification_tag=row['tag'])
        if row['scene'] not in scenes:
            prepared=STAGE/'floor_support_v1/scenes'/row['scene']/'nav_v1'
            files={str((prepared/n).relative_to(ROOT)):sha(prepared/n) for n in ['map.json','localization_map.json','migration_manifest.json','projection.json','repair_map_manifest.json',row['scene']+'.sdf']}
            migration=json.loads((prepared/'migration_manifest.json').read_text())
            for n,h in migration['asset_files_sha256'].items():
                rel='iotj_scene_build/scenes/'+row['scene']+'/assets/'+n
                assert sha(ROOT/rel)==h,('Asset changed',rel);assets[rel]=h
            scenes[row['scene']]=dict(prepared=str(prepared.relative_to(ROOT)),files_sha256=files,asset_root='iotj_scene_build/scenes/'+row['scene']+'/assets')
    seed_main=[52501,52502,52503,52504,52505];seed_secondary=seed_main[:3];speeds=[.10,.20,.25]
    import hashlib
    def make(cid,arm,speed,seed,phase):
        c=cases[cid];fingerprint=hashlib.sha256(json.dumps(dict(phase=phase,arm=arm,case=c['sha256'],scene=scenes[c['scene']]['files_sha256'],sources=sources,image=base.IMAGE_ID,speed=speed,seed=seed),sort_keys=True).encode()).hexdigest()
        key=f"{arm}|{c['scene']}|{c['route']}|{speed:.2f}|{seed}|{fingerprint[:16]}"
        return dict(key=key,id=fingerprint,arm=arm,case_id=cid,scene=c['scene'],route=c['route'],speed=speed,seed=seed,phase=phase)
    rows=[]
    for ci,cid in enumerate(cases):
        for si,speed in enumerate(speeds):
            for ki,seed in enumerate(seed_main):
                arms=[a for a in ARMS if a in ('full','native_async') or seed in seed_secondary]
                k=(ci+si+ki)%len(arms);arms=arms[k:]+arms[:k]
                rows += [make(cid,a,speed,seed,'formal') for a in arms]
    preflight=[make(next(iter(cases)),a,.20,52431,'preflight') for a in ARMS]
    totals={a:sum(r['arm']==a for r in rows) for a in ARMS}
    assert len(rows)==2970 and len(cases)==45 and len(scenes)==23
    reference=STAGE/'common_arm_v1/runs/common_full_v1/position_1/manifest.json'
    m=dict(version=1,cohort=COHORT,matrix_total=2970,matrix_per_arm=totals,scenes=scenes,cases=cases,rows=rows,preflight_rows=preflight,
        seeds=dict(main=seed_main,secondary=seed_secondary,qualification=[52401,52411,52421,52431]),speeds=speeds,
        sources_sha256=sources,runtime_core_sha256=json.loads(reference.read_text())['source_sha256'],asset_files_sha256=assets,
        image_id=base.IMAGE_ID,image_tag=base.IMAGE,runners=dict(rulesync=str((OUT/'tools/scene_runner_rulesync_formal_v1.py').relative_to(ROOT)),native=str((OUT/'tools/scene_runner_native_formal_v1.py').relative_to(ROOT))),
        evidence_sha256={str(p.relative_to(ROOT)):sha(p) for p in evidence.glob('*.json')},
        selection='All 45 independently planning-qualified cases in 23 layouts; no selection on motion outcome; original 34-layout source pool/exclusions retained',
        scheduling='Case barrier; speed then seed then cyclic arm rotation by case/speed/seed index, fixed before formal outcomes',
        resources=dict(workers=3,cpu_per_container=3,memory_gib_per_container=3,container_swap=False,min_available_memory_gib=6,min_disk_free_gib=20),
        raw_retention='Keep every original trial and failure; closed cases archived without deleting originals; no retries/replacements',
        independent_audit='All planned rows including failed prefixes; absent/unexercised epoch unassessed, never zero-filled',
        next_stage='No automatic mechanism/3D release; complete independent audit required',
        scope='Same-host ground navigation in HSSD mesh simulation; held-out sensor seeds, not unseen real buildings or physical robots')
    atomic(OUT/'manifest.json',m);digest=sha(OUT/'manifest.json')
    atomic(OUT/'pin.json',dict(cohort=COHORT,manifest=str(OUT/'manifest.json'),sha256=digest))
    print(json.dumps(dict(sha256=digest,cases=len(cases),layouts=len(scenes),rows=len(rows),matrix_per_arm=totals,asset_checks=len(assets))))


if __name__=='__main__':main()
