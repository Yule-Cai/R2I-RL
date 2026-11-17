"""Wait for resources, then execute the frozen cohort and full evidence checks."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parent


def write_status(phase, **fields):
    p=ROOT/'pipeline_status.tmp'
    p.write_text(json.dumps(dict(phase=phase, host_monotonic_ns=time.monotonic_ns(), **fields),indent=2)+'\n')
    p.replace(ROOT/'pipeline_status.json')


def resources():
    available=int(next(l.split()[1] for l in Path('/proc/meminfo').read_text().splitlines() if l.startswith('MemAvailable:')))*1024
    names=subprocess.check_output(['ps','-eo','comm='],text=True).splitlines()
    graphics_present=any(n.strip().lower() in ('rdr2.exe','playrdr2.exe') for n in names)
    jobs=subprocess.check_output(['podman','ps','--format','{{.Names}}'],text=True).splitlines()
    competing=any(n.startswith('rulesync-') for n in jobs)
    free=shutil.disk_usage(ROOT).free
    return dict(available_memory_gib=round(available/2**30,3), free_disk_gib=round(free/2**30,3),
                observed_graphics_program_running=graphics_present, competing_rulesync_job=competing,
                ready=available>=5*2**30 and free>=30*2**30 and not graphics_present and not competing)


def execute(name,args):
    write_status('executing',stage=name)
    with (ROOT/(name+'.log')).open('x') as stream:
        subprocess.run([sys.executable,*args],cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT,check=True)


def main():
    try:
        manifest=json.loads((ROOT/'manifest.json').read_text())
        for name,digest in manifest['inputs_sha256'].items():
            assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,name
        ready_count=0
        while ready_count<2:
            state=resources();ready_count=ready_count+1 if state['ready'] else 0
            write_status('waiting_for_resources',resource_check=state,consecutive_ready_checks=ready_count)
            if ready_count<2:time.sleep(30)
        (ROOT/'resource_start.json').write_text(json.dumps(state,indent=2)+'\n')
        execute('driver',['campaign.py'])
        assert json.loads((ROOT/'status.json').read_text())['phase']=='complete'
        execute('decode',['decode_all.py'])
        execute('audit',['collect_and_audit.py'])
        execute('aggregate',['aggregate_campaign.py'])
        artifact='RuleSync_Final_Moving_36_Evidence'
        execute('export',['export_review_artifact.py','--output',artifact])
        execute('verification',[artifact+'/analysis/verify_artifact.py','--root',artifact,
                               '--output','artifact_verification_workstation.json'])
        write_status('complete',planned=36,aggregate='aggregate.json',artifact=artifact)
    except Exception as exc:
        write_status('inspection_required',error=repr(exc))
        raise


if __name__=='__main__':main()
