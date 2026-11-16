#!/usr/bin/env python3
"""Owned-container overnight workflow. Formal cohort starts only after pilot QA."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import time

ROOT=Path('/home/gamer/Projects/rulesync-submission-20261012')
CONTAINER='rulesync-nav2-pilot-20261012'
VERIFY='verification03'
FORMAL='formal48'
IMAGE='localhost/rulesync-nav2-verified-env:20261012'


def state(phase,**kw):
    (ROOT/'overnight_status.json').write_text(json.dumps(dict(
        phase=phase,updated_wall_ns=time.time_ns(),**kw),indent=2)+'\n')


def inside(command,check=True):
    setup='source /opt/ros/lyrical/setup.bash; source /work/install/setup.bash; '
    setup+='export PYTHONPATH=/experiment:/work/scripts:$PYTHONPATH; '
    return subprocess.run(['podman','exec',CONTAINER,'bash','-lc',setup+command],check=check)


def main():
    if (ROOT/VERIFY).exists() or (ROOT/FORMAL).exists():
        raise RuntimeError('fresh output directories required; do not duplicate or overwrite')
    state('verification_running',formal_started=False)
    verify=inside('python3 /experiment/benchmark_nav2_combination.py --verify-matrix '
        '--output /experiment/'+VERIFY,check=False)
    flag=ROOT/VERIFY/'verification.json'
    if verify.returncode or not flag.exists() or not json.loads(flag.read_text())['passed']:
        state('verification_failed',formal_started=False,returncode=verify.returncode)
        return 1
    audits=[json.loads((ROOT/VERIFY/f'trial_{i:02}'/'independent_audit.json').read_text()) for i in range(1,9)]
    if not all(a['audit_passed'] for a in audits): raise RuntimeError('independent pilot audit failed')
    state('freezing_runtime',formal_started=False)
    versions=subprocess.check_output(['podman','exec',CONTAINER,'dpkg-query','-W',
        'ros-lyrical-nav2-*','ros-lyrical-ros-gz-*','python3-shapely'],text=True)
    (ROOT/'runtime_packages.tsv').write_text(versions)
    subprocess.run(['podman','commit',CONTAINER,IMAGE],check=True)
    info=json.loads(subprocess.check_output(['podman','image','inspect',IMAGE],text=True))
    (ROOT/'runtime_image.json').write_text(json.dumps(info,indent=2))
    state('formal_running',formal_started=True,planned=48,verification=VERIFY,
        image_id=info[0]['Id'])
    run=inside('export RULESYNC_NAV2_DOMAIN_BASE=170; '
        'python3 /experiment/benchmark_nav2_combination.py --output /experiment/'+FORMAL,check=False)
    state('formal_auditing',formal_started=True,returncode=run.returncode)
    audit=inside('python3 /experiment/audit_nav2_combination.py /experiment/'+FORMAL,check=False)
    state('archiving',formal_started=True,returncode=run.returncode,audit_returncode=audit.returncode)
    archive=ROOT.parent/'rulesync-nav2-submission-20261012.tar.gz'
    subprocess.run(['tar','-czf',str(archive),'-C',str(ROOT.parent),ROOT.name],check=True)
    sha=hashlib.file_digest(archive.open('rb'),'sha256').hexdigest()
    state('acquisition_complete',formal_started=True,returncode=run.returncode,
        audit_returncode=audit.returncode,archive=str(archive),archive_sha256=sha,
        next='Download; verify digest; independent Mac re-audit; update main paper/figures/PDF.')
    return 0


if __name__=='__main__':
    parser=argparse.ArgumentParser(); parser.add_argument('--verification',default=VERIFY)
    args=parser.parse_args(); VERIFY=args.verification
    try: raise SystemExit(main())
    except Exception as exc:
        state('workflow_failed',error=repr(exc)); raise
