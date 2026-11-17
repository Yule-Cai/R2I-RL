"""Serial isolated campaign. Preserve every attempted run, including failures."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import time

ROOT = Path(__file__).resolve().parent
PARENT = Path('/home/gamer/Projects/rulesync-iotj-environments-20261028')
WORK = Path('/home/gamer/Projects/rulesync-review-20261010')
REFERENCE = Path('/home/gamer/Projects/rulesync-submission-20261012')
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def write(path, obj):
    with path.open('x') as stream:
        json.dump(obj, stream, indent=2)
        stream.write('\n')


def status(**fields):
    path = ROOT/'status.tmp'
    path.write_text(json.dumps(dict(updated_at=time.strftime('%Y-%m-%dT%H:%M:%S%z'), **fields), indent=2)+'\n')
    path.replace(ROOT/'status.json')


def verify_inputs(manifest):
    for name, digest in manifest['inputs_sha256'].items():
        assert sha(ROOT/name) == digest, ('Changed input', name)


def image_command(manifest):
    return ['podman','run','--rm','--network','none',
            '-v',str(WORK)+':/work:ro','-v',str(REFERENCE)+':/reference:ro',
            '-v',str(PARENT)+':/experiment:ro','--entrypoint','/bin/bash',manifest['image']]


def preflight(manifest):
    verify_inputs(manifest)
    image = subprocess.check_output(['podman','image','inspect',manifest['image'],'--format','{{.Id}}'],text=True).strip()
    assert image.removeprefix('sha256:') == manifest['image_id']
    names = subprocess.check_output(['podman','ps','--format','{{.Names}}'],text=True).splitlines()
    assert not any(name.startswith('rulesync-') for name in names), ('Other RuleSync job running', names)
    assert shutil.disk_usage(ROOT).free >= 30*2**30, 'Insufficient campaign disk budget'
    mem = int(next(x.split()[1] for x in Path('/proc/meminfo').read_text().splitlines() if x.startswith('MemAvailable:')))
    assert mem >= 5*1024**2
    # Reuse exactly the already audited runtime; a new public-code manager must
    # not silently replace the source used by the causal moving-update pair.
    code = '''import hashlib,json,pathlib
p=pathlib.Path('/experiment/channel_dev_20261104/runs/identity_exposure_full_dev_v1/mission/manifest.json')
m=json.loads(p.read_text()); checks={k:hashlib.sha256(pathlib.Path(k).read_bytes()).hexdigest()==v for k,v in m['source_sha256'].items()}
assert all(checks.values()),checks
print(json.dumps(dict(checked_runtime_files=checks,source_sha256=m['source_sha256'])))'''
    cli = image_command(manifest)+['-lc','source /opt/ros/lyrical/setup.bash && source /work/install/setup.bash && python3 -c '+shlex.quote(code)]
    runtime = json.loads(subprocess.check_output(cli,text=True,timeout=40))
    scene = PARENT/'office_prepared_v3_native'
    migration = json.loads((scene/'migration_manifest.json').read_text())
    assets = PARENT/'assets/office'
    verified = {}
    for name, digest in migration['asset_files_sha256'].items():
        path = assets/name
        assert sha(path) == digest, ('Asset mismatch', str(path))
        verified[str(path)] = digest
    runtime.update(scene_sha256={str(p):sha(p) for p in [scene/'office.sdf',scene/'map.json',scene/'migration_manifest.json']},
                   asset_files_sha256=verified, image_id=image,
                   manifest_sha256=sha(ROOT/'manifest.json'),
                   source_sha256_launcher=sha(Path(__file__)))
    write(ROOT/'preflight.json', runtime)
    return runtime


def launch(manifest, run):
    verify_inputs(manifest)
    assert shutil.disk_usage(ROOT).free >= 15*2**30, 'Disk reserve reached; preserve records and stop'
    root = ROOT/'runs'/run['id']
    root.mkdir(parents=True,exist_ok=False)
    inside = '/supplement/runs/'+run['id']+'/trace'
    args = ['python3','/supplement/source/trace_wrapper_v2.py','--output',inside,'--command-witness','--',
        '/supplement/source/runner_channel_v2.py','--scene-id','office',
        '--scene','/experiment/office_prepared_v3_native','--case','/supplement/'+run['case']['path'],
        '--speed',str(run['speed_mps']),'--seed',str(run['seed']),'--ros-domain',str(run['domain']),
        '--concurrency','1','--output',inside+'/mission','--stage','recovery',
        '--render-mode','gpu','--controller-profile','legacy','--geometry-profile','standard']
    script = ('source /opt/ros/lyrical/setup.bash && source /work/install/setup.bash && '
        'export PYTHONPATH=/supplement/source:/experiment/source:/reference:/work/scripts:$PYTHONPATH '
        'LTTNG_HOME=/tmp/rulesync_ras_supplement && '+shlex.join(args))
    name = 'rulesync-ras36-'+run['id'].replace('_','-')
    command = ['podman','run','--rm','--name',name,'--network','none',
        '--cpus','3','--memory','3g','--memory-swap','3g','--pids-limit','512','--timeout','1200',
        '--device','/dev/dri/renderD128',
        '-v',str(WORK)+':/work:ro','-v',str(REFERENCE)+':/reference:ro',
        '-v',str(PARENT)+':/experiment:ro','-v',str(PARENT/'assets')+':/assets:ro',
        '-v',str(ROOT)+':/supplement',
        '--env','ROS_DOMAIN_ID='+str(run['domain']),'--env','GZ_IP=127.0.0.1',
        '--env','RULESYNC_CHANNEL_SCHEDULE=/supplement/'+run['schedule'],
        '--entrypoint','/bin/bash',manifest['image'],'-lc',script]
    write(root/'launch.json',dict(run=run,manifest_sha256=sha(ROOT/'manifest.json'),
        command=command,started_ns=time.monotonic_ns(),wall_time=time.strftime('%Y-%m-%dT%H:%M:%S%z')))
    started = time.monotonic()
    with (root/'host.log').open('x') as stream:
        result = subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT,timeout=1250)
    record = dict(returncode=result.returncode,wall_s=time.monotonic()-started,
                  ended_ns=time.monotonic_ns())
    for name, path in [('trace','trace/trace_result.json'),('task','trace/mission/result.json')]:
        record[name] = json.loads((root/path).read_text()) if (root/path).exists() else None
    record['inputs_unchanged'] = all(sha(ROOT/p)==digest for p,digest in manifest['inputs_sha256'].items())
    write(root/'closed.json',record)
    assert record['inputs_unchanged']
    trace = record['trace']
    # A task failure is an outcome. Missing/discarded native evidence is an
    # instrumentation failure and suspends further trials for inspection.
    assert trace and 'error' not in trace and 'decode_error' not in trace, record
    assert trace.get('command_witness_exit_code') == 0, record
    assert trace['trace']['discarded_messages'] == [], record
    return record


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--preflight-only',action='store_true')
    parser.add_argument('--limit',type=int,default=36)
    args = parser.parse_args()
    manifest = json.loads((ROOT/'manifest.json').read_text())
    assert 1 <= args.limit <= 36
    with (ROOT/'.execution.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        if not (ROOT/'preflight.json').exists():
            preflight(manifest)
        else:
            assert json.loads((ROOT/'preflight.json').read_text())['manifest_sha256'] == sha(ROOT/'manifest.json')
            verify_inputs(manifest)
        if args.preflight_only:
            print(json.dumps(dict(preflight='passed',planned_runs=36)),flush=True)
            return
        (ROOT/'runs').mkdir(exist_ok=True)
        completed = []
        try:
            for run in manifest['runs'][:args.limit]:
                closed = ROOT/'runs'/run['id']/'closed.json'
                if closed.exists():
                    completed.append(run['id'])
                    continue
                assert not (ROOT/'runs'/run['id']).exists(), 'Incomplete attempt retained; inspect before resuming'
                if (ROOT/'STOP_AFTER_CURRENT_RUN').exists():
                    status(phase='stopped_by_marker',completed=completed,planned=36)
                    return
                status(phase='running',current=run['id'],completed=completed,planned=36)
                print(json.dumps(dict(event='start',run=run['id'])),flush=True)
                record = launch(manifest,run)
                completed.append(run['id'])
                print(json.dumps(dict(event='closed',run=run['id'],returncode=record['returncode'],
                    task=record['task'],wall_s=record['wall_s'])),flush=True)
            status(phase='complete' if len(completed)==36 else 'bounded_limit_reached',completed=completed,planned=36)
        except Exception as exc:
            status(phase='inspection_required',completed=completed,planned=36,error=repr(exc))
            raise


if __name__ == '__main__':
    main()
