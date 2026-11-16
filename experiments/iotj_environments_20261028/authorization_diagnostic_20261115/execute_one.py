"""Execute one predeclared diagnostic setting exactly once; never retry a row."""
import argparse,hashlib,json,subprocess,sys,time
from pathlib import Path
HERE=Path(__file__).resolve().parent;ROOT=HERE.parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 p=argparse.ArgumentParser();p.add_argument('--index',type=int,choices=[0,1,2],required=True);p.add_argument('--manifest-sha',required=True);a=p.parse_args()
 manifest=HERE/'manifest.json';assert sha(manifest)==a.manifest_sha
 m=json.loads(manifest.read_text());r=m['rows'][a.index]
 if a.index:
  prior=HERE/f'run_{a.index-1}.json';assert prior.exists(), 'qualify prior setting first'
  rec=json.loads(prior.read_text());assert rec.get('independently_verified'), 'prior independent audit required'
 started=HERE/f'started_{a.index}.json';assert not started.exists(),'no automatic retries'
 started.write_text(json.dumps(dict(spec=r,started_wall_ns=time.time_ns(),manifest_sha256=a.manifest_sha),indent=2)+'\n')
 tag='authprobe_diagnostic_20261115_v1_'+r['probe_kind']
 cmd=[sys.executable,str(HERE/'run_diagnostic.py'),'--scene',r['scene'],'--case-id',r['case_id'],'--speed',str(r['speed']),'--seed',str(r['seed']),'--ros-domain','230','--tag',tag,'--mech-mode','out_of_order','--monitor-arm','full','--probe-kind',r['probe_kind'],'--source-pin-sha',r['source_pin_sha256']]
 with (HERE/f'launch_{a.index}.log').open('x') as f:proc=subprocess.run(cmd,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT)
 folder=ROOT/'mechpairs_runs'/tag/'mech_out_of_order_full'/f"{r['case_id']}_s20_seed{r['seed']}"/'position_1'
 raw=json.loads((folder/'result.json').read_text()) if (folder/'result.json').exists() else None
 record=dict(spec=r,manifest_sha256=a.manifest_sha,command=cmd,exit_code=proc.returncode,output=str(folder.relative_to(ROOT)),raw_result=raw,finished_wall_ns=time.time_ns(),independently_verified=False)
 (HERE/f'run_{a.index}.json').write_text(json.dumps(record,indent=2)+'\n')
 print(json.dumps(dict(index=a.index,result=(raw or {}).get('result'),error=(raw or {}).get('error'),outdir=str(folder))),flush=True)
if __name__=='__main__':main()
