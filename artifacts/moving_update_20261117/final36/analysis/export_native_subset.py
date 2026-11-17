"""Export original native records needed to recheck the command/release chain.

Records are copied byte-for-byte. Original full traces remain untouched and
their SHA-256 and trace metadata are linked in a separate subset manifest.
"""
import argparse
import hashlib
import json
from pathlib import Path

TOPICS={'/cmd_vel','/rulesync/execution_authorization','/rulepatch/executable_path'}
INIT={'ros2:rcl_node_init','ros2:rcl_publisher_init','ros2:rmw_publisher_init',
      'ros2:rcl_subscription_init','ros2:rmw_subscription_init'}


def export(root,output):
    source=root/'native_events.jsonl'
    publishers={};subscriptions={}
    for line in source.open():
        row=json.loads(line);p=row['payload'];pid=row['context']['vpid']
        if row['name']=='ros2:rcl_publisher_init' and p['topic_name'] in TOPICS:
            publishers[(pid,p['publisher_handle'])]=p['rmw_publisher_handle']
        if row['name']=='ros2:rcl_subscription_init' and p['topic_name'] in TOPICS:
            subscriptions[(pid,p['rmw_subscription_handle'])]=p
    rmw_publishers={(pid,rmw) for (pid,_),rmw in publishers.items()}
    output.mkdir(parents=True,exist_ok=False)
    digest=hashlib.sha256();selected=0;total=0
    target=output/'native_relevant.jsonl'
    with source.open('rb') as stream,target.open('xb') as destination:
        for line in stream:
            digest.update(line);total+=1
            row=json.loads(line);p=row['payload'];pid=row['context']['vpid'];name=row['name']
            keep=(name in INIT or
                name=='ros2:rcl_publish' and (pid,p['publisher_handle']) in publishers or
                name=='ros2:rmw_publish' and (pid,p['rmw_publisher_handle']) in rmw_publishers or
                name=='ros2:rmw_take' and (pid,p['rmw_subscription_handle']) in subscriptions)
            if keep:destination.write(line);selected+=1
    trace=root/'trace_result.json'
    assert total==json.loads(trace.read_text())['trace']['events']
    manifest=dict(scope='Unchanged original native records for command and authorization topics; full-trace hashes retained',
        topics=sorted(TOPICS),full_native_sha256=digest.hexdigest(),full_native_events=total,
        selected_events=selected,selected_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),
        original_trace_result_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        exporter_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (output/'native_subset_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(dict(selected_events=selected,full_events=total)))


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();export(args.root,args.output)
