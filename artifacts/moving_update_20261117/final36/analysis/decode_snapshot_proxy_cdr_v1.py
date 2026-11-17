"""Decode retained CDR serializations independently; padding bytes may differ.

Never call serializations captured in application code original wire packets.
Compare all decoded response fields, retaining raw hashes and full CDR strings.
"""
import argparse
import base64
import hashlib
import json
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.convert import message_to_ordereddict
from rulesync_interfaces.srv import ListRules
from rulesync3d.ruleguard import snapshot_hash, rule_snapshot_items


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--log',action='append',help='Relative log path; repeat for real guard/relay logs')
    a=p.parse_args()
    decoded={}
    count=0
    for name in a.log or ['backend.jsonl','proxy/proxy.jsonl','client.jsonl']:
        for line in (a.root/name).read_text().splitlines():
            row=json.loads(line)
            if 'cdr_base64' not in row:continue
            raw=base64.b64decode(row['cdr_base64'],validate=True)
            digest=hashlib.sha256(raw).hexdigest()
            assert digest==row['cdr_sha256']
            message=deserialize_message(raw,ListRules.Response)
            assert message.active_hash==snapshot_hash(rule_snapshot_items(message.active))
            assert int(message.epoch)==row['epoch'] and message.active_hash==row['active_hash']
            canonical=json.dumps(message_to_ordereddict(message),sort_keys=True,separators=(',',':'),allow_nan=False)
            value=dict(fields=json.loads(canonical),canonical_sha256=hashlib.sha256(canonical.encode()).hexdigest())
            assert digest not in decoded or decoded[digest]==value
            decoded[digest]=value
            count+=1
    result=dict(scope='Independent ROS2 deserialization of retained application serializations;not wire packet capture',
                serialized_records=count,unique_cdr_serializations=len(decoded),all_snapshot_hashes_verified=True,
                decoder_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),decoded=decoded)
    with a.output.open('x') as out:out.write(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='decoded'},indent=2))


if __name__=='__main__':main()
