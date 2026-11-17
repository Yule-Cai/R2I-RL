"""The actual relay handler must forward zero-delay replies without timer ticks."""
import ast
import asyncio
import base64
import copy
import hashlib
import heapq
from pathlib import Path
from types import SimpleNamespace
import time
import unittest

SOURCE=Path(__file__).resolve().parent/'source/snapshot_response_proxy_v1.py'
tree=ast.parse(SOURCE.read_text())
cls=next(n for n in tree.body if isinstance(n,ast.ClassDef))
method=next(n for n in cls.body if isinstance(n,ast.AsyncFunctionDef) and n.name=='on_request')
namespace=dict(time=time,copy=copy,hashlib=hashlib,base64=base64,heapq=heapq,Future=asyncio.Future,
               serialize_message=lambda message:b'captured-test-response')
exec(compile(ast.fix_missing_locations(ast.Module(body=[method],type_ignores=[])),str(SOURCE),'exec'),namespace)


class FakeRelay:
    on_request=namespace['on_request']
    def __init__(self,action):
        self.seq=0;self.schedule=dict(maximum_requests=100)
        self.cache={};self.pending=[];self.unanswered=[];self.fatal=None
        self.action=action;self.events=[];self.bound=[]
        async def call(request):return SimpleNamespace(epoch=2,active_hash='unchanged-content',degraded=False)
        self.client=SimpleNamespace(call_async=call)
    def event(self,name,**fields):
        self.events.append((name,fields));return time.monotonic_ns()
    def spec_for(self,*args):return dict(action=self.action,delay_s=0.)
    def before_delivery(self,seq,response):self.bound.append(seq)


class RelayTests(unittest.IsolatedAsyncioTestCase):
    async def test_forward_finishes_without_any_delivery_timer(self):
        relay=FakeRelay('forward')
        response=await asyncio.wait_for(relay.on_request(None,None),.1)
        self.assertEqual(response.epoch,2)
        self.assertEqual(relay.pending,[])
        self.assertEqual(relay.bound,[1])
        self.assertEqual([name for name,_ in relay.events][-2:],['dispatch','callback_return_ready'])

    async def test_suppression_still_withholds_the_actual_reply(self):
        relay=FakeRelay('suppress')
        with self.assertRaises(asyncio.TimeoutError):
            await asyncio.wait_for(relay.on_request(None,None),.02)
        self.assertEqual(len(relay.unanswered),1)
        self.assertEqual(relay.pending,[])
        self.assertFalse(any(name=='dispatch' for name,_ in relay.events))


if __name__=='__main__':unittest.main()
