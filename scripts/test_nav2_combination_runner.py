"""Read-query retry policy tests without ROS, extracted from the executed driver."""
import ast
import copy
from pathlib import Path
from types import SimpleNamespace
import unittest


def extracted(name, namespace):
    tree=ast.parse(Path(__file__).with_name('benchmark_nav2_combination.py').read_text())
    node=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name==name)
    exec(compile(ast.Module(body=[node],type_ignores=[]),'runner','exec'),namespace)
    return namespace[name]


class RetryPolicy(unittest.TestCase):
    def setup_query(self, fail_count=1):
        calls=[]; removed=[]; events=[]; tick=[0.]
        def now(): tick[0]+=.1; return tick[0]
        def future(node,pending,seconds):
            self.assertGreater(seconds,0.)
            if len(calls)<=fail_count: raise TimeoutError('lost read')
            return 'response'
        client=SimpleNamespace(srv_name='/read',
            call_async=lambda req:calls.append(req) or len(calls),
            remove_pending_request=lambda f:removed.append(f))
        namespace={'time':SimpleNamespace(monotonic=now),
            'base':SimpleNamespace(future=future)}
        query=extracted('read_query',namespace)
        node=SimpleNamespace(event=lambda *a,**kw:events.append((a,kw)))
        return query,node,client,calls,removed,events

    def test_lost_query_retried_and_removed(self):
        q,n,c,calls,removed,events=self.setup_query()
        self.assertEqual(q(n,c,'request'),'response')
        self.assertEqual(calls,['request','request']); self.assertEqual(removed,[1])
        self.assertEqual(len(events),1)

    def test_success_not_repeated(self):
        q,n,c,calls,removed,_=self.setup_query(0)
        self.assertEqual(q(n,c,'request'),'response')
        self.assertEqual(len(calls),1); self.assertEqual(removed,[])

    def test_retry_budget_expires(self):
        q,n,c,calls,removed,_=self.setup_query(100)
        with self.assertRaisesRegex(TimeoutError,'read query unavailable'):
            q(n,c,'request',deadline_s=.8)
        self.assertEqual(len(calls),len(removed))

    def test_mutations_use_nonretry_call(self):
        calls=[]
        namespace={'base':SimpleNamespace(future=lambda n,f,t:f)}
        call=extracted('call',namespace)
        c=SimpleNamespace(call_async=lambda req:calls.append(req) or 'reply')
        self.assertEqual(call(None,c,'commit'),'reply')
        self.assertEqual(calls,['commit'])
        tree=ast.parse(Path(__file__).with_name('benchmark_nav2_combination.py').read_text())
        fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='commit_rule')
        self.assertNotIn('read_query',ast.unparse(fn))

    def test_mirrored_odometry_frame(self):
        tree=ast.parse(Path(__file__).with_name('benchmark_nav2_combination.py').read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Probe')
        fn=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='on_odom')
        def stamped(): return SimpleNamespace(header=SimpleNamespace(),pose=None)
        ns={'time':SimpleNamespace(monotonic=lambda:1.),'copy':copy,'PoseStamped':stamped,
            'base':SimpleNamespace(SPAWN=(2.,-.5))}
        exec(compile(ast.Module(body=[fn],type_ignores=[]),'runner','exec'),ns)
        outputs=[]
        node=SimpleNamespace(direction=-1,pose_pub=SimpleNamespace(publish=outputs.append),
            get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:0)))
        pose=SimpleNamespace(position=SimpleNamespace(x=.2,y=.1,z=0.),
            orientation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.))
        ns['on_odom'](node,SimpleNamespace(pose=SimpleNamespace(pose=pose)))
        self.assertEqual(node.odom,[1.8,-.6]); self.assertEqual(outputs[0].header.frame_id,'map')
        q=outputs[0].pose.orientation; self.assertEqual((q.x,q.y,q.z,q.w),(0.,0.,1.,0.))
        self.assertEqual(pose.orientation.w,1.,'input pose must not be mutated')

    def test_explicit_startup_once_after_discovery(self):
        events=[]
        class Request:
            STARTUP=0
            def __init__(self,command): self.command=command
        manager=SimpleNamespace(wait_for_service=lambda **kw:events.append('manager discovered') or True)
        client=SimpleNamespace(wait_for_service=lambda **kw:True)
        queries=iter([1,1,1,3,3,3])
        namespace={'ManageLifecycleNodes':SimpleNamespace(Request=Request),
            'base':SimpleNamespace(GetState=SimpleNamespace(Request=lambda:None)),
            'read_query':lambda *a:SimpleNamespace(current_state=SimpleNamespace(id=next(queries))),
            'spin_for':lambda n,t:events.append(('settle',t)),
            'call':lambda n,c,r:events.append(('startup',r.command)) or SimpleNamespace(success=True),
            'time':SimpleNamespace(monotonic=lambda:0.)}
        start=extracted('start_navigation',namespace)
        node=SimpleNamespace(create_client=lambda *a:manager,
            states={n:client for n in ['planner','controller','monitor']})
        self.assertEqual(start(node),{'planner':3,'controller':3,'monitor':3})
        self.assertEqual(events,['manager discovered',('settle',3.),('startup',0)])


if __name__=='__main__': unittest.main()
