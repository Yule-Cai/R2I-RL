#!/usr/bin/env python3
"""Matched real-Nav2 experiment with a deliberately strong combination arm.

Both arms share RuleSync's durable manager/bridge as the rule-update backend,
continuous gate, real Nav2, mask application barrier, proactive cancellation,
freshness checks, geometric monitoring, and one velocity exit. Thus this is an
execution-binding comparison, NOT an independent comparison of transaction
algorithms. Geometry-only disables the existing monitor's in-flight identity
test, as in DynamicVolume; it keeps initial token validation. Successful strong
baseline behavior is an expected and reportable outcome.
"""
import argparse
import copy
import hashlib
import importlib
import json
import math
import os
from pathlib import Path
import platform
import random
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from geometry_msgs.msg import Point32, PoseStamped
from nav_msgs.msg import Path as PathMsg, OccupancyGrid
from nav2_msgs.msg import CostmapFilterInfo
from nav2_msgs.srv import GetCostmap, ManageLifecycleNodes
from nav2_msgs.action import ComputePathToPose, FollowPath
from std_msgs.msg import String
from rulesync_interfaces.msg import RuleGuardState, RuleSpec
from rulesync_interfaces.srv import ProposeRule, CommitRule, ListRules
import yaml

import nav2_combination_pilot as base
from gazebo_world_pose import instrument_robot
from rulesync3d.authorization import path_digest

NEW = [(-.3,-.8),(.7,-.8),(.7,-.2),(-.3,-.2)]
SOURCES = ['rulesync_manager.manager_node','rulesync_manager.snapshot_identity',
    'rulesync_manager.vote_coordinator','rulesync_manager.validator',
    'rulesync_nav2_bridge.bridge_node','rulesync_guard.guard_node','topopatch_ros.gate_node',
    'rulesync3d.ruleguard','rulesync3d.gate2d','rulesync3d.authorization']


class Probe(base.Probe):
    def __init__(self,root,trial):
        self.trial=trial; self.direction=trial['direction']; self.blocked=True
        self.goal_handle=None; self.cancel_future=None; self.executable=None; self.guard=None
        self.gate_status=None; self.mask_queue=[]; self.expected_epoch=0
        self.last_pose_at=0.; self.last_guard_at=0.; self.mask_generation=None
        super().__init__(root)
        self.cmd_pub=self.create_publisher(base.TwistStamped,'/cmd_vel_nav',50)
        self.pose_pub=self.create_publisher(PoseStamped,'/ruleguard/pose',30)
        self.candidate_pub=self.create_publisher(PathMsg,'/rulepatch/repaired_path',10)
        self.create_subscription(PathMsg,'/rulepatch/executable_path',self.on_executable,20)
        self.create_subscription(RuleGuardState,'/rulesync/ruleguard_state',self.on_guard,30)
        self.create_subscription(String,'/rulepatch/gate_status',self.on_gate,20)
        self.create_subscription(base.TwistStamped,'/cmd_vel_controller',self.relay_cmd,50)
        self.create_subscription(OccupancyGrid,'/source/mask',self.queue_mask,self.qos)
        self.create_subscription(CostmapFilterInfo,'/source/filter_info',self.queue_info,self.qos)
        self.create_timer(.01,self.flush_masks)
        self.create_timer(.05,self.watchdog)
        self.propose=self.create_client(ProposeRule,'/rulesync/propose_rule')
        self.commit=self.create_client(CommitRule,'/rulesync/commit_rule')
        self.list=self.create_client(ListRules,'/rulesync/list_rules')
        self.delay=.2 if trial['condition']=='delayed_mask' else 0.
        self.pending_info=None
    def on_odom(self,m):
        # The differential-drive odometry origin follows the initial heading.
        # Align both mirrored starts with their goal rather than adding a 180deg
        # initial turn to only one arm/direction.
        sign=self.direction
        self.odom=[sign*m.pose.pose.position.x+base.SPAWN[0],
            sign*m.pose.pose.position.y+base.SPAWN[1]]
        self.last_pose_at=time.monotonic()
        p=PoseStamped(); p.header.frame_id='map'; p.header.stamp=self.get_clock().now().to_msg()
        p.pose=copy.deepcopy(m.pose.pose); p.pose.position.x=self.odom[0]; p.pose.position.y=self.odom[1]
        if sign<0:
            q=p.pose.orientation; x,y,z,w=q.x,q.y,q.z,q.w
            q.x=-y; q.y=x; q.z=w; q.w=-z
        self.pose_pub.publish(p)
    def on_guard(self,m):
        self.guard=m; self.last_guard_at=time.monotonic()
        self.event('guard_state',state=int(m.state),reason=m.reason,
            authority=[int(m.authority_epoch),m.authority_hash],execution=[int(m.execution_epoch),m.execution_hash])
        if m.state==RuleGuardState.HOLD: self.hold('monitor:'+m.reason)
    def on_executable(self,m):
        if not m.poses: self.hold('empty-executable-path')
        else:
            self.executable=m
            self.event('monitor_path',digest=path_digest(m),points=len(m.poses))
    def on_gate(self,m):
        self.gate_status=json.loads(m.data); self.event('gate_verdict',**self.gate_status)
    def hold(self,reason):
        if not self.blocked: self.event('interlock_hold',reason=reason)
        self.blocked=True
        if self.goal_handle is not None and self.cancel_future is None:
            self.cancel_future=self.goal_handle.cancel_goal_async()
            self.event('cancel_requested',reason=reason)
    def relay_cmd(self,m):
        out=copy.deepcopy(m)
        if self.blocked: out.twist.linear.x=0.; out.twist.angular.z=0.
        self.cmd_pub.publish(out)
    def watchdog(self):
        now=time.monotonic()
        if now-self.last_pose_at>.5 or now-self.last_guard_at>.5: self.hold('adapter-freshness')
        if self.blocked:
            m=base.TwistStamped(); m.header.stamp=self.get_clock().now().to_msg(); self.cmd_pub.publish(m)
    def queue_mask(self,m):
        self.mask_queue.append((time.monotonic()+self.delay,copy.deepcopy(m)))
    def queue_info(self,m): self.pending_info=copy.deepcopy(m)
    def flush_masks(self):
        now=time.monotonic()
        while self.mask_queue and self.mask_queue[0][0]<=now:
            _,m=self.mask_queue.pop(0)
            source=np.asarray(m.data).reshape(m.info.height,m.info.width)>50
            inflated=np.zeros_like(source); cells=math.ceil(.40/m.info.resolution)
            h,w=source.shape
            for dy in range(-cells,cells+1):
                for dx in range(-cells,cells+1):
                    if math.hypot(dx,dy)*m.info.resolution>.40+1e-8: continue
                    a=max(0,dy); b=min(h,h+dy); c=max(0,dx); d=min(w,w+dx)
                    inflated[a:b,c:d] |= source[a-dy:b-dy,c-dx:d-dx]
            m.data=(inflated.astype(np.int8)*100).ravel().tolist()
            m.header.stamp=self.get_clock().now().to_msg(); m.info.map_load_time=m.header.stamp
            self.mask_pub.publish(m)
            info=CostmapFilterInfo(); info.header=m.header; info.type=0
            info.filter_mask_topic='/pilot/mask'; info.multiplier=1.; self.info_pub.publish(info)
            self.mask_generation=dict(hash=hashlib.sha256(source.tobytes()).hexdigest(),
                stamp_ns=base.stamp_ns(m.header.stamp))
            (self.root/f"mask_{self.mask_generation['stamp_ns']}.json").write_text(json.dumps(dict(
                generation=self.mask_generation,resolution=m.info.resolution,width=w,height=h,
                origin=[m.info.origin.position.x,m.info.origin.position.y],
                raw_occupied=np.flatnonzero(source).tolist(),occupied=np.flatnonzero(inflated).tolist())))
            self.event('mask_released',**self.mask_generation)
    def publish_map(self):
        # Known static obstacle, shared by both arms; no map-estimation claim.
        super().publish_maps(False)
        grid=OccupancyGrid(); grid.header.frame_id='map'; grid.header.stamp=self.get_clock().now().to_msg()
        grid.info.resolution=.05; grid.info.width=160; grid.info.height=160
        grid.info.origin.position.x=-4.; grid.info.origin.position.y=-4.; grid.info.origin.orientation.w=1.
        grid.info.map_load_time=grid.header.stamp; grid.data=[0]*25600
        for j in range(160):
            for i in range(160):
                x=-4+(i+.5)*.05; y=-4+(j+.5)*.05
                if -.3<=x<=.3 and 1.7<=y<=2.3: grid.data[j*160+i]=100
        self.map_pub.publish(grid)


def spin_for(node,seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end: rclpy.spin_once(node,timeout_sec=.01)


def call(node,client,request): return base.future(node,client.call_async(request),35.)


def read_query(node,client,request,deadline_s=35.):
    """Bounded read-only retries; never repeat a mutation or action request."""
    deadline=time.monotonic()+deadline_s
    while time.monotonic()<deadline:
        pending=client.call_async(request)
        try: return base.future(node,pending,min(2.,deadline-time.monotonic()))
        except TimeoutError:
            client.remove_pending_request(pending)
            node.event('read_query_timeout',service=client.srv_name)
    raise TimeoutError('read query unavailable: '+client.srv_name)


def start_navigation(node):
    """Explicit startup after discovery, once only; assert real active states."""
    manager=node.create_client(ManageLifecycleNodes,'/lifecycle_manager_pilot/manage_nodes')
    if not manager.wait_for_service(timeout_sec=30.):
        raise TimeoutError('lifecycle manager service unavailable')
    for name,client in node.states.items():
        if not client.wait_for_service(timeout_sec=30.): raise TimeoutError(name+' state missing')
        state=int(read_query(node,client,base.GetState.Request()).current_state.id)
        assert state==1, (name,'expected fresh unconfigured node',state)
    # Let the manager's own lifecycle service endpoints complete discovery.
    # This is before exposure and applies identically to both arms.
    spin_for(node,3.)
    request=ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.STARTUP)
    response=call(node,manager,request)  # mutation: never blindly repeat STARTUP
    assert response.success,'lifecycle startup refused'
    deadline=time.monotonic()+50.
    while True:
        states={n:int(read_query(node,c,base.GetState.Request()).current_state.id)
            for n,c in node.states.items()}
        if all(s==3 for s in states.values()): return states
        if time.monotonic()>deadline: raise TimeoutError('lifecycle not active: '+str(states))
        spin_for(node,.1)


def ready_manager(node,epoch=None):
    deadline=time.monotonic()+40.
    for c in [node.list,node.propose,node.commit]:
        if not c.wait_for_service(timeout_sec=20.): raise TimeoutError(c.srv_name)
    while time.monotonic()<deadline:
        response=read_query(node,node.list,ListRules.Request())
        if not response.degraded and (epoch is None or response.epoch==epoch): return response
        spin_for(node,.1)
    raise TimeoutError('manager agreement unavailable')


def commit_rule(node,epoch):
    r=RuleSpec(); r.rule_id='nav2_combination_zone'; r.epoch=epoch
    r.rule_type=RuleSpec.KEEP_OUT; r.severity=RuleSpec.ENFORCED; r.frame_id='map'
    r.valid_until.sec=3600
    region=base.RULE if epoch==1 else NEW
    r.polygon.points=[Point32(x=float(x),y=float(y)) for x,y in region]
    p=call(node,node.propose,ProposeRule.Request(rule=r)); assert p.accepted,p.reason
    c=call(node,node.commit,CommitRule.Request(epoch=epoch)); assert c.committed,c.reason
    response=ready_manager(node,epoch)
    node.event('commit_reply',epoch=epoch,identity=[int(response.epoch),response.active_hash])
    return response


def applied(node,epoch):
    center=(0.,.6) if epoch==1 else (.2,-.5)
    outside=(.2,-.5) if epoch==1 else (0.,.6)
    deadline=time.monotonic()+30.
    for c in node.costmaps.values():
        if not c.wait_for_service(timeout_sec=15.): raise TimeoutError('costmap unavailable')
    while time.monotonic()<deadline:
        maps={n:read_query(node,c,GetCostmap.Request()).map for n,c in node.costmaps.items()}
        values={n:[base.cost_at(m,*center),base.cost_at(m,*outside)] for n,m in maps.items()}
        fresh=(node.mask_generation is not None and all(
            base.stamp_ns(m.metadata.update_time)>=node.mask_generation['stamp_ns'] for m in maps.values()))
        if fresh and all(v[0]==254 and v[1]!=254 for v in values.values()):
            for name,m in maps.items():
                p=m.metadata
                (node.root/f'costmap_epoch{epoch}_{name}.json').write_text(json.dumps(dict(
                    epoch=epoch,frame=m.header.frame_id,resolution=p.resolution,
                    width=p.size_x,height=p.size_y,origin=[p.origin.position.x,p.origin.position.y],
                    update_time_ns=base.stamp_ns(p.update_time),data=list(m.data))))
            node.event('corresponding_costmaps_applied',epoch=epoch,values=values,
                mask=node.mask_generation); return
        spin_for(node,.05)
    raise TimeoutError('corresponding mask absent from actual costmaps: '+str(values))


def plan(node):
    if not node.planner.wait_for_server(timeout_sec=10.): raise TimeoutError('planner missing')
    req=ComputePathToPose.Goal(); req.planner_id='GridBased'; req.goal.header.frame_id='map'
    req.goal.header.stamp=node.get_clock().now().to_msg()
    req.goal.pose.position.x=base.GOAL[0]; req.goal.pose.position.y=base.GOAL[1]
    req.goal.pose.orientation.w=1. if node.direction>0 else 0.
    req.goal.pose.orientation.z=0. if node.direction>0 else 1.
    h=base.future(node,node.planner.send_goal_async(req)); assert h.accepted
    r=base.future(node,h.get_result_async()).result; assert r.error_code==0,r.error_msg
    node.event('planner_result',xy=[[p.pose.position.x,p.pose.position.y] for p in r.path.poses])
    return r.path


def admit(node,path,epoch,expect=True):
    node.executable=None; node.gate_status=None; node.candidate_pub.publish(path)
    base.wait(node,lambda:node.gate_status is not None,10.)
    assert bool(node.gate_status['execute'])==expect,node.gate_status
    if not expect: return
    base.wait(node,lambda:node.executable is not None and node.guard is not None
        and node.guard.state!=RuleGuardState.HOLD and node.guard.execution_epoch==epoch,10.)
    assert path_digest(node.executable)==path_digest(path)


def follow(node,path,epoch):
    assert node.blocked
    node.goal_handle=None; node.cancel_future=None
    if not node.controller.wait_for_server(timeout_sec=10.): raise TimeoutError('controller missing')
    g=FollowPath.Goal(); g.path=path; g.controller_id='FollowPath'; g.goal_checker_id='GoalChecker'
    g.progress_checker_id='ProgressChecker'; g.path_handler_id='PathHandler'
    handle=base.future(node,node.controller.send_goal_async(g)); assert handle.accepted
    node.goal_handle=handle; node.blocked=False
    node.event('task_release',epoch=epoch,digest=path_digest(path),
        identity=[int(node.guard.execution_epoch),node.guard.execution_hash])
    return handle.get_result_async()


def freeze(root):
    sources=[Path(__file__),Path(__file__).with_name('audit_nav2_combination.py'),Path(base.__file__),Path(instrument_robot.__code__.co_filename),
        Path('/work/scripts/benchmark_dynamic_rule_update.py'),Path('/work/scripts/benchmark_rule_churn.py')]
    sources += [Path(importlib.import_module(n).__file__) for n in SOURCES]
    sim=Path('/opt/ros/lyrical/share/nav2_minimal_tb3_sim')
    sources += [sim/'worlds/tb3_empty_world.sdf.xacro',sim/'urdf/gz_waffle.sdf.xacro',
        sim/'urdf/turtlebot3_waffle.urdf',sim/'configs/turtlebot3_waffle_bridge.yaml']
    hashes={}
    for i,p in enumerate(sources):
        target=root/'source'/f'{i:02}_{p.name}'; target.parent.mkdir(exist_ok=True)
        shutil.copy2(p,target); hashes[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
    return hashes


def run_trial(output,trial,pilot):
    base.SPAWN=(-2.*trial['direction'],-.5); base.GOAL=(2.*trial['direction'],-.5)
    root=output; root.mkdir(parents=True,exist_ok=False)
    hashes=freeze(root)
    params=base.parameters()
    params['lifecycle_manager_pilot']['ros__parameters']['autostart']=False
    params['controller_server']['ros__parameters']['GoalChecker']['xy_goal_tolerance']=.10
    for scope in ['global_costmap','local_costmap']:
        p=params[scope][scope]['ros__parameters']
        p['filters']=['keepout_filter','filter_inflation']
        p['filter_inflation']=dict(plugin='nav2_costmap_2d::InflationLayer',inflation_radius=.8,cost_scaling_factor=3.)
    (root/'params.yaml').write_text(yaml.safe_dump(params))
    # Fresh domain per independent process trial prevents stale discovery from
    # the preceding trial. A restart within one trial retains its domain.
    ordinal=int(root.name.split('_')[-1]) if root.name.startswith('trial_') else 1
    domain=int(os.environ.get('RULESYNC_NAV2_DOMAIN_BASE','120'))+ordinal
    assert 0<=domain<=232
    env=os.environ.copy(); env.update(ROS_DOMAIN_ID=str(domain),GZ_PARTITION='rulesync_nav2_'+root.name,
        RULESYNC_CHURN_FOLDER=str(root),RULESYNC_DYNAMIC_ARM=trial['arm'],
        RULESYNC_TEST_POSE2D=f'{base.SPAWN[0]},{base.SPAWN[1]}',
        RULESYNC_TEST_GOAL2D=f'{base.GOAL[0]},{base.GOAL[1]}')
    env.pop('ROS_DISCOVERY_SERVER',None); os.environ.update(env)
    sim=Path('/opt/ros/lyrical/share/nav2_minimal_tb3_sim')
    env['GZ_SIM_RESOURCE_PATH']=str(sim/'models')+':'+str(sim.parent)
    manifest=dict(trial=trial,pilot=pilot,source_sha256=hashes,platform=platform.platform(),
        parameters_sha256=hashlib.sha256((root/'params.yaml').read_bytes()).hexdigest(),
        shared_backend='RuleSync durable transaction backend in BOTH arms; execution-binding comparison',
        ros_domain_id=domain,initial_map_yaw_rad=0. if trial['direction']>0 else math.pi,
        world='controlled ground plane with static 0.6m box at (0,2); known matching static map',
        body_envelope_xy=[[-.21,-.17],[.11,-.17],[.11,.17],[-.21,.17]],
        max_world_sim_gap_s=.10,
        proposal_mask_inflation_m=.4,gate_rho_m=base.RHO,update_trigger_x=-1.6*trial['direction'],
        max_follow_wall_s=100,pose_and_guard_timeout_s=.5)
    (root/'manifest.json').write_text(json.dumps(manifest,indent=2))
    procs=base.Processes(root,env); node=None; result=dict(result='FAIL',**trial,pilot=pilot,phase='startup')
    role=Path('/work/scripts/benchmark_dynamic_rule_update.py')
    def backend():
        procs.start('bridge'+str(len(procs.items)),['ros2','run','rulesync_nav2_bridge','bridge','--ros-args',
            '-p','auto_replan:=false','-p','auto_rollback:=false',
            '-r','/rulesync/keepout_mask:=/source/mask','-r','/keepout_costmap_filter_info:=/source/filter_info'],.3)
        procs.start('manager'+str(len(procs.items)),[sys.executable,str(role),'--role','manager','--ros-args',
            '-p','state_path:='+str(root/'state.json'),'-p','wal_path:='+str(root/'wal.jsonl'),
            '-p','txn_log_path:='+str(root/'txn.log'),'-p','prepare_timeout_s:=12.'],.3)
    try:
        robot=instrument_robot(sim/'urdf/gz_waffle.sdf.xacro',root,env); world=root/'world.sdf'
        subprocess.run(['xacro','-o',str(world),'headless:=True',str(sim/'worlds/tb3_empty_world.sdf.xacro')],check=True,env=env)
        tree=ET.parse(world); model=ET.SubElement(tree.getroot().find('world'),'model',{'name':'static_obstacle'})
        ET.SubElement(model,'static').text='true'; ET.SubElement(model,'pose').text='0 2 0.5 0 0 0'
        link=ET.SubElement(model,'link',{'name':'body'})
        for kind in ['collision','visual']:
            shape=ET.SubElement(link,kind,{'name':kind}); geo=ET.SubElement(shape,'geometry')
            ET.SubElement(ET.SubElement(geo,'box'),'size').text='0.6 0.6 1.0'
        tree.write(world,encoding='unicode',xml_declaration=True)
        procs.start('gazebo',['gz','sim','-r','-s',str(world)],8.)
        procs.start('spawn',['ros2','launch','nav2_minimal_tb3_sim','spawn_tb3.launch.py',
            'robot_name:=turtlebot3_waffle',f'robot_sdf:={robot}',f'x_pose:={base.SPAWN[0]}',
            'y_pose:=-0.5','z_pose:=0.01',f"yaw:={0. if trial['direction']>0 else math.pi}"],1.)
        procs.start('world_bridge',['ros2','run','ros_gz_bridge','parameter_bridge',
            '/rulesync_test/world_pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'])
        procs.start('robot_state',['ros2','run','robot_state_publisher','robot_state_publisher','--ros-args',
            '-p','use_sim_time:=true','-p','robot_description:='+ (sim/'urdf/turtlebot3_waffle.urdf').read_text()])
        procs.start('map_tf',['ros2','run','tf2_ros','static_transform_publisher','--frame-id','map',
            '--child-frame-id','odom','--x',str(base.SPAWN[0]),'--y','-0.5','--z','0',
            '--yaw',str(0. if trial['direction']>0 else math.pi)])
        rclpy.init(); node=Probe(root,trial); node.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
        base.wait(node,lambda:node.last_world is not None and node.scan_count>0 and node.odom is not None,40.)
        p=node.last_world['xyz']; assert math.hypot(p[0]-base.SPAWN[0],p[1]+.5)<.02
        q=node.last_world['quaternion']; yaw=math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]**2+q[2]**2))
        expected=0. if trial['direction']>0 else math.pi
        assert abs(math.atan2(math.sin(yaw-expected),math.cos(yaw-expected)))<.03
        node.publish_map(); backend()
        procs.start('gate',['ros2','run','topopatch_ros','gate_node','--ros-args','-p','rho:=0.3'])
        procs.start('guard',[sys.executable,str(role),'--role','guard','--ros-args',
            '-p','rho:=0.3','-p','rho_warn:=0.3','-p','rho_release:=0.05',
            '-p','snapshot_poll_s:=0.05','-p','hold_publish_s:=0.05','-p','use_sim_time:=true'])
        ready_manager(node); commit_rule(node,1)
        for package,exe in [('nav2_planner','planner_server'),('nav2_controller','controller_server'),
                            ('nav2_collision_monitor','collision_monitor')]:
            cmd=['ros2','run',package,exe,'--ros-args','--params-file',str(root/'params.yaml')]
            if exe=='controller_server': cmd+=['-r','cmd_vel:=/cmd_vel_controller']
            procs.start(exe,cmd,.3)
        procs.start('lifecycle',['ros2','run','nav2_lifecycle_manager','lifecycle_manager','--ros-args',
            '-r','__node:=lifecycle_manager_pilot','--params-file',str(root/'params.yaml')])
        states=start_navigation(node)
        node.event('active_states',states=states)
        pubs=node.get_publishers_info_by_topic('/cmd_vel')
        assert len(pubs)==1 and pubs[0].node_name=='collision_monitor'
        applied(node,1); result['phase']='initial_follow'
        path=plan(node); admit(node,path,1); node.active=True
        initial=follow(node,path,1)
        base.wait(node,lambda:node.odom[0]*trial['direction']>=-1.6,30.)
        assert not node.blocked,'unexpected HOLD before update trigger'
        node.event('update_requested',xy=node.odom)
        # Strong baseline interlock: neither arm waits for propagation to stop.
        node.hold('proactive-update-request'); result['phase']='update'
        commit_rule(node,2)
        base.wait(node,initial.done,15.)
        node.event('old_action_terminal',status=int(initial.result().status))
        if trial['condition']=='restart':
            node.event('backend_restart_begin')
            for name,p,_ in procs.items:
                if name.startswith(('manager','bridge')) and p.poll() is None:
                    os.killpg(p.pid,9); p.wait()
            backend(); ready_manager(node,2); node.event('backend_restart_ready')
        applied(node,2); spin_for(node,.15)
        if trial['condition']=='invalid_candidate':
            bad=PathMsg(); bad.header.frame_id='map'; bad.header.stamp=node.get_clock().now().to_msg()
            for x,y in [tuple(node.odom),base.GOAL]:
                p=PoseStamped(); p.header=bad.header; p.pose.position.x=x; p.pose.position.y=y; p.pose.orientation.w=1.; bad.poses.append(p)
            admit(node,bad,2,False); node.event('injected_candidate_denied')
        result['phase']='recovery_follow'
        path=plan(node); admit(node,path,2); final=follow(node,path,2)
        followed=base.future(node,final,100.)
        node.active=False; spin_for(node,.3)
        assert followed.status==4 and followed.result.error_code==0,followed.result.error_msg
        assert not node.world_errors,node.world_errors
        p=node.last_world['xyz']; error=math.hypot(p[0]-base.GOAL[0],p[1]-base.GOAL[1])
        assert error<=.20,error
        assert all(v>0 for v in node.nonzero.values())
        result.update(result='PASS',independent_goal_error_m=error,nonzero_commands=node.nonzero,
            states=states,scan_messages=node.scan_count)
    except (Exception,KeyboardInterrupt) as exc:
        result['error']=repr(exc)
        if isinstance(exc,KeyboardInterrupt): result['interrupted']=True
    finally:
        try:
            if node:
                if rclpy.ok():
                    node.hold('terminal'); spin_for(node,.1); node.event('terminal',**result)
                node.events.close(); node.world.close(); node.destroy_node(); rclpy.try_shutdown()
        except Exception as exc: result['cleanup_error']=repr(exc)
        finally: procs.close()
        result['source_unchanged']=all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in hashes.items())
        (root/'result.json').write_text(json.dumps(result,indent=2)+'\n'); print(json.dumps(result),flush=True)
    if result.get('interrupted'): raise SystemExit(130)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--pilot',action='store_true'); parser.add_argument('--verify-matrix',action='store_true')
    parser.add_argument('--arm',choices=['full','geometry_only'],default='full')
    parser.add_argument('--condition',choices=['normal','delayed_mask','invalid_candidate','restart'],default='normal')
    parser.add_argument('--direction',type=int,choices=[-1,1],default=1)
    args=parser.parse_args()
    if args.pilot:
        r=run_trial(args.output,dict(arm=args.arm,condition=args.condition,direction=args.direction,rep=0),True)
        return 0 if r['result']=='PASS' else 1
    if args.verify_matrix:
        args.output.mkdir(parents=True,exist_ok=False)
        rows=[dict(arm=a,condition=c,direction=(-1 if i%2 else 1),rep=0)
            for a in ['full','geometry_only'] for i,c in enumerate(['normal','delayed_mask','invalid_candidate','restart'])]
        (args.output/'pilot_schedule.json').write_text(json.dumps(rows,indent=2))
        ok=True
        for i,t in enumerate(rows,1):
            r=run_trial(args.output/f'trial_{i:02}',t,True)
            check=subprocess.run([sys.executable,str(Path(__file__).with_name('audit_nav2_combination.py')),
                str(args.output/f'trial_{i:02}'),'--trial'],stdout=(args.output/f'audit_{i:02}.log').open('w'))
            ok &= r['result']=='PASS' and check.returncode==0
        (args.output/'verification.json').write_text(json.dumps(dict(passed=ok,pilot=True,attempted=8)))
        return 0 if ok else 1
    args.output.mkdir(parents=True,exist_ok=False)
    rows=[dict(arm=a,condition=c,direction=d,rep=r) for a in ['full','geometry_only']
        for c in ['normal','delayed_mask','invalid_candidate','restart'] for d in [-1,1] for r in range(1,4)]
    random.Random(20261012).shuffle(rows)
    manifest=dict(seed=20261012,planned=48,schedule=rows,predeclared_wall_ns=time.time_ns(),
        runner_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        scope='strong existing-module composition with SHARED durable backend; execution-binding comparison',
        no_retries=True,pilots_excluded=True)
    (args.output/'cohort_manifest.json').write_text(json.dumps(manifest,indent=2))
    for i,t in enumerate(rows,1):
        r=run_trial(args.output/f'trial_{i:02}',t,False)
        with (args.output/'attempts.jsonl').open('a') as f: f.write(json.dumps(dict(attempt=i,**r))+'\n')
    return 0


if __name__=='__main__': raise SystemExit(main())
