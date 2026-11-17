"""Warehouse DEV integration, reusing the frozen RuleSync/Nav2 components.

Separate fresh outputs for every attempt. Stationary acquisition precedes any
navigation; original submitted cohorts and production sources are read-only.
"""
import argparse
import copy
import hashlib
import json
import math
import os
import random
from pathlib import Path
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.srv import GetCostmap
from rcl_interfaces.srv import GetParameters
from rulesync_interfaces.msg import RuleGuardState
from tf2_ros import Buffer, TransformListener, TransformException
import yaml

import benchmark_nav2_combination as bench
base=bench.base


def write(path,obj):
    path.write_text(json.dumps(obj,indent=2)+'\n')


class WarehouseProbe(bench.Probe):
    def __init__(self,root,trial,map_record):
        self.map_record=map_record
        self.latest_scan=None
        self.use_map_pose=False; self.map_pose_ready=False
        super().__init__(root,trial)
        self.tf_buffer=Buffer(); self.tf_listener=TransformListener(self.tf_buffer,self)
        self.odom_log=(root/'localization.jsonl').open('w')
        self.gpu_log=None

    def sample_gpu(self):
        clients={}
        for process in Path('/proc').iterdir():
            if not process.name.isdigit():continue
            try:
                for fd in (process/'fd').iterdir():
                    try:
                        if '/dev/dri/' not in os.readlink(fd):continue
                        raw=(process/'fdinfo'/fd.name).read_text();fields={line.split(':',1)[0]:line.split(':',1)[1].strip() for line in raw.splitlines() if line.startswith('drm-')}
                        if fields.get('drm-driver')=='i915':clients[(process.name,fields.get('drm-client-id',fd.name))]=dict(pid=int(process.name),fields=fields)
                    except OSError:pass
            except OSError:pass
        self.gpu_log.write(json.dumps(dict(receipt_ns=time.monotonic_ns(),sim_ns=self.get_clock().now().nanoseconds,clients=list(clients.values())))+'\n');self.gpu_log.flush()

    def on_odom(self,msg):
        if not self.use_map_pose:
            return super().on_odom(msg)
        try:
            tf=self.tf_buffer.lookup_transform('map','base_footprint',rclpy.time.Time())
            if (self.get_clock().now().nanoseconds-base.stamp_ns(tf.header.stamp))/1e9>.25:
                return
        except TransformException:
            return
        t=tf.transform.translation; q=tf.transform.rotation
        self.odom=[t.x,t.y]; self.last_pose_at=time.monotonic(); self.map_pose_ready=True
        p=PoseStamped(); p.header=tf.header; p.pose.position.x=t.x; p.pose.position.y=t.y
        p.pose.orientation=copy.deepcopy(q); self.pose_pub.publish(p)
        row=dict(receipt_ns=time.monotonic_ns(),sim_ns=base.stamp_ns(tf.header.stamp),
            map_xy=self.odom,raw_odom_xy=[msg.pose.pose.position.x,msg.pose.pose.position.y],
            quaternion=[q.x,q.y,q.z,q.w])
        self.odom_log.write(json.dumps(row)+'\n')

    def on_scan(self,msg):
        self.scan_count+=1; self.latest_scan=msg

    def publish_map(self):
        m=self.map_record; grid=OccupancyGrid()
        grid.header.frame_id='map'; grid.header.stamp=self.get_clock().now().to_msg()
        grid.info.resolution=m['resolution']; grid.info.width=m['width']; grid.info.height=m['height']
        grid.info.origin.position.x=m['origin'][0]; grid.info.origin.position.y=m['origin'][1]
        grid.info.origin.orientation.w=1.; grid.info.map_load_time=grid.header.stamp
        grid.data=m['data']; self.map_pub.publish(grid)

    def registration(self):
        m=self.map_record; scan=self.latest_scan; world=self.last_world
        q=world['quaternion']; yaw=math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]**2+q[2]**2))
        # Laser origin from the retained robot's sensor pose in base_scan;
        # base_scan itself has no link pose. Initial acquisition is stationary.
        sx=world['xyz'][0]-.064*math.cos(yaw); sy=world['xyz'][1]-.064*math.sin(yaw)
        grid=np.asarray(m['data']).reshape(m['height'],m['width'])
        errors=[]; endpoints=[]
        for i,r in enumerate(scan.ranges):
            if not math.isfinite(r) or not .10<r<min(scan.range_max-.05,15.): continue
            a=yaw+scan.angle_min+i*scan.angle_increment
            x=sx+r*math.cos(a); y=sy+r*math.sin(a)
            col=math.floor((x-m['origin'][0])/m['resolution'])
            row=math.floor((y-m['origin'][1])/m['resolution'])
            lo_x=max(0,col-20); hi_x=min(m['width'],col+21)
            lo_y=max(0,row-20); hi_y=min(m['height'],row+21)
            if hi_x<=lo_x or hi_y<=lo_y: error=2.
            else:
                ys,xs=np.where(grid[lo_y:hi_y,lo_x:hi_x]==100)
                error=float(np.min(np.hypot((xs+lo_x+.5)*m['resolution']+m['origin'][0]-x,
                    (ys+lo_y+.5)*m['resolution']+m['origin'][1]-y))) if len(xs) else 2.
            errors.append(error); endpoints.append([x,y,error])
        result=dict(finite_returns=len(errors),median_endpoint_map_distance_m=float(np.median(errors)) if errors else None,
            fraction_within_030m=float(np.mean(np.asarray(errors)<=.30)) if errors else 0.,
            world_xyz=world['xyz'],world_yaw=yaw,odom_xy=self.odom,
            thresholds=dict(min_returns=60,median_max_m=.25,min_fraction_within_030m=.70),
            scope='stationary one-pose registration diagnostic; not a localization-accuracy study',endpoints=endpoints)
        result['passed']=bool(len(errors)>=60 and np.median(errors)<=.25 and result['fraction_within_030m']>=.70)
        write(self.root/'registration.json',result)
        write(self.root/'stationary_scan.json',dict(angle_min=scan.angle_min,angle_increment=scan.angle_increment,
            range_min=scan.range_min,range_max=scan.range_max,ranges=[float(r) if math.isfinite(r) else None for r in scan.ranges]))
        return result


def applied(node,epoch):
    probes=node.task_probes[str(epoch)]
    center=probes['inside']; outside=probes['outside']
    deadline=time.monotonic()+30.
    for c in node.costmaps.values():
        if not c.wait_for_service(timeout_sec=15.): raise TimeoutError('costmap unavailable')
    values={}
    while time.monotonic()<deadline:
        maps={n:bench.read_query(node,c,GetCostmap.Request()).map for n,c in node.costmaps.items()}
        values={n:[base.cost_at(m,*center),base.cost_at(m,*outside)] for n,m in maps.items()}
        fresh=node.mask_generation is not None and all(base.stamp_ns(m.metadata.update_time)>=node.mask_generation['stamp_ns'] for m in maps.values())
        if fresh and all(v[0]==254 and v[1]!=254 for v in values.values()):
            for name,m in maps.items():
                p=m.metadata
                write(node.root/f'costmap_epoch{epoch}_{name}.json',dict(epoch=epoch,frame=m.header.frame_id,
                    resolution=p.resolution,width=p.size_x,height=p.size_y,
                    origin=[p.origin.position.x,p.origin.position.y],update_time_ns=base.stamp_ns(p.update_time),data=list(m.data)))
            node.event('corresponding_costmaps_applied',epoch=epoch,values=values,mask=node.mask_generation)
            return
        bench.spin_for(node,.05)
    raise TimeoutError('corresponding mask absent from actual costmaps: '+str(values))


def run(args):
    assert 0<=args.seed<2**31
    assert args.speed in (.10,.20,.25)
    formal=bool(args.cohort_id)
    if formal:
        assert args.cohort_id=='FurnishedGeneralization1350_20261030'
        assert args.stage=='recovery' and args.target_rtf==1. and args.render_mode=='gpu' and args.controller_profile=='legacy' and args.geometry_profile=='standard'
        assert args.seed in (51001,51002,51003,51004,51005)
    assert 0<=args.ros_domain<=232 and args.concurrency>=1
    random.seed(args.seed); np.random.seed(args.seed)
    root=args.output; root.mkdir(parents=True,exist_ok=False)
    # Each process owns its camera outputs, including in concurrent containers.
    # Change only documentation save paths in the run-local world copy.
    tree=ET.parse(args.scene/(args.scene_id+'.sdf'))
    camera_paths=tree.findall('./world/model/link/sensor/camera/save/path')
    assert len(camera_paths)==2
    for path in camera_paths:
        name=Path(path.text).name
        assert name in ('perspective','overview')
        assert path.text=='/experiment/current/images/'+name
        path.text=str(root/'images'/name)
    step=tree.find('./world/physics/max_step_size');factor=tree.find('./world/physics/real_time_factor')
    assert step is not None and factor is not None and float(step.text)==.003 and float(factor.text)==1.
    if args.target_rtf!=1.:factor.text=str(args.target_rtf)
    tree.write(root/'world.sdf',encoding='unicode',xml_declaration=True)
    map_record=json.loads((args.scene/'map.json').read_text())
    migration=json.loads((args.scene/'migration_manifest.json').read_text())
    case=json.loads(args.case.read_text()) if args.case else None
    if case:
        assert case['schema_version']==1 and case['scene']==args.scene_id
        if formal:assert case.get('formal_eligible') and case.get('formal_scope')=='new_layout_motion_under_fixed_configuration'
        assert case['map_sha256']==hashlib.sha256((args.scene/'map.json').read_bytes()).hexdigest()
        assert case['trigger']['type']=='displacement_from_spawn'
        assert case['trigger']['distance_m']>0 and case['reference_length_m']>=6
        assert not case.get('planning_only') or args.stage in ('plan','plan_pair'), 'Planning-only input cannot release motion'
        args.spawn=case['spawn'];args.goal=case['goal']
        base.RULE=case['initial_rule'];bench.NEW=case['replacement_rule']
        for polygon in [base.RULE,bench.NEW]:
            assert len(polygon)>=3 and all(len(p)==2 and all(math.isfinite(v) for v in p) for p in polygon)
    base.SPAWN=tuple(args.spawn); base.GOAL=tuple(args.goal)
    assert args.stage=='stationary' or case or base.SPAWN==(-2.,-.5), 'Custom recovery geometry requires a qualified task definition'
    occupancy=np.asarray(map_record['data']).reshape(map_record['height'],map_record['width'])
    rows,cols=np.where(occupancy==100)
    goal_clearance=float(np.min(np.hypot((cols+.5)*map_record['resolution']+map_record['origin'][0]-base.GOAL[0],
        (rows+.5)*map_record['resolution']+map_record['origin'][1]-base.GOAL[1])))-math.sqrt(2)*map_record['resolution']/2
    assert goal_clearance>=.50,'Task goal overlaps the retained stop polygon clearance: '+str(goal_clearance)
    trial=dict(arm='full',condition='normal',direction=1,rep=args.seed-51001 if formal else 0,seed=args.seed,speed_mps=args.speed)
    hashes=bench.freeze(root)
    for path in [Path(__file__),args.scene/(args.scene_id+'.sdf'),args.scene/'migration_manifest.json',args.scene/'map.json']:
        dest=root/'source'/('scene_'+path.name); shutil.copy2(path,dest)
    if case:shutil.copy2(args.case,root/'source/scene_case.json')
    probes=case['costmap_probes'] if case else {'1':{'inside':[0.,.6],'outside':[.2,-.5]},'2':{'inside':[.2,-.5],'outside':[0.,.6]}}
    sim_budget=None;wall_budget=140.
    if case:
        timing=case['timing']
        sim_budget=timing['sim_fixed_s']+timing['distance_multiplier']*case['reference_length_m']/args.speed
        wall_budget=timing['wall_fixed_s']+sim_budget/timing['minimum_budget_rtf']
        assert 0<sim_budget<4000 and 0<wall_budget<11000
    params=base.parameters(); params['lifecycle_manager_pilot']['ros__parameters']['autostart']=False
    params['amcl']={'ros__parameters':dict(use_sim_time=True,base_frame_id='base_footprint',
        global_frame_id='map',odom_frame_id='odom',scan_topic='scan',map_topic='map',
        tf_broadcast=True,transform_tolerance=1.,robot_model_type='nav2_amcl::DifferentialMotionModel',
        laser_model_type='likelihood_field',max_beams=60,min_particles=500,max_particles=2000,
        alpha1=.2,alpha2=.2,alpha3=.2,alpha4=.2,alpha5=.2,sigma_hit=.2,z_hit=.5,z_rand=.5,
        update_min_d=.1,update_min_a=.1,set_initial_pose=True,random_seed=args.seed,
        initial_pose=dict(x=base.SPAWN[0],y=base.SPAWN[1],z=0.,yaw=0.))}
    params['lifecycle_manager_pilot']['ros__parameters']['node_names'].insert(0,'amcl')
    params['controller_server']['ros__parameters']['GoalChecker']['xy_goal_tolerance']=.10
    params['controller_server']['ros__parameters']['FollowPath']['desired_linear_vel']=args.speed
    if args.controller_profile=='low_speed_v1':
        assert not formal
        params['controller_server']['ros__parameters']['FollowPath'].update(regulated_linear_scaling_min_speed=.05,cost_scaling_dist=.5,use_regulated_linear_velocity_scaling=True,use_cost_regulated_linear_velocity_scaling=True,inflation_cost_scaling_factor=3.)
    if args.controller_profile=='upstream_mppi_v1':
        assert not formal
        upstream=Path('/experiment/nav2_upstream_1_5_2/nav2_params.yaml')
        assert hashlib.sha256(upstream.read_bytes()).hexdigest()=='31401c31d1dd185b9e671adae54bb966daac277ac562390425616042947bb317'
        follow=copy.deepcopy(yaml.safe_load(upstream.read_text())['controller_server']['ros__parameters']['FollowPath'])
        assert follow['plugin']=='nav2_mppi_controller::MPPIController'
        follow.update(vx_max=args.speed,vx_min=-args.speed,visualize=False,publish_optimal_trajectory=False)
        params['controller_server']['ros__parameters']['FollowPath']=follow
        shutil.copy2(upstream,root/'source/nav2_upstream_1_5_2.yaml')
    for scope in ['global_costmap','local_costmap']:
        p=params[scope][scope]['ros__parameters']; p['filters']=['keepout_filter','filter_inflation']
        p['filter_inflation']=dict(plugin='nav2_costmap_2d::InflationLayer',inflation_radius=.8,cost_scaling_factor=3.)
    stop_zone=json.loads(params['collision_monitor']['ros__parameters']['PolygonStop']['points'])
    protected_radius=max(math.hypot(*point) for point in stop_zone)
    if args.geometry_profile=='stop_envelope':
        assert not formal
        for name in ['global_costmap','local_costmap']:params[name][name]['ros__parameters']['robot_radius']=protected_radius
    if args.geometry_profile=='stop_polygon':
        assert not formal and args.controller_profile=='upstream_mppi_v1'
        params['local_costmap']['local_costmap']['ros__parameters']['footprint']=json.dumps(stop_zone)
        follow=params['controller_server']['ros__parameters']['FollowPath']
        follow['CostCritic']['consider_footprint']=True;follow['TrajectoryValidator']['consider_footprint']=True
    (root/'params.yaml').write_text(yaml.safe_dump(params))
    env=os.environ.copy(); env.update(ROS_DOMAIN_ID=str(args.ros_domain),GZ_PARTITION='rulesync_scene_'+root.name,
        RULESYNC_CHURN_FOLDER=str(root),RULESYNC_DYNAMIC_ARM='full',RULESYNC_TEST_POSE2D=','.join(map(str,base.SPAWN)),
        RULESYNC_TEST_GOAL2D=','.join(map(str,base.GOAL)),LP_NUM_THREADS='2',OMP_NUM_THREADS='2',OPENBLAS_NUM_THREADS='1')
    if args.render_mode=='gpu':
        env.pop('LIBGL_ALWAYS_SOFTWARE',None)
        env.update(MESA_LOADER_DRIVER_OVERRIDE='iris',EGL_PLATFORM='surfaceless')
        os.environ.pop('LIBGL_ALWAYS_SOFTWARE',None)
        probe=subprocess.run(['eglinfo','-B','-p','surfaceless'],env=env,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=20)
        (root/'gpu_renderer.log').write_text(probe.stdout)
        assert probe.returncode==0 and 'Intel' in probe.stdout and 'Iris' in probe.stdout and 'llvmpipe' not in probe.stdout,'Hardware Intel rendering must be verified'
    else:env['LIBGL_ALWAYS_SOFTWARE']='1'
    env.pop('ROS_DISCOVERY_SERVER',None)
    sim=Path('/opt/ros/lyrical/share/nav2_minimal_tb3_sim')
    env['GZ_SIM_RESOURCE_PATH']='/assets/'+args.scene_id+'/models:'+str(sim/'models')+':'+str(sim.parent)
    os.environ.update(env)
    manifest=dict(pilot=not formal,formal_denominator=formal,cohort_id=args.cohort_id,stage=args.stage,trial=trial,source_sha256=hashes,
        parameters_sha256=hashlib.sha256((root/'params.yaml').read_bytes()).hexdigest(),
        world=migration.get('world_provenance_label','Paper B pinned AWS '+args.scene_id+' original meshes'),spawn=base.SPAWN,goal=base.GOAL,
        requested_speed_mps=args.speed,seed=args.seed,controller_profile=args.controller_profile,planner_geometry=dict(profile=args.geometry_profile,stop_zone=stop_zone,circumscribed_stop_radius_m=protected_radius,costmap_radius_m=protected_radius if args.geometry_profile=='stop_envelope' else .3,effective_readback=args.geometry_profile in ('stop_envelope','stop_polygon')),controller_source_sha256=hashlib.sha256((root/'source/nav2_upstream_1_5_2.yaml').read_bytes()).hexdigest() if args.controller_profile=='upstream_mppi_v1' else None,
        render_mode=args.render_mode,gpu_device='/dev/dri/renderD128' if args.render_mode=='gpu' else None,
        target_real_time_factor=args.target_rtf,physics_step_size_s=float(step.text),
        timing_profile='accelerated_simulation_DEV_separate_from_latency_cohort' if args.target_rtf!=1. else 'target_1x_simulation',
        task_id=case['id'] if case else 'legacy_short_DEV',
        task_source_sha256=hashlib.sha256(args.case.read_bytes()).hexdigest() if case else None,
        costmap_probes=probes,initial_rule=base.RULE,replacement_rule=bench.NEW,
        max_follow_wall_s=wall_budget,max_follow_sim_s=sim_budget,
        ros_domain_id=args.ros_domain,gazebo_partition=env['GZ_PARTITION'],
        declared_concurrent_jobs=args.concurrency,
        runtime_world_sha256=hashlib.sha256((root/'world.sdf').read_bytes()).hexdigest(),
        runtime_world_edits=['documentation camera save paths']+(['target real-time factor only; fixed physics step'] if args.target_rtf!=1. else []),camera_output_root=str(root/'images'),
        seed_consumers=['Gazebo --seed (including simulated sensor noise)','AMCL random_seed','Python random','NumPy random'],
        task_goal_known_map_clearance_m=goal_clearance,task_goal_min_clearance_m=.50,
        localization='AMCL map-to-odom and wheel odometry for recovery; fixed alignment only in stationary preflight; ECM pose evaluation only',
        body_envelope_xy=[[-.21,-.17],[.11,-.17],[.11,.17],[-.21,.17]],max_world_sim_gap_s=.10,
        gate_rho_m=.30,device='one handheld; no physical robot or second edge device',
        scenario=('stationary asset/map registration only' if args.stage=='stationary'
                  else 'pre-release planning qualification; no motion released' if args.stage in ('plan','plan_pair')
                  else ('frozen within-layout stability evaluation' if formal else 'geometry-qualified long-route replacement DEV') if case else 'bounded 4m integration task; not full aisle or multi-room coverage'),
        runner_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    write(root/'manifest.json',manifest)
    procs=base.Processes(root,env); node=None; result=dict(result='FAIL',pilot=not formal,formal_denominator=formal,cohort_id=args.cohort_id,stage=args.stage,phase='startup')
    role=Path('/work/scripts/benchmark_dynamic_rule_update.py')
    try:
        robot=bench.instrument_robot(sim/'urdf/gz_waffle.sdf.xacro',root,env)
        procs.start('gazebo',['gz','sim','-r','-s','--headless-rendering','--seed',str(args.seed),str(root/'world.sdf')],5.)
        procs.start('spawn',['ros2','launch','nav2_minimal_tb3_sim','spawn_tb3.launch.py',
            'robot_name:=turtlebot3_waffle',f'robot_sdf:={robot}',f'x_pose:={base.SPAWN[0]}',f'y_pose:={base.SPAWN[1]}','z_pose:=0.06','yaw:=0'],1.)
        procs.start('world_bridge',['ros2','run','ros_gz_bridge','parameter_bridge',
            '/rulesync_test/world_pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'])
        procs.start('robot_state',['ros2','run','robot_state_publisher','robot_state_publisher','--ros-args',
            '-p','use_sim_time:=true','-p','robot_description:='+(sim/'urdf/turtlebot3_waffle.urdf').read_text()])
        if args.stage=='stationary':
            procs.start('map_tf',['ros2','run','tf2_ros','static_transform_publisher','--frame-id','map',
                '--child-frame-id','odom','--x',str(base.SPAWN[0]),'--y',str(base.SPAWN[1]),'--z','0','--yaw','0'])
        rclpy.init(); node=WarehouseProbe(root,trial,map_record)
        node.task_probes=probes
        node.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
        if args.render_mode=='gpu':
            node.gpu_log=(root/'gpu_activity.jsonl').open('w');node.create_timer(2.,node.sample_gpu)
        base.wait(node,lambda:node.last_world is not None and node.scan_count>=5 and node.odom is not None,50.)
        bench.spin_for(node,1.)
        result['phase']='registration'; registration=node.registration()
        assert math.dist(node.last_world['xyz'][:2],base.SPAWN)<.05, 'Spawn drift or collision'
        assert registration['passed'], 'Original map registration needs correction; see registration.json'
        node.event('registration_checked',passed=True,scan_messages=node.scan_count)
        node.event('scene_capture_requested',world_xyz=node.last_world['xyz'],motion_released=False)
        for camera in ['perspective','overview']:
            procs.start('camera_'+camera,['gz','topic','-e','--json-output','-n','1','-t','/capture/'+camera])
        # Acquire documentation before enabling the authorization backend; never
        # activate a high-resolution camera during an admitted motion interval.
        camera_workers=[process for name,process,_ in procs.items if name in ('camera_perspective','camera_overview')]
        def captures_complete():
            assert all(process.poll() in (None,0) for process in camera_workers), 'Native camera acquisition process failed'
            return len(camera_workers)==2 and all(process.poll()==0 for process in camera_workers)
        result['phase']='documentation_capture'
        base.wait(node,captures_complete,30.)
        if args.stage=='stationary':
            result.update(result='PASS',scan_messages=node.scan_count,registration_passed=True)
            return result
        node.publish_map()
        procs.start('amcl',['ros2','run','nav2_amcl','amcl','--ros-args','--params-file',str(root/'params.yaml')])
        procs.start('bridge',['ros2','run','rulesync_nav2_bridge','bridge','--ros-args',
            '-p','auto_replan:=false','-p','auto_rollback:=false',
            '-r','/rulesync/keepout_mask:=/source/mask','-r','/keepout_costmap_filter_info:=/source/filter_info'],.3)
        procs.start('manager',[sys.executable,str(role),'--role','manager','--ros-args',
            '-p','state_path:='+str(root/'state.json'),'-p','wal_path:='+str(root/'wal.jsonl'),
            '-p','txn_log_path:='+str(root/'txn.log'),'-p','prepare_timeout_s:=12.'],.3)
        procs.start('gate',['ros2','run','topopatch_ros','gate_node','--ros-args','-p','rho:=0.3'])
        procs.start('guard',[sys.executable,str(role),'--role','guard','--ros-args',
            '-p','rho:=0.3','-p','rho_warn:=0.3','-p','rho_release:=0.05',
            '-p','snapshot_poll_s:=0.05','-p','hold_publish_s:=0.05','-p','use_sim_time:=true'])
        bench.ready_manager(node); bench.commit_rule(node,1)
        for package,exe in [('nav2_planner','planner_server'),('nav2_controller','controller_server'),('nav2_collision_monitor','collision_monitor')]:
            cmd=['ros2','run',package,exe,'--ros-args','--params-file',str(root/'params.yaml')]
            if exe=='controller_server': cmd+=['-r','cmd_vel:=/cmd_vel_controller']
            procs.start(exe,cmd,.3)
        procs.start('lifecycle',['ros2','run','nav2_lifecycle_manager','lifecycle_manager','--ros-args',
            '-r','__node:=lifecycle_manager_pilot','--params-file',str(root/'params.yaml')])
        states=bench.start_navigation(node); node.event('active_states',states=states)
        if args.controller_profile=='low_speed_v1':
            client=node.create_client(GetParameters,'/controller_server/get_parameters')
            base.wait(node,client.service_is_ready,15.)
            query=GetParameters.Request();query.names=['FollowPath.regulated_linear_scaling_min_speed','FollowPath.cost_scaling_dist']
            response=bench.read_query(node,client,query)
            values={key:float(value.double_value) for key,value in zip(['regulated_linear_scaling_min_speed','cost_scaling_dist'],response.values)}
            assert len(response.values)==2 and all(value.type==3 for value in response.values)
            assert abs(values['regulated_linear_scaling_min_speed']-.05)<1e-9 and abs(values['cost_scaling_dist']-.5)<1e-9
            values.update(receipt_ns=time.monotonic_ns(),sim_ns=node.get_clock().now().nanoseconds,readback='ROS asynchronous GetParameters; probe callbacks remain active')
            write(root/'effective_low_speed_profile.json',values)
        if args.controller_profile=='upstream_mppi_v1':
            client=node.create_client(GetParameters,'/controller_server/get_parameters');base.wait(node,client.service_is_ready,15.)
            query=GetParameters.Request();query.names=['FollowPath.vx_max','FollowPath.vx_min','FollowPath.plugin']
            response=bench.read_query(node,client,query);assert len(response.values)==3
            assert abs(response.values[0].double_value-args.speed)<1e-9 and abs(response.values[1].double_value+args.speed)<1e-9
            assert response.values[2].string_value=='nav2_mppi_controller::MPPIController'
            write(root/'effective_mppi_profile.json',dict(vx_max=response.values[0].double_value,vx_min=response.values[1].double_value,plugin=response.values[2].string_value,optimizer_rng='upstream default, not explicitly seeded by this adapter'))
        if args.geometry_profile=='stop_envelope':
            actual={}
            for name in ['global_costmap','local_costmap']:
                client=node.create_client(GetParameters,'/'+name+'/'+name+'/get_parameters');base.wait(node,client.service_is_ready,15.)
                query=GetParameters.Request();query.names=['robot_radius'];reply=bench.read_query(node,client,query)
                assert len(reply.values)==1 and abs(reply.values[0].double_value-protected_radius)<1e-9
                actual[name]=reply.values[0].double_value
            write(root/'effective_planner_geometry.json',actual)
        if args.geometry_profile=='stop_polygon':
            actual={}
            for name,names in [('global_costmap',['robot_radius']),('local_costmap',['footprint','footprint_padding'])]:
                client=node.create_client(GetParameters,'/'+name+'/'+name+'/get_parameters');base.wait(node,client.service_is_ready,15.)
                query=GetParameters.Request();query.names=names;reply=bench.read_query(node,client,query)
                if name=='global_costmap':
                    actual['global_radius']=reply.values[0].double_value;assert abs(actual['global_radius']-.3)<1e-9
                else:
                    actual['local_footprint']=json.loads(reply.values[0].string_value);actual['padding']=reply.values[1].double_value;assert actual['local_footprint']==stop_zone and actual['padding']>=0
            client=node.create_client(GetParameters,'/controller_server/get_parameters');base.wait(node,client.service_is_ready,15.)
            query=GetParameters.Request();query.names=['FollowPath.CostCritic.consider_footprint','FollowPath.TrajectoryValidator.consider_footprint'];reply=bench.read_query(node,client,query)
            assert len(reply.values)==2 and all(v.type==1 and v.bool_value for v in reply.values)
            actual['cost_critic_full_footprint']=actual['validator_full_footprint']=True
            write(root/'effective_planner_geometry.json',actual)
        node.use_map_pose=True; node.odom=None
        base.wait(node,lambda:node.map_pose_ready and node.odom is not None,20.)
        node.event('map_localization_ready',xy=node.odom)
        pubs=node.get_publishers_info_by_topic('/cmd_vel')
        assert len(pubs)==1 and pubs[0].node_name=='collision_monitor'
        applied(node,1); result['phase']='initial_follow'
        path=bench.plan(node)
        if args.stage in ('plan','plan_pair'):
            bench.admit(node,path,1)
            if args.stage=='plan_pair':
                assert base.path_intersects_polygons([[p.pose.position.x,p.pose.position.y] for p in path.poses],[bench.NEW],inflation_radius=.30),'Replacement must invalidate the actual initial path'
                bench.commit_rule(node,2);applied(node,2);bench.spin_for(node,.5)
                renewed=bench.plan(node);bench.admit(node,renewed,2)
                node.event('planning_pair_complete',motion_released=False,renewed_points=len(renewed.poses))
            node.event('planning_preflight_complete',motion_released=False,path_points=len(path.poses))
            result.update(result='PASS',phase='planning_preflight',motion_released=False,
                scan_messages=node.scan_count,registration_passed=True)
            return result
        if case:
            invalidated=base.path_intersects_polygons([[p.pose.position.x,p.pose.position.y] for p in path.poses],
                [bench.NEW],inflation_radius=.30)
            node.event('replacement_invalidates_initial_path',invalidated=bool(invalidated))
            assert bool(invalidated)==case.get('expect_path_invalidated',True),'Actual path does not match the preregistered geometry condition'
        bench.admit(node,path,1); node.active=True
        initial=bench.follow(node,path,1)
        if case:
            base.wait(node,lambda:math.dist(node.odom,base.SPAWN)>=case['trigger']['distance_m'],35.)
        else:base.wait(node,lambda:node.odom[0]>=-1.6,35.)
        assert not node.blocked,'HOLD before update trigger'
        node.event('update_requested',xy=node.odom)
        node.hold('proactive-update-request'); result['phase']='update'
        bench.commit_rule(node,2); base.wait(node,initial.done,15.)
        node.event('old_action_terminal',status=int(initial.result().status))
        applied(node,2); bench.spin_for(node,.5)
        result['phase']='recovery_follow'
        path=bench.plan(node); bench.admit(node,path,2); final=bench.follow(node,path,2)
        follow_wall_start=time.monotonic();follow_sim_start=node.get_clock().now().nanoseconds
        def finished_within_budget():
            if final.done():return True
            if sim_budget is not None and (node.get_clock().now().nanoseconds-follow_sim_start)/1e9>sim_budget:
                raise TimeoutError('recovery simulation-time budget expired')
            return False
        try:base.wait(node,finished_within_budget,wall_budget)
        except TimeoutError as exc:
            if 'simulation-time' in str(exc):raise
            raise TimeoutError('recovery wall-clock budget expired') from exc
        followed=final.result()
        node.event('recovery_budget_observed',wall_elapsed_s=time.monotonic()-follow_wall_start,
            sim_elapsed_s=(node.get_clock().now().nanoseconds-follow_sim_start)/1e9,
            wall_budget_s=wall_budget,sim_budget_s=sim_budget)
        node.event('recovery_action_terminal',status=int(followed.status),error_code=int(followed.result.error_code),
            error_msg=followed.result.error_msg)
        node.active=False; bench.spin_for(node,.3)
        assert followed.status==4 and followed.result.error_code==0,dict(status=int(followed.status),
            error_code=int(followed.result.error_code),message=followed.result.error_msg)
        error=math.dist(node.last_world['xyz'][:2],base.GOAL)
        assert error<=.20,error
        assert not node.world_errors,node.world_errors
        result.update(result='PASS',independent_goal_error_m=error,nonzero_commands=node.nonzero,
            states=states,scan_messages=node.scan_count,registration_passed=True)
    except (Exception,KeyboardInterrupt) as exc:
        result['error']=repr(exc)
    finally:
        if node is not None:
            try:
                if rclpy.ok():
                    node.hold('terminal'); bench.spin_for(node,.2); node.event('terminal',**result)
                node.events.close(); node.world.close(); node.odom_log.close()
                if node.gpu_log:node.gpu_log.close()
                node.destroy_node(); rclpy.try_shutdown()
            except Exception as exc: result['cleanup_error']=repr(exc)
        procs.close()
        result['source_unchanged']=all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in hashes.items())
        write(root/'result.json',result); print(json.dumps(result),flush=True)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene',type=Path,required=True)
    p.add_argument('--scene-id',choices=['warehouse', 'house', 'bookstore', 'office', 'hospital', 'clearpath_warehouse', 'constructed_multiroom_house', 'constructed_residential_02', 'constructed_residential_03', 'constructed_residential_04', 'constructed_residential_05', 'constructed_residential_06', 'constructed_crossdock', 'constructed_storage_aisles', 'constructed_warehouse_03', 'constructed_warehouse_04', 'constructed_warehouse_05', 'constructed_hospital_01', 'constructed_hospital_02', 'constructed_hospital_03', 'constructed_hospital_04', 'constructed_hospital_05', 'constructed_office_corridor', 'constructed_office_02', 'constructed_office_03', 'constructed_office_04', 'constructed_office_05', 'constructed_retail_01', 'constructed_retail_02', 'constructed_retail_03', 'constructed_retail_04', 'constructed_retail_05', 'constructed_retail_06'],default='warehouse')
    p.add_argument('--spawn',type=float,nargs=2,default=[-2.,-.5])
    p.add_argument('--goal',type=float,nargs=2,default=[2.,-.5])
    p.add_argument('--speed',type=float,choices=[.10,.20,.25],default=.20)
    p.add_argument('--seed',type=int,default=20261028)
    p.add_argument('--ros-domain',type=int,default=211)
    p.add_argument('--concurrency',type=int,default=1)
    p.add_argument('--case',type=Path)
    p.add_argument('--geometry-profile',choices=['standard','stop_envelope','stop_polygon'],default='standard')
    p.add_argument('--controller-profile',choices=['legacy','low_speed_v1','upstream_mppi_v1'],default='legacy')
    p.add_argument('--cohort-id',default=None)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--stage',choices=['stationary','plan','plan_pair','recovery'],default='stationary')
    p.add_argument('--render-mode',choices=['software','gpu'],default='software')
    p.add_argument('--target-rtf',type=float,choices=[1.,2.],default=1.)
    args=p.parse_args()
    raise SystemExit(0 if run(args)['result']=='PASS' else 1)
