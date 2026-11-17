#!/usr/bin/env python3
"""Real Nav2 integration pilot. Excluded from every formal cohort denominator.

Uses ComputePathToPose + FollowPath, both Keepout Filters and Collision Monitor.
The free static map is a declared oracle; live LaserScan supplies physical
obstacles. Wheel odometry controls motion; ECM model pose is evaluation-only.
No successful pilot is evidence for a paired RuleSync comparison by itself.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import TwistStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.msg import CostmapFilterInfo
from nav2_msgs.srv import GetCostmap
from nav2_msgs.action import ComputePathToPose, FollowPath
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
import yaml

from gazebo_world_pose import instrument_robot, validate_transform, stamp_ns
from rulesync3d.gate2d import path_intersects_polygons

SPAWN = (-2., -.5)
GOAL = (2., -.5)
RULE = [(-.6,.4),(.6,.4),(.6,.8),(-.6,.8)]
RHO = .30


def parameters():
    result = {}
    controller = dict(use_sim_time=True, controller_frequency=20.,
        enable_stamped_cmd_vel=True, costmap_update_timeout=1.,
        controller_plugins=['FollowPath'], goal_checker_plugins=['GoalChecker'],
        progress_checker_plugins=['ProgressChecker'], path_handler_plugins=['PathHandler'],
        failure_tolerance=.3,
        FollowPath=dict(plugin='nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
            desired_linear_vel=.20, lookahead_dist=.5, use_velocity_scaled_lookahead_dist=False,
            use_rotate_to_heading=True, rotate_to_heading_angular_vel=.5,
            use_collision_detection=True, max_allowed_time_to_collision_up_to_carrot=1.),
        GoalChecker=dict(plugin='nav2_controller::SimpleGoalChecker', xy_goal_tolerance=.15,
            yaw_goal_tolerance=.25, stateful=True),
        ProgressChecker=dict(plugin='nav2_controller::SimpleProgressChecker',
            required_movement_radius=.1, movement_time_allowance=15.),
        PathHandler=dict(plugin='nav2_controller::FeasiblePathHandler', prune_distance=2.))
    result['controller_server'] = {'ros__parameters': controller}
    result['planner_server'] = {'ros__parameters': dict(use_sim_time=True,
        planner_plugins=['GridBased'], expected_planner_frequency=1.,
        GridBased=dict(plugin='nav2_navfn_planner::NavfnPlanner', tolerance=.1,
            use_astar=True, allow_unknown=False))}
    for scope in ['global_costmap','local_costmap']:
        p = dict(use_sim_time=True, global_frame='map', robot_base_frame='base_footprint',
            update_frequency=10., publish_frequency=5., resolution=.05,
            robot_radius=RHO, transform_tolerance=.3, always_send_full_costmap=True,
            plugins=['static_layer','obstacle_layer','inflation_layer'], filters=['keepout_filter'],
            static_layer=dict(plugin='nav2_costmap_2d::StaticLayer', map_subscribe_transient_local=True),
            obstacle_layer=dict(plugin='nav2_costmap_2d::ObstacleLayer', observation_sources='scan',
                scan=dict(topic='/scan', data_type='LaserScan', clearing=True, marking=True,
                    raytrace_max_range=3., obstacle_max_range=2.5)),
            inflation_layer=dict(plugin='nav2_costmap_2d::InflationLayer',
                inflation_radius=.5, cost_scaling_factor=3.),
            keepout_filter=dict(plugin='nav2_costmap_2d::KeepoutFilter', enabled=True,
                filter_info_topic='/pilot/filter_info', override_lethal_cost=False))
        if scope=='local_costmap':
            p.update(rolling_window=True, width=6, height=6)
        result[scope] = {scope: {'ros__parameters': p}}
    result['collision_monitor'] = {'ros__parameters': dict(use_sim_time=True,
        enabled=True, enable_stamped_cmd_vel=True, base_frame_id='base_footprint',
        odom_frame_id='odom', cmd_vel_in_topic='/cmd_vel_nav', cmd_vel_out_topic='/cmd_vel',
        transform_tolerance=.3, source_timeout=.5, stop_pub_timeout=2.,
        polygons=['PolygonStop'],
        PolygonStop=dict(type='polygon', points='[[0.35,0.30],[0.35,-0.30],[-0.30,-0.30],[-0.30,0.30]]',
            action_type='stop', min_points=4, enabled=True, visualize=False),
        observation_sources=['scan'], scan=dict(type='scan',topic='/scan',enabled=True))}
    result['lifecycle_manager_pilot'] = {'ros__parameters': dict(use_sim_time=True,
        autostart=True, bond_timeout=4., node_names=['planner_server','controller_server','collision_monitor'])}
    return result


class Processes:
    def __init__(self, root, env):
        self.root,self.env,self.items=root,env,[]
    def start(self,name,args,delay=0.):
        stream=(self.root/(name+'.log')).open('w')
        p=subprocess.Popen(args,stdout=stream,stderr=subprocess.STDOUT,env=self.env,start_new_session=True)
        self.items.append((name,p,stream))
        if delay: time.sleep(delay)
    def close(self):
        for _,p,_ in reversed(self.items):
            if p.poll() is None:
                try: os.killpg(p.pid,signal.SIGINT)
                except ProcessLookupError: pass
        for _,p,stream in self.items:
            try: p.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                try: os.killpg(p.pid,signal.SIGTERM)
                except ProcessLookupError: pass
                try: p.wait(timeout=3.)
                except subprocess.TimeoutExpired: os.killpg(p.pid,signal.SIGKILL); p.wait()
            stream.close()


class Probe(Node):
    def __init__(self,root):
        super().__init__('nav2_combination_pilot')
        self.root=root; self.events=(root/'events.jsonl').open('w')
        self.world=(root/'world_pose.jsonl').open('w')
        self.last_world=None; self.world_errors=[]; self.scan_count=0; self.odom=None
        self.nonzero={'/cmd_vel_nav':0,'/cmd_vel':0}; self.active=False
        self.qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        self.map_pub=self.create_publisher(OccupancyGrid,'/map',self.qos)
        self.mask_pub=self.create_publisher(OccupancyGrid,'/pilot/mask',self.qos)
        self.info_pub=self.create_publisher(CostmapFilterInfo,'/pilot/filter_info',self.qos)
        self.create_subscription(TFMessage,'/rulesync_test/world_pose',self.on_world,100)
        self.create_subscription(LaserScan,'/scan',self.on_scan,rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom',self.on_odom,30)
        for topic in self.nonzero:
            self.create_subscription(TwistStamped,topic,lambda m,t=topic:self.on_cmd(t,m),100)
        self.planner=ActionClient(self,ComputePathToPose,'/compute_path_to_pose')
        self.controller=ActionClient(self,FollowPath,'/follow_path')
        self.states={n:self.create_client(GetState,'/'+n+'/get_state') for n in
            ['planner_server','controller_server','collision_monitor']}
        self.costmaps={n:self.create_client(GetCostmap,'/'+n+'/get_costmap') for n in
            ['global_costmap','local_costmap']}
    def event(self,kind,**data):
        self.events.write(json.dumps(dict(kind=kind,receipt_ns=time.monotonic_ns(),
            sim_ns=self.get_clock().now().nanoseconds,**data))+'\n'); self.events.flush()
    def on_world(self,m):
        try:
            if len(m.transforms)!=1: raise ValueError('one model transform required')
            t=m.transforms[0]; p=t.transform.translation; q=t.transform.rotation
            ns=stamp_ns(t.header.stamp)
            validate_transform(t.header.frame_id,t.child_frame_id,ns,(p.x,p.y,p.z),(q.x,q.y,q.z,q.w))
            if self.last_world and ns<=self.last_world['sim_ns']: raise ValueError('nonmonotonic world stamp')
            row=dict(sim_ns=ns,receipt_ns=time.monotonic_ns(),xyz=[p.x,p.y,p.z],
                parent=t.header.frame_id,child=t.child_frame_id,
                quaternion=[q.x,q.y,q.z,q.w],active=self.active)
            self.last_world=row; self.world.write(json.dumps(row)+'\n')
        except ValueError as e: self.world_errors.append(str(e))
    def on_scan(self,m): self.scan_count+=1
    def on_odom(self,m): self.odom=[m.pose.pose.position.x+SPAWN[0],m.pose.pose.position.y+SPAWN[1]]
    def on_cmd(self,t,m):
        v,w=m.twist.linear.x,m.twist.angular.z
        if abs(v)+abs(w)>1e-6: self.nonzero[t]+=1
        self.event('command',topic=t,v=v,w=w)
    def publish_maps(self,rule=False):
        grid=OccupancyGrid(); grid.header.frame_id='map'; grid.header.stamp=self.get_clock().now().to_msg()
        grid.info.resolution=.05; grid.info.width=160; grid.info.height=160
        grid.info.origin.position.x=-4.; grid.info.origin.position.y=-4.; grid.info.origin.orientation.w=1.
        grid.info.map_load_time=grid.header.stamp; grid.data=[0]*25600
        self.map_pub.publish(grid)
        if rule:
            for j in range(160):
                for i in range(160):
                    x=-4+(i+.5)*.05; y=-4+(j+.5)*.05
                    if -.6<=x<=.6 and .4<=y<=.8: grid.data[j*160+i]=100
        self.mask_pub.publish(grid)
        info=CostmapFilterInfo(); info.header=grid.header; info.type=0
        info.filter_mask_topic='/pilot/mask'; info.base=0.; info.multiplier=1.
        self.info_pub.publish(info)
        self.event('mask_published',rule=rule)


def wait(node,condition,seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.02)
        if condition(): return
    raise TimeoutError('pilot condition timed out')


def future(node,f,seconds=30.):
    wait(node,f.done,seconds)
    return f.result()


def cost_at(m,x,y):
    p=m.metadata; i=math.floor((x-p.origin.position.x)/p.resolution)
    j=math.floor((y-p.origin.position.y)/p.resolution)
    if not (0<=i<p.size_x and 0<=j<p.size_y): raise ValueError('probe outside actual costmap')
    return int(m.data[j*p.size_x+i])


def run(args):
    root=args.output; root.mkdir(parents=True,exist_ok=False)
    (root/'params.yaml').write_text(yaml.safe_dump(parameters()))
    env=os.environ.copy(); env['ROS_DOMAIN_ID']='71'; env['GZ_PARTITION']='rulesync_nav2_pilot_'+root.name
    env.pop('ROS_DISCOVERY_SERVER',None)
    os.environ.update({k:env[k] for k in ['ROS_DOMAIN_ID','GZ_PARTITION']})
    sim=Path('/opt/ros/lyrical/share/nav2_minimal_tb3_sim')
    env['GZ_SIM_RESOURCE_PATH']=str(sim/'models')+':'+str(sim.parent)
    manifest=dict(pilot=True,formal_denominator=False,spawn=SPAWN,goal=GOAL,rule=RULE,rho=RHO,
        static_map='oracle-free-8m-square; physical obstacles from live scan',
        source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in
            [Path(__file__),Path(instrument_robot.__code__.co_filename),sim/'urdf/gz_waffle.sdf.xacro',
             sim/'worlds/tb3_sandbox.sdf.xacro']})
    (root/'manifest.json').write_text(json.dumps(manifest,indent=2))
    procs=Processes(root,env); node=None; result=dict(result='FAIL',pilot=True)
    try:
        robot=instrument_robot(sim/'urdf/gz_waffle.sdf.xacro',root,env)
        world=root/'world.sdf'
        subprocess.run(['xacro','-o',str(world),'headless:=True',str(sim/'worlds/tb3_sandbox.sdf.xacro')],check=True,env=env)
        procs.start('gazebo',['gz','sim','-r','-s',str(world)],8.)
        procs.start('spawn',['ros2','launch','nav2_minimal_tb3_sim','spawn_tb3.launch.py',
            'robot_name:=turtlebot3_waffle',f'robot_sdf:={robot}','x_pose:=-2','y_pose:=-0.5','z_pose:=0.01'],1.)
        procs.start('world_bridge',['ros2','run','ros_gz_bridge','parameter_bridge',
            '/rulesync_test/world_pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'])
        procs.start('robot_state',['ros2','run','robot_state_publisher','robot_state_publisher','--ros-args',
            '-p','use_sim_time:=true','-p','robot_description:='+ (sim/'urdf/turtlebot3_waffle.urdf').read_text()])
        procs.start('map_tf',['ros2','run','tf2_ros','static_transform_publisher','--frame-id','map',
            '--child-frame-id','odom','--x','-2','--y','-0.5','--z','0','--yaw','0'])
        rclpy.init(); node=Probe(root); node.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
        wait(node,lambda:node.last_world is not None and node.scan_count>0 and node.odom is not None,40.)
        p=node.last_world['xyz']; assert math.hypot(p[0]-SPAWN[0],p[1]-SPAWN[1])<.02
        node.publish_maps()
        for package,exe in [('nav2_planner','planner_server'),('nav2_controller','controller_server'),
                            ('nav2_collision_monitor','collision_monitor')]:
            cmd=['ros2','run',package,exe,'--ros-args','--params-file',str(root/'params.yaml')]
            if exe=='controller_server': cmd+=['-r','cmd_vel:=/cmd_vel_nav']
            procs.start(exe,cmd,.3)
        procs.start('lifecycle',['ros2','run','nav2_lifecycle_manager','lifecycle_manager','--ros-args',
            '-r','__node:=lifecycle_manager_pilot','--params-file',str(root/'params.yaml')])
        for name,c in node.states.items():
            if not c.wait_for_service(timeout_sec=30.): raise TimeoutError(name+' state missing')
        deadline=time.monotonic()+50.
        while True:
            states={n:int(future(node,c.call_async(GetState.Request()),5.).current_state.id) for n,c in node.states.items()}
            if all(s==3 for s in states.values()): break
            if time.monotonic()>deadline: raise TimeoutError('not all lifecycle nodes active: '+str(states))
            for _ in range(20): rclpy.spin_once(node,timeout_sec=.02)
        node.event('active_states',states=states)
        node.publish_maps(rule=True)
        for name,c in node.costmaps.items():
            if not c.wait_for_service(timeout_sec=20.): raise TimeoutError(name+' service missing')
        deadline=time.monotonic()+20.
        while True:
            maps={n:future(node,c.call_async(GetCostmap.Request()),5.).map for n,c in node.costmaps.items()}
            costs={n:cost_at(m,0.,.6) for n,m in maps.items()}
            if all(v==254 for v in costs.values()): break
            if time.monotonic()>deadline: raise TimeoutError('actual costmap rule not applied: '+str(costs))
        node.event('actual_keepout_applied',costs=costs)
        publishers=[dict(name=i.node_name,namespace=i.node_namespace) for i in node.get_publishers_info_by_topic('/cmd_vel')]
        assert len(publishers)==1 and publishers[0]['name']=='collision_monitor', publishers
        if not node.planner.wait_for_server(timeout_sec=10.): raise TimeoutError('planner unavailable')
        request=ComputePathToPose.Goal(); request.planner_id='GridBased'
        request.goal.header.frame_id='map'; request.goal.header.stamp=node.get_clock().now().to_msg()
        request.goal.pose.position.x=GOAL[0]; request.goal.pose.position.y=GOAL[1]
        request.goal.pose.orientation.w=1.; request.use_start=False
        handle=future(node,node.planner.send_goal_async(request)); assert handle.accepted
        planned=future(node,handle.get_result_async()).result
        assert planned.error_code==0, planned.error_msg
        xy=[(p.pose.position.x,p.pose.position.y) for p in planned.path.poses]
        assert len(xy)>1 and not path_intersects_polygons(xy,[RULE],inflation_radius=RHO)
        node.event('admitted_plan',xy=xy)
        if not node.controller.wait_for_server(timeout_sec=10.): raise TimeoutError('controller unavailable')
        request=FollowPath.Goal(); request.path=planned.path; request.controller_id='FollowPath'
        request.goal_checker_id='GoalChecker'; request.progress_checker_id='ProgressChecker'; request.path_handler_id='PathHandler'
        node.active=True; handle=future(node,node.controller.send_goal_async(request)); assert handle.accepted
        followed=future(node,handle.get_result_async(),100.)
        node.active=False
        for _ in range(30): rclpy.spin_once(node,timeout_sec=.02)
        assert followed.status==4 and followed.result.error_code==0, followed.result.error_msg
        assert not node.world_errors, node.world_errors
        p=node.last_world['xyz']; goal_error=math.hypot(p[0]-GOAL[0],p[1]-GOAL[1])
        assert goal_error<=.20, goal_error
        assert all(v>0 for v in node.nonzero.values()), node.nonzero
        result.update(result='PASS',states=states,costs=costs,cmd_publishers=publishers,
            planned_points=len(xy),nonzero_commands=node.nonzero,independent_goal_error_m=goal_error,
            final_world_pose=node.last_world,scan_messages=node.scan_count)
    except Exception as e:
        result['error']=repr(e)
    finally:
        if node:
            node.event('pilot_terminal',**result)
            node.events.close(); node.world.close(); node.destroy_node(); rclpy.shutdown()
        procs.close()
        (root/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result),flush=True)
    return result['result']=='PASS'


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('--output',type=Path,required=True)
    raise SystemExit(0 if run(parser.parse_args()) else 1)
