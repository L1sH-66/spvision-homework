#!/usr/bin/env python3
import ast
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
root=Path(__file__).resolve().parents[1]
for rel,digest in json.loads((root/'docs/protected_files.sha256.json').read_text()).items():
    assert hashlib.sha256((root/rel).read_bytes()).hexdigest()==digest,'Protected file changed: '+rel
for p in (root/'src').rglob('*.xml'):ET.parse(p)
for p in (root/'src').rglob('*.py'):ast.parse(p.read_text(),filename=str(p))
config=yaml.safe_load((root/'src/sp_nav_bringup/config/nav_params.yaml').read_text())
def check(value,path=''):
    assert value is not None,'Unset parameter: '+path
    if isinstance(value,dict):
        for k,v in value.items():check(v,path+'/'+k)
    elif isinstance(value,list):
        for i,v in enumerate(value):check(v,path+'/'+str(i))
check(config)
sim=yaml.safe_load((root/'src/sp_nav_sim/config/sim_robot.yaml').read_text())['sim_robot_node']['ros__parameters']
c=config['controller_server']['ros__parameters'];p=config['planner_server']['ros__parameters']
assert c['cmd_vel_topic']==sim['cmd_vel_topic']
assert c['odom_topic']==sim['odom_topic'] and c['odom_frame_id']==sim['odom_frame_id']
assert c['local_path_topic']==p['path_topic']=='/global_path'
assert c['PidController']['max_speed']<=sim['v_max']
assert c['PidController']['max_command']<=sim['v_max'],'commands above the simulator v_max'
assert config['esdf_map_publisher']['ros__parameters']['map_yaml']==sim['map_yaml']
# ROS parameters are strictly typed: as_double() rejects an integer literal such as "300".
for key,value in c['PidController'].items():
    expected={'base_frame_id':str,'controller_mode':str,'mpc_horizon':int}.get(key,float)
    assert type(value) is expected,f'PidController.{key} must be {expected.__name__} in nav_params.yaml'
assert c['PidController']['controller_mode'] in ('mpc','pursuit')
for key,value in p['AStar'].items():
    expected={'lethal_cost':int,'smooth_iterations':int,'smooth_max_cost':int,'curvature_iterations':int,'smooth_enable':bool}.get(key,float)
    assert type(value) is expected,f'AStar.{key} must be {expected.__name__} in nav_params.yaml'
print('PASS: protected originals, XML/Python syntax, all required YAML values and types, topics/frames/map/speed consistency')
