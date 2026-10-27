#!/usr/bin/env python3
"""Development-only headless end-to-end test; publishes one goal, never cmd_vel.
Official evaluation must instead use the RViz click described in README.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener
ROOT=Path(__file__).resolve().parents[1]
output=ROOT/'test-output'/'ros-smoke';output.mkdir(parents=True,exist_ok=True)
env=dict(os.environ,QT_QPA_PLATFORM='offscreen',SDL_VIDEODRIVER='dummy',SDL_AUDIODRIVER='dummy',NAV_RESULTS_DIR=str(output))
# Run in a unique ROS domain, with all child nodes inheriting it.
os.environ.setdefault('ROS_DOMAIN_ID','87');env['ROS_DOMAIN_ID']=os.environ['ROS_DOMAIN_ID']
rclpy.init();node=rclpy.create_node('single_goal_smoke_test')
pub=node.create_publisher(PoseStamped,'/goal_pose',10);client=node.create_client(SetBool,'/set_control_enable')
buffer=Buffer();listener=TransformListener(buffer,node);maps=[]
sub=node.create_subscription(OccupancyGrid,'/global_costmap',lambda m:maps.append(m.info.width),1)
start=time.monotonic();sent=False;ok=False
with (output/'launch.log').open('w') as log:
    proc=subprocess.Popen(['ros2','launch','sp_nav_bringup','evaluation.launch.py','rviz:=false'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        while time.monotonic()-start<210:
            rclpy.spin_once(node,timeout_sec=.1)
            if proc.poll() is not None:raise RuntimeError('Launch exited; inspect launch.log')
            if not sent and time.monotonic()-start>8 and maps and client.service_is_ready() and pub.get_subscription_count()>=2 and buffer.can_transform('map','base_link',rclpy.time.Time()):
                msg=PoseStamped();msg.header.frame_id='map';msg.header.stamp=node.get_clock().now().to_msg()
                msg.pose.position.x=14.1;msg.pose.position.y=14.1;msg.pose.orientation.w=1.
                pub.publish(msg);sent=True;print('Sent exactly one /goal_pose',flush=True)
            if sent:
                for p in output.glob('*/metrics.json'):
                    if p.stat().st_mtime<time.time()-(time.monotonic()-start):continue
                    report=json.loads(p.read_text())
                    if report['status']=='completed':
                        assert report['single_fixed_goal'] and report['action_succeeded']
                        assert report['final_error_m']<.05
                        ok=True;print(json.dumps(report,indent=2));break
            if ok:break
        if not ok:raise RuntimeError('No validated completion before timeout; inspect test-output/ros-smoke/launch.log')
    finally:
        os.killpg(proc.pid,signal.SIGINT)
        try:proc.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid,signal.SIGTERM)
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        node.destroy_node();rclpy.shutdown()
print('PASS: single goal through both BT layers, planning, plugin controller and unmodified simulator')
