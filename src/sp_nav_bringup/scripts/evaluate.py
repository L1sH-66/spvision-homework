#!/usr/bin/env python3
"""Passive observation only: never publishes goals, velocities or service requests."""
import csv
import json
import math
import os
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath
from action_msgs.msg import GoalStatusArray
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException

class Evaluator(Node):
    def __init__(self):
        super().__init__('navigation_evaluator')
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.goal=None;self.goals=0;self.started=None;self.first=None;self.active=None
        self.rows=[];self.last_stamp=None;self.last_xy=None;self.stable=0.;self.saved=False
        self.action_success=False;self.arrival=None;self.trajectory=NavPath();self.trajectory.header.frame_id='map'
        self.directory=Path(os.environ.get('NAV_RESULTS_DIR',str(Path.home()/'rm_nav_results')))/time.strftime('%Y%m%d-%H%M%S')
        self.create_subscription(PoseStamped,'/goal_pose',self.on_goal,10)
        self.create_subscription(NavPath,'/global_path',self.on_path,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(GoalStatusArray,'/navigate_to_pose/_action/status',self.on_status,
            QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.marker_pub=self.create_publisher(MarkerArray,'/evaluation/markers',10)
        self.track_pub=self.create_publisher(NavPath,'/evaluation/trajectory',10)
        self.create_timer(1/30,self.tick)
    def on_goal(self,msg):
        self.goals+=1
        if self.started is not None:
            self.get_logger().error('Second goal detected: trial invalid');return
        self.goal=np.array([msg.pose.position.x,msg.pose.position.y]);self.started=time.monotonic()
        self.goal_frame=msg.header.frame_id
        self.directory.mkdir(parents=True,exist_ok=True)
        self.get_logger().info('Recording single-click trial to '+str(self.directory))
    def on_path(self,msg):
        if self.started is None or msg.header.frame_id!='map' or not msg.poses:return
        p=np.array([[v.pose.position.x,v.pose.position.y] for v in msg.poses])
        self.active=p
        if self.first is None:self.first=p.copy()
    def on_status(self,msg):
        if self.started is not None:
            self.action_success=any(s.status==4 for s in msg.status_list)
    @staticmethod
    def error(x,p):
        if p is None:return None
        if len(p)==1:return float(np.linalg.norm(x-p[0]))
        a=p[:-1];v=np.diff(p,axis=0);d=np.sum(v*v,axis=1)
        t=np.clip(np.sum((x-a)*v,axis=1)/np.maximum(d,1e-20),0,1)
        return float(np.min(np.linalg.norm(x-a-t[:,None]*v,axis=1)))
    def tick(self):
        try:t=self.tf.lookup_transform('map','base_link',rclpy.time.Time())
        except TransformException:return
        stamp=t.header.stamp.sec+t.header.stamp.nanosec*1e-9
        if self.get_clock().now().nanoseconds*1e-9-stamp>.3:return
        xy=np.array([t.transform.translation.x,t.transform.translation.y])
        self.draw(xy,t)
        if self.started is None or self.last_stamp==stamp:return
        dt=stamp-self.last_stamp if self.last_stamp is not None else 0
        speed=float(np.linalg.norm(xy-self.last_xy)/dt) if dt>0 else 0.
        self.last_stamp=stamp;self.last_xy=xy.copy()
        elapsed=time.monotonic()-self.started;goal_error=float(np.linalg.norm(xy-self.goal))
        self.rows.append([elapsed,*xy,speed,goal_error,self.error(xy,self.first),self.error(xy,self.active)])
        self.stable=self.stable+dt if 0<dt<.3 and goal_error<.05 and speed<.06 else 0.
        if self.stable>=.3 and self.arrival is None:self.arrival=elapsed
        if not self.saved and self.arrival is not None and self.action_success and elapsed>self.arrival+2:
            self.save('completed');self.saved=True
    def draw(self,xy,t):
        markers=[]
        for i,(x,y,scale,color) in enumerate([(xy[0],xy[1],.5,(.1,.7,1.)),(14.1,14.1,.2,(1.,.2,.1))]):
            m=Marker();m.header.frame_id='map';m.header.stamp=self.get_clock().now().to_msg()
            m.ns='evaluation';m.id=i;m.type=Marker.CYLINDER if i==0 else Marker.SPHERE;m.action=Marker.ADD
            m.pose.position.x=float(x);m.pose.position.y=float(y);m.pose.position.z=.1;m.pose.orientation.w=1.
            m.scale.x=m.scale.y=scale;m.scale.z=.15;m.color.r,m.color.g,m.color.b=color;m.color.a=.9
            markers.append(m)
        self.marker_pub.publish(MarkerArray(markers=markers))
        if self.started is not None and (not self.trajectory.poses or self.last_stamp != t.header.stamp.sec+t.header.stamp.nanosec*1e-9):
            p=PoseStamped();p.header=t.header;p.pose.position.x=float(xy[0]);p.pose.position.y=float(xy[1]);p.pose.orientation=t.transform.rotation
            self.trajectory.poses.append(p);self.trajectory.poses=self.trajectory.poses[-10000:]
            self.trajectory.header.stamp=t.header.stamp;self.track_pub.publish(self.trajectory)
    def save(self,status):
        if self.started is None:return
        self.directory.mkdir(parents=True,exist_ok=True)
        with (self.directory/'samples.csv').open('w',newline='') as f:
            writer=csv.writer(f);writer.writerow(['elapsed_s','x','y','speed_mps','goal_error_m','first_path_error_m','active_path_error_m']);writer.writerows(self.rows)
        report={'status':status,'goal_count':self.goals,'goal':self.goal.tolist(),'goal_frame':self.goal_frame,
            'single_fixed_goal':self.goals==1 and self.goal_frame=='map' and float(np.linalg.norm(self.goal-[14.1,14.1]))<1e-6,
            'action_succeeded':self.action_success,'stable_arrival_s':self.arrival,'samples':len(self.rows),
            'final_error_m':self.rows[-1][4] if self.rows else None,
            'collision_count':'not observed: no ground-truth collision topic; inspect simulator/video'}
        for index,key in [(5,'first_path'),(6,'active_path')]:
            e=[row[index] for row in self.rows if row[index] is not None]
            if e:report[key]={'mean_m':float(np.mean(e)),'rmse_m':float(np.sqrt(np.mean(np.square(e)))),'max_m':max(e),'p95_m':float(np.percentile(e,95))}
        (self.directory/'metrics.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
        self.get_logger().info('Metrics saved: '+str(self.directory/'metrics.json'))

def main():
    rclpy.init();node=Evaluator()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.save('completed' if node.saved else 'interrupted')
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
