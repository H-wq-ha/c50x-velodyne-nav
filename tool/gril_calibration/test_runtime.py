"""Synthetic ROS integration check; does not access the real sensors."""
import math
import argparse
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu,PointCloud2,PointField
from nav_msgs.msg import Odometry

ROOT=Path(__file__).resolve().parents[2]


def main():
    parser=argparse.ArgumentParser(description='Synthetic scene only; no real hardware')
    parser.add_argument('--imu-height',type=float,default=.76)
    parser.add_argument('--lidar-height',type=float,default=.9877)
    parser.add_argument('--manual-height',action='store_true',help='Supply the synthetic height as a prior; default tests automatic estimation without it')
    args=parser.parse_args()
    rclpy.init(args=[]);node=rclpy.create_node('gril_synthetic_scene_test')
    cloud_pub=node.create_publisher(PointCloud2,'/gril_test/cloud',20)
    imu_pub=node.create_publisher(Imu,'/gril_test/imu',2000)
    heights=[];odom=[];ground_stamps=[]
    def ground(msg):
        ground_stamps.append(msg.header.stamp.sec)
        if msg.width*msg.height<100:return
        fields={f.name:f for f in msg.fields}
        data=np.ndarray((msg.height,msg.width),dtype=np.dtype(dict(names=['z'],formats=['<f4'],offsets=[fields['z'].offset],itemsize=msg.point_step)),buffer=msg.data,strides=(msg.row_step,msg.point_step))
        heights.append(float(-np.median(data['z'])))
    def position(msg):
        p=msg.pose.pose.position;odom.append([p.x,p.y,p.z])
    subs=[node.create_subscription(PointCloud2,'/patchworkpp/ground',ground,qos_profile_sensor_data),
          node.create_subscription(Odometry,'/aft_mapped_to_init',position,qos_profile_sensor_data)]
    # A known simulated floor, a large table top, and two walls test floor selection without
    # encoding the answer in a single-plane input.
    x,y=np.meshgrid(np.arange(-8,8,.15),np.arange(-8,8,.15));floor=np.column_stack([x.ravel(),y.ravel(),np.full(x.size,-args.lidar_height)])
    x,y=np.meshgrid(np.arange(-2,2,.1),np.arange(-2,2,.1));table=np.column_stack([x.ravel(),y.ravel(),np.full(x.size,-.2*args.lidar_height)])
    a,b=np.meshgrid(np.arange(-6,6,.2),np.arange(-.8*args.lidar_height,2,.2));wall1=np.column_stack([np.full(a.size,6),a.ravel(),b.ravel()]);wall2=np.column_stack([a.ravel(),np.full(a.size,-6),b.ravel()])
    xyz=np.concatenate([floor,table,wall1,wall2]);n=len(xyz)
    dtype=np.dtype(dict(names=['x','y','z','intensity','ring','time'],formats=['<f4']*4+['<u2','<f4'],offsets=[0,4,8,12,16,20],itemsize=24))
    data=np.zeros(n,dtype=dtype)
    for i,a in enumerate('xyz'):data[a]=xyz[:,i]
    data['intensity']=20;data['ring']=np.arange(n)%16;data['time']=np.linspace(0,.099,n)
    fields=[PointField(name=k,offset=dtype.fields[k][1],datatype=PointField.UINT16 if k=='ring' else PointField.FLOAT32,count=1) for k in dtype.names]
    out=Path(tempfile.mkdtemp(prefix='gril_ros_integration_'));log=(out/'backend.log').open('w')
    binary=ROOT/'tool/gril_ws/install/gril_calib/lib/gril_calib/gril_calib_exec'
    proc=subprocess.Popen([str(binary),'--ros-args','--params-file',str(ROOT/'tool/GRIL-Calib/config/vlp16_current.yaml'),
        '-p','common.lid_topic:=/gril_test/cloud','-p','common.imu_topic:=/gril_test/imu',
        '-p','calibration.imu_sensor_height:='+str(args.imu_height),
        '-p','ground.auto_height:='+('false' if args.manual_height else 'true'),
        '-p','patchworkpp.sensor_height:='+str(args.lidar_height if args.manual_height else 0.0),
        '-p','output_directory:='+str(out)],stdout=log,stderr=subprocess.STDOUT)
    try:
        end=time.monotonic()+3
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
        epoch=1767225000
        start=time.monotonic();next_imu=0;next_cloud=0
        while time.monotonic()-start<7:
            now=time.monotonic()-start
            while next_imu/200<=now:
                ns=next_imu*5000000;msg=Imu();msg.header.stamp.sec=epoch+ns//1000000000;msg.header.stamp.nanosec=ns%1000000000
                msg.linear_acceleration.z=9.81;msg.orientation_covariance[0]=-1;imu_pub.publish(msg);next_imu+=1
            if next_cloud/10<=now:
                ns=next_cloud*100000000;cloud=PointCloud2();cloud.header.stamp.sec=epoch+ns//1000000000;cloud.header.stamp.nanosec=ns%1000000000;cloud.header.frame_id='velodyne'
                cloud.height=1;cloud.width=n;cloud.fields=fields;cloud.point_step=24;cloud.row_step=24*n;cloud.is_dense=True;cloud.data=data.tobytes();cloud_pub.publish(cloud);next_cloud+=1
            rclpy.spin_once(node,timeout_sec=.002)
            if proc.poll() is not None:raise RuntimeError('backend stopped: '+str(proc.returncode))
    finally:
        if proc.poll() is None:os.kill(proc.pid,signal.SIGINT)
        try:proc.wait(timeout=10)
        except subprocess.TimeoutExpired:proc.terminate();proc.wait(timeout=5)
        log.close();node.destroy_node();rclpy.shutdown()
    print('Integration output:',out,'exit:',proc.returncode,'ground frames:',len(heights),'odometry:',len(odom),'ground height:',np.median(heights) if heights else None)
    assert proc.returncode==0,(out/'backend.log').read_text()[-3000:]
    assert len(heights)>=30,'insufficient valid floor output'
    assert abs(float(np.median(heights))-args.lidar_height)<.02,'table was mistaken for the floor'
    assert all(s==1767225000 or s>1767225000 for s in ground_stamps),'ground timestamps missing'
    assert len(odom)>=30 and np.all(np.isfinite(odom)),'invalid odometry'
    assert not (out/'result/GRIL_Calib_result.txt').read_text().strip(),'static scene incorrectly produced calibration'
    if not args.manual_height:assert 'Automatic LiDAR floor height initialized from point clouds' in (out/'backend.log').read_text(),'automatic initialization missing'
    print('PASS: floor discrimination, point-time pipeline, finite odometry, static-data rejection, clean shutdown')


if __name__=='__main__':main()
