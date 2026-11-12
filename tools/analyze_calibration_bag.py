#!/usr/bin/env python3
"""Extract measured inputs and motion from a ROS2 calibration SQLite bag."""
import argparse
import json
from pathlib import Path
import sqlite3
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation


def stamp(m):
    return m.sec * 10**9 + m.nanosec


def stats(x):
    x = np.asarray(x)
    return {k:float(v) for k,v in zip(('min','median','p95','p99','max','mean'),
        (*np.percentile(x,[0,50,95,99,100]),np.mean(x)))}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--end-record-ns', type=int)
    args = parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    conn = sqlite3.connect('file:' + str(next(args.bag.glob('*.db3')).resolve()) + '?mode=ro',uri=True)
    topics={row[0]:(row[1],get_message(row[2])) for row in conn.execute('select id,name,type from topics')}
    imu, odom, clouds, scans = [], [], [], []
    packet_ns=[]
    cloud_number=0
    fields_seen=[]
    query='select topic_id,timestamp,data from messages'
    params=()
    if args.end_record_ns is not None:
        query+=' where timestamp <= ?'
        params=(args.end_record_ns,)
    query+=' order by timestamp,id'
    for topic_id, received, data in conn.execute(query,params):
        name,cls=topics[topic_id]
        msg=deserialize_message(data,cls)
        t=stamp(msg.header.stamp)
        if name.endswith('/imu'):
            imu.append([t,received,*[getattr(msg.linear_acceleration,k) for k in 'xyz'],
                        *[getattr(msg.angular_velocity,k) for k in 'xyz']])
        elif name=='/aft_mapped_to_init':
            p,q=msg.pose.pose.position,msg.pose.pose.orientation
            odom.append([t,received,p.x,p.y,p.z,q.x,q.y,q.z,q.w])
        elif name.endswith('/velodyne_packets'):
            pt=[stamp(p.stamp) for p in msg.packets]
            packet_ns.extend(pt)
            scans.append([t,received,pt[0],pt[-1],len(pt),
                          int.from_bytes(bytes(msg.packets[0].data[1200:1204]),'little')])
        elif name.endswith('/velodyne_points'):
            fm={1:'i1',2:'u1',3:'i2',4:'u2',5:'i4',6:'u4',7:'f4',8:'f8'}
            dtype=np.dtype({'names':[f.name for f in msg.fields],
                            'formats':[('>' if msg.is_bigendian else '<')+fm[f.datatype] for f in msg.fields],
                            'offsets':[f.offset for f in msg.fields], 'itemsize':msg.point_step})
            fields_seen=list(dtype.names)
            p=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,
                         strides=(msg.row_step,msg.point_step)).reshape(-1)
            xyz=np.column_stack([p[k] for k in 'xyz']); finite=np.isfinite(xyz).all(axis=1)
            radius=np.linalg.norm(xyz[finite],axis=1)
            tm=p['time']; rings=p['ring']; eig=np.linalg.eigvalsh(np.cov(xyz[finite].T))
            clouds.append([t,received,len(p),finite.sum(),float(tm.min()),float(tm.max()),
                           int((~np.isfinite(tm)|(tm<0)|(tm>.2)).sum()),len(np.unique(rings)),
                           float(radius.min()),float(radius.max()),*eig])
            if cloud_number%30==0:
                np.savez_compressed(args.output/f'cloud_{cloud_number:04d}.npz',stamp_ns=t,
                    xyz=xyz[::3],ring=rings[::3],point_time=tm[::3])
            cloud_number+=1
    arrays={}
    for key,values,header in [('imu',imu,'stamp_ns,receive_ns,ax,ay,az,gx,gy,gz'),
                             ('odom',odom,'stamp_ns,receive_ns,x,y,z,qx,qy,qz,qw'),
                             ('clouds',clouds,'stamp_ns,receive_ns,points,finite,min_time,max_time,invalid_time,rings,min_range,max_range,eig0,eig1,eig2'),
                             ('scans',scans,'stamp_ns,receive_ns,first_packet_ns,last_packet_ns,packets,hour_us')]:
        # Save timestamps as integers, preserving all nanoseconds.
        arr=np.array(values,dtype=object);arrays[key]=arr
        import csv
        with (args.output/(key+'.csv')).open('w') as f:
            w=csv.writer(f);w.writerow(header.split(','));w.writerows(values)
    it=np.array([r[0] for r in imu],dtype=np.int64)
    ct=np.array([r[0] for r in clouds],dtype=np.int64)
    st=np.array([r[0] for r in scans],dtype=np.int64)
    ot=np.array([r[0] for r in odom],dtype=np.int64)
    origin=it[0]
    im=np.array(imu,dtype=float);cl=np.array(clouds,dtype=float);od=np.array(odom,dtype=float)
    rotation=Rotation.from_quat(od[:,5:9]); angle=np.linalg.norm((rotation[:-1].inv()*rotation[1:]).as_rotvec(),axis=1)
    dt=np.diff(ot)/1e9;velocity=np.linalg.norm(np.diff(od[:,2:5],axis=0),axis=1)/dt
    idx=np.searchsorted(it,ct);valid=(idx>0)&(idx<len(it));i=idx[valid]
    nearest=np.minimum(ct[valid]-it[i-1],it[i]-ct[valid])/1e6
    packet_ns=np.array(packet_ns,dtype=np.int64)
    summary=dict(origin_ns=int(origin),end_record_ns=args.end_record_ns,topic_counts={k:len(v) for k,v in arrays.items()},
        durations_s={k:(int(v[-1,0])-int(v[0,0]))/1e9 for k,v in arrays.items()},
        hardware_hz={k:(len(v)-1)/((int(v[-1,0])-int(v[0,0]))/1e9) for k,v in arrays.items()},
        imu_interval_ms=stats(np.diff(it)/1e6),imu_gyro_norm_rads=stats(np.linalg.norm(im[:,5:8],axis=1)),
        imu_accel_norm_mps2=stats(np.linalg.norm(im[:,2:5],axis=1)),
        imu_nonfinite=int((~np.isfinite(im[:,2:])).sum()),imu_nonincreasing=int((np.diff(it)<=0).sum()),
        scan_interval_ms=stats(np.diff(st)/1e6),cloud_interval_ms=stats(np.diff(ct)/1e6),
        scan_without_cloud_ns=np.setdiff1d(st,ct).tolist(),cloud_without_scan_ns=np.setdiff1d(ct,st).tolist(),
        packet_interval_us=stats(np.diff(packet_ns)/1000),packet_gaps=int((np.diff(packet_ns)>2_000_000).sum()),
        scan_header_not_first_packet=sum(r[0]!=r[2] for r in scans),
        cloud_point_count=stats(cl[:,2]),nonfinite_xyz=int((cl[:,2]-cl[:,3]).sum()),invalid_point_times=int(cl[:,6].sum()),
        point_time_range_s=[float(cl[:,4].min()),float(cl[:,5].max())],rings=stats(cl[:,7]),fields=fields_seen,
        nearest_imu_to_cloud_ms=stats(nearest),odom_position_norm_m=stats(np.linalg.norm(od[:,2:5],axis=1)),
        odom_angular_rate_rads=stats(angle/dt),odom_derived_speed_mps=stats(velocity),
        odom_first_above_10m_s=float((ot[np.argmax(np.linalg.norm(od[:,2:5],axis=1)>10)]-origin)/1e9),
        nonpositive_odom_intervals=int((dt<=0).sum()))
    (args.output/'bag_analysis.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps(summary,indent=2))


if __name__=='__main__':
    main()
