"""Live/replay GRIL planar calibration using the validated hardware-time frontend."""
import argparse
import collections
from datetime import datetime
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry
import yaml
from motor_startup import QuietMotorStartup

ROOT = Path(__file__).resolve().parents[2]
TOOL = ROOT/'tool'


def main():
    parser = argparse.ArgumentParser(description='VLP16 + STM32 平面标定，默认同时录包')
    parser.add_argument('--port', default='/dev/serial/by-id/usb-FIREPHX_USB_SER_FX2348N-if00')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--duration', type=float, default=0, help='总运行秒数；0 表示直到优化完成或 Ctrl+C')
    parser.add_argument('--imu-height', type=float, default=0.01, help='IMU 芯片中心离地高度，单位米；当前实测 0.01')
    lidar_height_args = parser.add_mutually_exclusive_group()
    lidar_height_args.add_argument('--lidar-height', type=float, help='可选：雷达测量原点离地高度（米）；省略时自动从点云估计')
    lidar_height_args.add_argument('--lidar-base-height', type=float, help='正立 VLP16 底座离地高度，单位米；自动加 0.0377 米换算到测量原点')
    parser.add_argument('--no-record', action='store_true')
    parser.add_argument('--no-motor-control', action='store_true', help='关闭停转校零流程，使用原有雷达持续运行方式')
    parser.add_argument('--bag', type=Path, help='回放已有平地驾驶录包，省略时使用真实硬件')
    parser.add_argument('--rate', type=float, default=0.5, help='录包回放倍率')
    args = parser.parse_args()
    if args.lidar_base_height is not None:
        if not math.isfinite(args.lidar_base_height) or args.lidar_base_height <= 0:
            parser.error('雷达底座离地高度必须为正数，单位米')
        args.lidar_height = args.lidar_base_height + 0.0377
    auto_height = args.lidar_height is None
    if not all(math.isfinite(v) and v > 0 for v in [args.imu_height,args.rate]+([] if auto_height else [args.lidar_height])):
        parser.error('高度和回放倍率必须为正数')
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error('duration 必须是非负秒数')
    binary=TOOL/'gril_ws/install/gril_calib/lib/gril_calib/gril_calib_exec'
    if not binary.is_file(): parser.error('缺少编译产物，请先运行 build.sh')
    if args.bag and not (args.bag/'metadata.yaml').is_file(): parser.error('bag 中缺少 metadata.yaml')
    if not args.bag and not Path(args.port).exists(): parser.error('IMU 串口不存在')
    out=(args.output or TOOL/'runs'/('gril_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))).resolve()
    out.mkdir(parents=True,exist_ok=False)
    (out/'solver').mkdir()
    cfg=yaml.safe_load((TOOL/'GRIL-Calib/config/vlp16_current.yaml').read_text())
    params=cfg['/**']['ros__parameters']
    params['output_directory']=str(out/'solver')
    params['calibration']['imu_sensor_height']=args.imu_height
    params.setdefault('ground',{})['auto_height']=auto_height
    params['patchworkpp']['sensor_height']=0.0 if auto_height else args.lidar_height
    (out/'parameters.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False))
    print('本轮目录：'+str(out),flush=True)
    print(f'本轮高度：IMU 芯片中心 {args.imu_height:.4f} m；雷达离地距离'+('由点云自动估计。' if auto_height else f'使用手动初值 {args.lidar_height:.4f} m。'),flush=True)
    print('正在回放已有数据，无需操作设备。' if args.bag else
          '请先保持整车静止，等【开始运动】提示；随后在平地反复左右转弯、走 8 字并加减速。',flush=True)
    rclpy.init(args=[])
    node=Node('gril_capture_monitor')
    counts=collections.Counter(); latest={}; previous={}; gaps=collections.Counter(); stamps={}; last_odom=None
    ground_sizes=[]; ground_heights=[]; ground_tilts=[]; imu_gyro=[]; imu_acc=[]; odom_dist=0.0
    def observe(name,msg):
        counts[name]+=1; latest[name]=time.monotonic()
        t=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if name in previous and t<=previous[name]: gaps[name+'_nonmonotonic']+=1
        previous[name]=t
        stamps.setdefault(name,[t,t])[1]=t
    def imu(msg):
        observe('imu',msg)
        imu_gyro.append([getattr(msg.angular_velocity,a) for a in 'xyz'])
        imu_acc.append([getattr(msg.linear_acceleration,a) for a in 'xyz'])
    def ground(msg):
        observe('ground',msg);ground_sizes.append(msg.width*msg.height)
        fields={f.name:f for f in msg.fields}
        if msg.width*msg.height<20:return
        dtype=np.dtype(dict(names=list('xyz'),formats=['>f4' if msg.is_bigendian else '<f4']*3,
            offsets=[fields[a].offset for a in 'xyz'],itemsize=msg.point_step))
        cloud=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,strides=(msg.row_step,msg.point_step))
        xyz=np.column_stack([cloud[a].ravel() for a in 'xyz']);xyz=xyz[np.all(np.isfinite(xyz),axis=1)]
        if len(xyz)<20:return
        centroid=xyz.mean(axis=0);_,_,v=np.linalg.svd(xyz-centroid,full_matrices=False)
        normal=v[-1];normal*=1 if normal[2]>=0 else -1
        ground_heights.append(float(-normal@centroid));ground_tilts.append(float(np.degrees(np.arccos(np.clip(normal[2],-1,1)))))
    def odom(msg):
        nonlocal last_odom,odom_dist
        observe('odometry',msg);p=msg.pose.pose.position;v=np.array([p.x,p.y,p.z])
        if last_odom is not None:odom_dist+=float(np.linalg.norm(v-last_odom))
        last_odom=v
        if not np.all(np.isfinite(v)):gaps['odometry_nonfinite']+=1
    imu_monitor_qos=QoSProfile(depth=2000,reliability=ReliabilityPolicy.RELIABLE)
    cloud_monitor_qos=QoSProfile(depth=200,reliability=ReliabilityPolicy.RELIABLE)
    subs=[node.create_subscription(Imu,'/calibration/imu',imu,imu_monitor_qos),
        node.create_subscription(PointCloud2,'/calibration/velodyne_points',lambda m:observe('cloud',m),cloud_monitor_qos),
        node.create_subscription(PointCloud2,'/patchworkpp/ground',ground,cloud_monitor_qos),
        node.create_subscription(Odometry,'/aft_mapped_to_init',odom,cloud_monitor_qos)]
    processes=[];files=[];started=time.monotonic();lastprint=started;ready_time=None;guided=False;reason='';failure=False
    motor=None;lidar_started=None;next_motor_poll=0.0
    def start(name,cmd):
        f=(out/(name+'.log')).open('w');files.append(f)
        processes.append((name,subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)))
    def start_lidar_pipeline():
        nonlocal lidar_started
        start('gril',[str(binary),'--ros-args','-r','__node:=gril_calibration','--params-file',str(out/'parameters.yaml')])
        start('driver',['/opt/ros/humble/lib/velodyne_driver/velodyne_driver_node','--ros-args','-r','__node:=gril_velodyne_driver',
            '-p','model:=VLP16','-p','device_ip:=192.168.1.201','-p','port:=2368','-p','rpm:=600.0',
            '-p','frame_id:=velodyne','-p','gps_time:=true','-p','time_offset:=0.0','-p','timestamp_first_packet:=true'])
        start('decoder',['/opt/ros/humble/lib/velodyne_pointcloud/velodyne_transform_node','--ros-args','-r','__node:=gril_velodyne_decoder',
            '-r','velodyne_packets:=/calibration/velodyne_packets','-r','velodyne_points:=/calibration/velodyne_points_unchecked',
            '-p','calibration:=/opt/ros/humble/share/velodyne_pointcloud/params/VLP16db.yaml','-p','model:=VLP16',
            '-p','min_range:=0.5','-p','max_range:=50.0','-p',"fixed_frame:=''",'-p',"target_frame:=''",'-p','organize_cloud:=false'])
        lidar_started=time.monotonic()
    try:
        existing=node.get_node_names()
        if any(x in existing for x in ['gril_calibration','calibration_hardware_frontend']):
            raise RuntimeError('检测到现有标定进程，请先结束原采集以避免争用设备')
        if args.bag:
            start('gril',[str(binary),'--ros-args','-r','__node:=gril_calibration','--params-file',str(out/'parameters.yaml')])
            time.sleep(1)
            start('replay',['ros2','bag','play',str(args.bag.resolve()),'--rate',str(args.rate),'--delay','2'])
        else:
            if not args.no_motor_control:
                motor=QuietMotorStartup()
                print('先暂时停转雷达，完成 10 秒静止零偏检查；之后自动恢复 600 转/分钟。请保持整套装置静止。',flush=True)
                motor.pause()
            if not args.no_record:
                start('recorder',['ros2','bag','record','-o',str(out/'bag'),'--max-cache-size','100000000',
                    '/calibration/imu','/calibration/velodyne_points','/calibration/velodyne_packets'])
            start('frontend',['/usr/bin/python3',str(TOOL/'calibration/hardware_frontend.py'),'--port',args.port,
                '--output',str(out/'frontend'),'--tolerant','--motion-mode','planar',
                '--points-input','/calibration/velodyne_points_unchecked'])
            if motor is None:start_lidar_pipeline()
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.02);now=time.monotonic()
            exits={name:p.poll() for name,p in processes}
            if any(v is not None for v in exits.values()):
                reason='进程结束 '+str(exits)
                if exits.get('replay')==0:
                    # Allow queued scans to finish before stopping the backend.
                    tail=now+5
                    while time.monotonic()<tail and processes[0][1].poll() is None:rclpy.spin_once(node,timeout_sec=.05)
                failure=any(v not in (None,0) for v in exits.values());break
            if args.duration and now-started>=args.duration:reason='达到指定运行时长';break
            if counts['imu'] and ready_time is None:
                ready_time=now
                if motor is not None:
                    print('静止检查通过，正在恢复雷达转速；请继续保持静止。',flush=True)
                    motor.resume()
            if motor is not None and ready_time is not None and lidar_started is None and now>=next_motor_poll:
                next_motor_poll=now+.4
                if motor.ready():
                    start_lidar_pipeline()
                    print('雷达已恢复稳定转速，正在初始化点云和姿态，请再保持静止 5 秒。',flush=True)
            if not args.bag and lidar_started is not None and counts['cloud'] and now-lidar_started>=5 and not guided:
                print('【开始运动】现在可以在平地行驶：左右转弯、8 字、直线加速减速，保持安装高度不变。',flush=True);guided=True
            if now-lastprint>=10:
                age=now-latest.get('imu',started)
                imu_status=f'{age:.3f}s' if counts['imu'] else '等待静止检查通过'
                floor_status=f'{np.median(ground_heights[-20:]):.3f} m' if ground_heights else '尚未稳定识别'
                print(f'已运行 {now-started:.0f}s；IMU {counts["imu"]}，点云 {counts["cloud"]}，地面检测输出 {counts["ground"]}（有点 {len(ground_heights)}），里程计 {counts["odometry"]}；最新地面点数 {ground_sizes[-1] if ground_sizes else 0}；点云估计地面距离 {floor_status}；IMU 延迟 {imu_status}',flush=True)
                lastprint=now
    except KeyboardInterrupt:reason='用户结束采集'
    except Exception as error:reason=str(error);failure=True
    finally:
        for name,p in reversed(processes):
            if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        for name,p in processes:
            try:p.wait(timeout=20)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
        for f in files:f.close()
        if motor is not None:
            try:
                motor.restore()
                print('雷达原始转速与激光设置已恢复。',flush=True)
            except Exception as error:
                reason=str(error);failure=True;print('雷达设置恢复失败：'+str(error),flush=True)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    resultfile=out/'solver/result/GRIL_Calib_result.txt'
    text=resultfile.read_text() if resultfile.exists() else ''
    complete='LiDAR-IMU calibration result:' in text and 'Translation' in text and not any(x in text.lower() for x in ['nan','inf'])
    gyro=np.array(imu_gyro);acc=np.array(imu_acc)
    frontend_path=out/'frontend/status.json'
    frontend_state=json.loads(frontend_path.read_text()) if frontend_path.exists() else None
    failure = failure or bool(gaps.get('odometry_nonfinite')) or any(p.returncode not in (0,-2) for _,p in processes)
    summary=dict(reason=reason,wall_seconds=time.monotonic()-started,calibration_complete=complete,
        process_exit_codes={name:p.returncode for name,p in processes},counts=dict(counts),timestamp_errors=dict(gaps),
        stream_hz={k:(counts[k]-1)/(v[1]-v[0]) if v[1]>v[0] else 0 for k,v in stamps.items()},
        ground_points_median=float(np.median(ground_sizes)) if ground_sizes else None,
        ground_height_median=float(np.median(ground_heights)) if ground_heights else None,
        ground_tilt_median_deg=float(np.median(ground_tilts)) if ground_tilts else None,
        odometry_distance_m=odom_dist,
        gyro_std_rad_s=gyro.std(axis=0).tolist() if len(gyro) else None,
        acceleration_std_m_s2=acc.std(axis=0).tolist() if len(acc) else None)
    summary['frontend']=frontend_state
    summary['quiet_motor_startup']=motor.events if motor is not None else None
    if frontend_state and frontend_state.get('failure'):
        summary['reason']='前端停止：'+frontend_state['failure']
    summary['runtime_healthy']=not failure and all(counts[k]>=20 for k in ['imu','cloud','ground','odometry'])
    summary['valid_floor_fraction']=len(ground_heights)/len(ground_sizes) if ground_sizes else 0.0
    height_reference=float(np.median(ground_heights)) if ground_heights else None
    height_mad=float(np.median(np.abs(np.array(ground_heights)-height_reference))) if ground_heights else None
    summary['lidar_height_mode']='automatic_from_clouds' if auto_height else 'manual'
    summary['ground_height_mad_m']=height_mad
    summary['ground_model_usable']=len(ground_heights)>=20 and summary['valid_floor_fraction']>=0.8 and (height_mad<0.03 if auto_height else abs(height_reference-args.lidar_height)<0.15)
    summary['motion_observed']=len(gyro)>20 and float(gyro.std(axis=0).max())>0.05 and float(np.linalg.norm(acc.std(axis=0)[:2]))>0.10
    (out/'run_status.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False))
    print(json.dumps(summary,ensure_ascii=False),flush=True)
    if frontend_state is not None and not frontend_state.get('ready'):
        print('前端初始化未通过，尚未开始发布标定输入；请先处理前端故障，本轮未进入地面识别和运动采集。',flush=True)
    else:
        if not summary['ground_model_usable']:print('尚未持续识别到稳定地面，请让雷达充分看到平整地面；不会用不稳定的候选平面完成标定。',flush=True)
        if not summary['motion_observed']:print('未检测到足够的转弯和加减速；静止数据只能做接收检查。',flush=True)
    if failure:
        for name,p in processes:
            if p.returncode not in (0,-2):print(name+' 最后日志：\n'+(out/(name+'.log')).read_text()[-2500:],flush=True)
    if complete:print('【已生成标定结果】'+str(resultfile),flush=True)
    elif frontend_state is not None and not frontend_state.get('ready'):
        print('【尚未生成标定结果】前端初始化失败，原因见上方日志；本次原始数据已保留。',flush=True)
    else:print('【尚未生成可用标定结果】请检查运动、地面和进程日志；本次数据已保留。',flush=True)
    return 1 if failure else (0 if complete else 2)


if __name__=='__main__':raise SystemExit(main())
