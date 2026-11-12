#!/usr/bin/env python3
"""Plot OA-LICalib JSON results as LiDAR-to-IMU coordinate frames."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np
from scipy.spatial.transform import Rotation


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--result', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--label', help='Data source label shown in the figure')
    ap.add_argument('--validation', default='真实安装精度及重复性尚未验证')
    args = ap.parse_args()
    text = args.result.read_text()
    final = json.loads(text)['final']
    transform = np.array(final['T_imu_lidar'], dtype=float)
    delay = float(final['time_offset_s'])
    method = 'OA-LICalib final refinement result'
    if (transform.shape != (4,4) or not np.isfinite(transform).all()
            or not np.isfinite(delay) or not np.allclose(transform[3], [0,0,0,1])):
        raise ValueError('Invalid homogeneous transformation')
    rotation_raw = transform[:3,:3]
    if (not np.allclose(rotation_raw.T @ rotation_raw, np.eye(3), atol=1e-3)
            or not np.isclose(np.linalg.det(rotation_raw), 1., atol=1e-3)):
        raise ValueError('Invalid rotation matrix')
    u, _, vt = np.linalg.svd(rotation_raw)
    rotation = u @ np.diag([1,1,np.linalg.det(u @ vt)]) @ vt
    translation = transform[:3,3]
    orientation = Rotation.from_matrix(rotation)
    quaternion = orientation.as_quat()
    rpy = orientation.as_euler('xyz', degrees=True)
    final_t = np.eye(4); final_t[:3,:3]=rotation;final_t[:3,3]=translation
    inverse = np.eye(4);inverse[:3,:3]=rotation.T;inverse[:3,3]=-rotation.T@translation
    args.output.mkdir(parents=True, exist_ok=True)
    metadata = dict(source_result=str(args.result.resolve()),calibration_method=method,
        data_label=args.label or args.result.name,
        parent_frame='imu_link',child_frame='velodyne',point_mapping='p_imu = R_imu_from_lidar @ p_lidar + t_imu_from_lidar',
        translation_m=translation.tolist(),rotation_matrix=rotation.tolist(),
        quaternion_xyzw=quaternion.tolist(),rpy_xyz_degrees_from_matrix=rpy.tolist(),
        transform_imu_from_lidar=final_t.tolist(),transform_lidar_from_imu=inverse.tolist(),
        separation_m=float(np.linalg.norm(translation)),time_lag_imu_to_lidar_s=delay,
        rounding_rotation_adjustment_frobenius=float(np.linalg.norm(rotation-rotation_raw)),
        validation=args.validation)
    (args.output/'lidar_imu_tf.json').write_text(json.dumps(metadata,indent=2,ensure_ascii=False))
    plt.rcParams.update({'font.family':'Noto Sans CJK JP','axes.unicode_minus':False,'font.size':11})
    fig=plt.figure(figsize=(13.5,8),layout='constrained')
    grid=fig.add_gridspec(1,2,width_ratios=[1.9,1])
    ax=fig.add_subplot(grid[0,0],projection='3d')
    info=fig.add_subplot(grid[0,1]);info.axis('off')
    colors=['#dc2626','#169447','#2563eb']
    origin=np.zeros(3); lidar=translation*100; length=12
    for center,basis,suffix,style in [(origin,np.eye(3),'I','-'),(lidar,rotation,'L','--')]:
        ax.scatter(*center,color='#111827' if suffix=='I' else '#7c3aed',s=50,depthshade=False)
        for k, (axis_name,color) in enumerate(zip('XYZ',colors)):
            vector=basis[:,k]*length;tip=center+vector
            ax.plot(*np.column_stack([center,tip]),color=color,lw=2.5,ls=style)
            ax.quiver(*center,*vector,color=color,arrow_length_ratio=.15,linewidth=1.5,linestyle=style)
            ax.text(*(tip+basis[:,k]*1.5),f'{axis_name}_{suffix}',color=color,fontsize=12,fontweight='bold')
    ax.text(* (origin+np.array([-3,1,-3])), 'IMU 原点',fontsize=11,color='#111827')
    ax.text(* (lidar+np.array([-4,0,3])), 'LiDAR 原点',fontsize=11,color='#7c3aed')
    ax.plot(*np.column_stack([origin,lidar]),color='#7c3aed',lw=1.8,ls=':')
    ax.text(*(lidar*.52+np.array([-8,-3,0])),f'{np.linalg.norm(translation)*100:.2f} cm',color='#7c3aed',fontsize=11)
    all_points=np.vstack([origin,lidar,np.eye(3)*length,*(lidar+rotation[:,k]*length for k in range(3))])
    middle=(all_points.min(axis=0)+all_points.max(axis=0))/2
    half=np.ptp(all_points,axis=0).max()/2+5
    ax.set(xlim=(middle[0]-half,middle[0]+half),ylim=(middle[1]-half,middle[1]+half),zlim=(middle[2]-half,middle[2]+half),
        xlabel='IMU X / cm',ylabel='IMU Y / cm',zlabel='IMU Z / cm')
    ax.set_box_aspect((1,1,1));ax.view_init(elev=24,azim=36)
    ax.legend(handles=[Line2D([],[],color='#374151',lw=2,ls='-',label='IMU 坐标轴（实线）'),
                       Line2D([],[],color='#374151',lw=2,ls='--',label='LiDAR 坐标轴（虚线）')],loc='upper left',fontsize=10)
    for axis in (ax.xaxis,ax.yaxis,ax.zaxis):axis.pane.set_alpha(.08)
    info.text(0,.96,'TF 父坐标系：imu_link\nTF 子坐标系：velodyne',fontsize=14,fontweight='bold',va='top',linespacing=1.7)
    info.text(0,.78,'LiDAR 原点在 IMU 坐标系中的位置',fontsize=12,fontweight='bold')
    info.text(0,.62,f'X = {lidar[0]:+.3f} cm\nY = {lidar[1]:+.3f} cm\nZ = {lidar[2]:+.3f} cm',fontsize=15,linespacing=1.7,color='#7c3aed')
    info.text(0,.48,'旋转 RPY（由结果矩阵计算）',fontsize=12,fontweight='bold')
    info.text(0,.34,f'Roll   = {rpy[0]:+.3f}°\nPitch = {rpy[1]:+.3f}°\nYaw   = {rpy[2]:+.3f}°',fontsize=13,linespacing=1.6)
    info.text(0,.22,r'$p_{IMU}=R\,p_{LiDAR}+t$',fontsize=17)
    info.text(0,.08,'红 X · 绿 Y · 蓝 Z\n箭头等长仅用于显示，不代表传感器尺寸。\n坐标范围为等比例，位置单位为厘米。',fontsize=10,color='#4b5563',linespacing=1.7)
    fig.suptitle('IMU 与 LiDAR 的 TF 关系（标定细化结果）',fontsize=18)
    fig.supxlabel(f'数据来源：{args.label or args.result.name}｜{args.validation}',fontsize=11,color='#4b5563')
    fig.savefig(args.output/'lidar_imu_tf.png',dpi=160,bbox_inches='tight')
    fig.savefig(args.output/'lidar_imu_tf.svg',bbox_inches='tight')
    plt.close(fig)
    # Orthographic projections help inspect signed translations without perspective.
    fig,axes=plt.subplots(1,3,figsize=(15,5),layout='constrained')
    for ax,(a,b),name in zip(axes,[(0,1),(0,2),(1,2)],['XY 投影','XZ 投影','YZ 投影']):
        for center,basis,suffix,style in [(origin,np.eye(3),'I','-'),(lidar,rotation,'L','--')]:
            ax.scatter(center[a],center[b],s=35,color='#111827' if suffix=='I' else '#7c3aed')
            ax.annotate('IMU' if suffix=='I' else 'LiDAR',(center[a],center[b]),xytext=(-16,-15),textcoords='offset points',fontsize=10)
            for k,color in enumerate(colors):
                tip=center+basis[:,k]*length
                if np.linalg.norm((tip-center)[[a,b]])<2:continue
                ax.annotate('',(tip[a],tip[b]),(center[a],center[b]),arrowprops=dict(arrowstyle='->',color=color,lw=1.8,ls=style))
                ax.text(tip[a],tip[b],f'{"XYZ"[k]}_{suffix}',color=color,fontsize=9)
        ax.plot([origin[a],lidar[a]],[origin[b],lidar[b]],color='#7c3aed',ls=':',lw=1.2)
        ax.set(xlabel=f'IMU {"XYZ"[a]} / cm',ylabel=f'IMU {"XYZ"[b]} / cm',title=name)
        projected=all_points[:,[a,b]]
        center2=(projected.min(axis=0)+projected.max(axis=0))/2
        half2=np.ptp(projected,axis=0).max()/2+4
        ax.set_xlim(center2[0]-half2,center2[0]+half2)
        ax.set_ylim(center2[1]-half2,center2[1]+half2)
        ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.2)
    fig.suptitle('TF 正交投影：实线 IMU、虚线 LiDAR；红 X、绿 Y、蓝 Z',fontsize=15)
    fig.savefig(args.output/'lidar_imu_tf_projections.png',dpi=150,bbox_inches='tight')
    plt.close(fig)
    print(json.dumps(metadata,indent=2,ensure_ascii=False))


if __name__=='__main__':main()
