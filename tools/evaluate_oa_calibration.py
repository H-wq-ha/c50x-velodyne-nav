#!/usr/bin/env python3
"""Summarize OA-LICalib convergence and diagnostic geometry (no ground truth)."""
import argparse
import json
from pathlib import Path
import struct
import sys
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
sys.path.append(str(Path(__file__).resolve().parents[1] / 'tool/oa_licalib_venv/lib/python3.10/site-packages'))
import lzf


def read_pcd(path):
    with path.open('rb') as f:
        header = {}
        while True:
            row = f.readline().decode().strip().split()
            if row and row[0] != '#':
                header[row[0]] = row[1:]
            if row and row[0] == 'DATA':
                break
        n = int(header['POINTS'][0])
        fields = header['FIELDS']
        sizes = list(map(int, header['SIZE']))
        counts = list(map(int, header.get('COUNT', ['1'] * len(fields))))
        formats = [dict(F='f', U='u', I='i')[t] + str(s)
                   for t, s in zip(header['TYPE'], sizes)]
        if header['DATA'][0] == 'binary_compressed':
            compressed_size, size = struct.unpack('<II', f.read(8))
            raw = lzf.decompress(f.read(compressed_size), size)
            offset = 0
            xyz = []
            for name, s, count, fmt in zip(fields, sizes, counts, formats):
                if name in 'xyz' and len(name) == 1:
                    xyz.append((name, np.frombuffer(raw, dtype='<'+fmt,
                                count=n, offset=offset).copy()))
                offset += n * s * count
            p = np.column_stack([dict(xyz)[key] for key in 'xyz'])
        elif header['DATA'][0] == 'binary':
            dtype = np.dtype([(k, '<'+fmt, (c,)) if c > 1 else (k, '<'+fmt)
                              for k, fmt, c in zip(fields, formats, counts)])
            arr = np.frombuffer(f.read(), dtype=dtype, count=n)
            p = np.column_stack([arr[k] for k in 'xyz'])
        else:
            arr = np.loadtxt(f)
            p = arr[:, [fields.index(k) for k in 'xyz']]
    return p[np.isfinite(p).all(axis=1)]


def geometry(p):
    # Fixed 5 cm voxels and 25 cm neighborhoods; this is a map diagnostic.
    keys = np.floor(p / .05).astype(np.int32)
    _, ids = np.unique(keys, axis=0, return_index=True)
    p = p[ids]
    rng = np.random.default_rng(15)
    sample = p[rng.choice(len(p), min(8000, len(p)), replace=False)]
    tree = cKDTree(p)
    distance, index = tree.query(sample, k=48, distance_upper_bound=.25, workers=4)
    valid = np.isfinite(distance).sum(axis=1) >= 20
    thickness = []
    for ds, ix in zip(distance[valid], index[valid]):
        q = p[ix[np.isfinite(ds)]]
        eig = np.linalg.eigvalsh(np.cov(q.T))
        if eig[1] > 1e-8 and eig[0] / eig[1] < .2:
            thickness.append(np.sqrt(max(0., eig[0])))
    return p, dict(points_after_5cm_voxel=len(p), planar_neighborhoods=len(thickness),
                   median_local_plane_sigma_m=float(np.median(thickness)),
                   p90_local_plane_sigma_m=float(np.percentile(thickness, 90)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('output', type=Path)
    ap.add_argument('--bag-stem', default='dataset', help='Converted bag filename without .bag')
    args = ap.parse_args()
    out = args.output.resolve()
    records = []
    for line in (out/'simu_calib_result.txt').read_text().splitlines():
        row = line.split(',')
        if len(row) < 10:
            continue
        values = np.array(list(map(float, row[1:])))
        records.append(dict(iteration=int(values[0]), time_offset_s=values[1],
                            translation_m=values[2:5].tolist(),
                            quaternion_xyzw=values[5:9].tolist()))
    assert records
    final = records[-1]
    rot = Rotation.from_quat(final['quaternion_xyzw'])
    transform = np.eye(4)
    transform[:3, :3] = rot.as_matrix()
    transform[:3, 3] = final['translation_m']
    final['rpy_xyz_deg'] = rot.as_euler('xyz', degrees=True).tolist()
    final['T_imu_lidar'] = transform.tolist()
    changes = []
    for a, b in zip(records[:-1], records[1:]):
        delta = Rotation.from_quat(a['quaternion_xyzw']).inv()*Rotation.from_quat(b['quaternion_xyzw'])
        changes.append(dict(iteration=b['iteration'], rotation_change_deg=float(np.degrees(delta.magnitude())),
                            translation_change_m=float(np.linalg.norm(np.array(a['translation_m'])-b['translation_m'])),
                            time_offset_change_s=b['time_offset_s']-a['time_offset_s']))
    cache = out/args.bag_stem
    maps = {}
    display = {}
    for label, path in [('initial_ndt', cache/'ndt_map.pcd'),
                         ('refined', cache/f'refined_map-iter{final["iteration"]}-seg0.pcd')]:
        if path.exists():
            cloud = read_pcd(path)
            original_count = len(cloud)
            cloud, metric = geometry(cloud)
            metric['original_points'] = original_count
            maps[label] = metric
            display[label] = cloud[::max(1, len(cloud)//100000)]
    imu = np.genfromtxt(out/'inputs/imu.csv', delimiter=',', names=True)
    imu_time = (imu['stamp_ns'] - imu['stamp_ns'][0])*1e-9
    gyro = np.column_stack([imu['g'+k] for k in 'xyz'])
    eig = np.linalg.eigvalsh(np.cov(gyro.T))
    timing = {}
    ndt = list(cache.glob('ndt-odom-*.txt'))
    if ndt:
        a = np.loadtxt(ndt[0])
        r = Rotation.from_quat(a[:, 4:8])
        dt = np.diff(a[:, 0])
        omega = (r[:-1].inv()*r[1:]).as_rotvec()/dt[:, None]
        t = (a[:-1, 0]+a[1:, 0])/2
        lags = np.linspace(-.15, .15, 301)
        errors = []
        for lag in lags:
            valid = (t+lag >= imu_time[0]) & (t+lag <= imu_time[-1])
            observed = np.interp(t[valid]+lag, imu_time, np.linalg.norm(gyro, axis=1))
            errors.append(np.sqrt(np.mean((np.linalg.norm(omega[valid], axis=1)-observed)**2)))
        timing = dict(magnitude_alignment_best_lag_s=float(lags[np.argmin(errors)]),
                      magnitude_alignment_rms_rads=float(min(errors)),
                      note='NDT angular-speed diagnostic; initialization uses IMU deskew, so this is not independent ground truth')
    fig, axes = plt.subplots(2, 2, figsize=(12, 9), constrained_layout=True)
    iteration = [x['iteration'] for x in records]
    for j, k in enumerate('xyz'):
        axes[0, 0].plot(iteration, [x['translation_m'][j]*100 for x in records], 'o-', label=k)
    axes[0, 0].set(xlabel='Refinement iteration', ylabel='LiDAR in IMU (cm)', title='Extrinsic translation convergence')
    axes[0, 0].legend()
    if timing:
        axes[0, 1].plot(lags*1000, errors)
        axes[0, 1].axvline(final['time_offset_s']*1000, color='r', label='OA-LICalib offset')
        axes[0, 1].legend()
    axes[0, 1].set(xlabel='LiDAR time correction (ms)', ylabel='Angular-speed magnitude RMS (rad/s)', title='Time-offset diagnostic')
    for ax, label in zip(axes[1], ['initial_ndt', 'refined']):
        if label in display:
            p = display[label]
            ax.scatter(p[:, 0], p[:, 1], c=p[:, 2], s=.3, cmap='viridis', vmin=-1, vmax=3, rasterized=True)
            ax.set_aspect('equal')
            ax.set(xlim=(-5, 5), ylim=(-5, 5), xlabel='x (m)', ylabel='y (m)')
        ax.set_title(label.replace('_', ' ').title()+' map (top view)')
    fig.savefig(out/'diagnostics.png', dpi=160)
    plt.close(fig)
    summary = dict(final=final, iteration_records=records, iteration_changes=changes,
                   map_diagnostics=maps, gyro_covariance_eigenvalues_rads2=eig.tolist(),
                   timing_diagnostic=timing,
                   accuracy_limit='No surveyed extrinsic or independent trajectory ground truth. Map thickness, solver cost and convergence do not establish absolute calibration accuracy.')
    (out/'evaluation.json').write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
