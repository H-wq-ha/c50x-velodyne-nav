#!/usr/bin/env python3
"""Analyze check_sensor_reception.py captures and render measured data plots."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def stats(x):
    x = np.asarray(x)
    return dict(min=float(x.min()), median=float(np.median(x)), mean=float(x.mean()),
                p99=float(np.percentile(x, 99)), max=float(x.max()), std=float(x.std())) if len(x) else {}


def rate(t):
    return float((len(t) - 1) / ((int(t[-1]) - int(t[0])) / 1e9)) if len(t) > 1 else None


def read_csv(path):
    return np.genfromtxt(path, delimiter=',', names=True, dtype=None, encoding=None, ndmin=1)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('directory', type=Path)
    args = ap.parse_args()
    d = args.directory.resolve()
    raw = [json.loads(line) for line in (d / 'imu_samples.jsonl').read_text().splitlines()]
    firmware = [json.loads(line) for line in (d / 'firmware_status.jsonl').read_text().splitlines()]
    gps = [json.loads(line) for line in (d / 'nmea_packets.jsonl').read_text().splitlines()]
    im = read_csv(d / 'published_imu.csv')
    scans = read_csv(d / 'raw_scans.csv')
    clouds = read_csv(d / 'clouds.csv')
    packets = read_csv(d / 'packet_times.csv')['epoch_us'].astype(np.int64)
    status = json.loads((d / 'status.json').read_text())
    meta = json.loads((d / 'capture_meta.json').read_text())
    epoch = np.array([r['epoch_us'] for r in raw], dtype=np.int64)
    mono = np.array([r['monotonic_us'] for r in raw], dtype=np.int64)
    receive = np.array([r['receive_monotonic_ns'] for r in raw], dtype=np.int64)
    seq = np.array([r['sequence'] for r in raw], dtype=np.int64)
    accel = np.array([r['accel'] for r in raw])
    gyro = np.array([r['gyro'] for r in raw])
    corrected_a = np.column_stack([im[a] for a in ('ax', 'ay', 'az')])
    corrected_g = np.column_stack([im[a] for a in ('gx', 'gy', 'gz')])
    dt = np.diff(epoch)
    scan_us = scans['first_epoch_us'].astype(np.int64)
    scan_ns = scan_us * 1000
    cloud_ns = clouds['stamp_ns'].astype(np.int64)
    # Only use scans completely covered by the recorded IMU window.
    covered = (scan_us >= epoch[0]) & (scans['last_epoch_us'] + 1307 <= epoch[-1])
    scan_covered = scan_us[covered]
    idx = np.searchsorted(epoch, scan_covered)
    nearest = np.minimum(abs(scan_covered - epoch[np.maximum(idx - 1, 0)]),
                         abs(epoch[np.minimum(idx, len(epoch) - 1)] - scan_covered))
    samples_per_scan = np.diff(np.searchsorted(epoch, scan_us))
    valid_checksums = 0
    for g in gps:
        sentence = g['sentence']
        if '$GPRMC,' in sentence and '*' in sentence:
            body, checksum = sentence[sentence.index('$GPRMC,') + 1:].split('*', 1)
            value = 0
            for char in body:
                value ^= ord(char)
            try:
                valid_checksums += value == int(checksum[:2], 16)
            except ValueError:
                pass
    fw_delta = {k:firmware[-1][k] - firmware[0][k] for k in (
        'lost_samples', 'imu_io_errors', 'sample_queue_drops', 'tx_drops', 'pps_count', 'nmea_sent')}
    cloud_matched = np.isin(cloud_ns, scan_ns)
    cloud_steps = np.diff(cloud_ns)
    packet_dt = np.diff(packets)
    # Compare elapsed clocks, removing the arbitrary absolute epoch offset.
    elapsed_hw = (epoch - epoch[0]) / 1e6
    elapsed_rx = (receive - receive[0]) / 1e9
    drift = np.polyfit(elapsed_hw, elapsed_rx - elapsed_hw, 1)[0] * 1e6
    result = dict(
        capture=meta, frontend=status,
        imu=dict(samples=len(raw), hardware_hz=rate(epoch * 1000), receive_hz=rate(receive),
                 duration_s=float(elapsed_hw[-1]), hardware_interval_ms=stats(dt / 1000),
                 receive_interval_ms=stats(np.diff(receive) / 1e6),
                 missing_sequences=int(np.maximum(((np.diff(seq) % 2**32) - 1), 0).sum()),
                 nonincreasing_timestamps=int((dt <= 0).sum()),
                 epoch_monotonic_delta_mismatches=int((dt != np.diff(mono)).sum()),
                 invalid_flags=sum((r['flags'] & 0x27) != 0x27 for r in raw),
                 published=len(im), published_hz=rate(im['stamp_ns']),
                 published_nonincreasing=int((np.diff(im['stamp_ns']) <= 0).sum()),
                 nonfinite_raw_values=int((~np.isfinite(np.column_stack([accel, gyro]))).sum()),
                 raw_accel_mean=accel.mean(axis=0).tolist(), raw_gyro_mean=gyro.mean(axis=0).tolist(),
                 corrected_accel_mean=corrected_a.mean(axis=0).tolist(),
                 corrected_accel_std=corrected_a.std(axis=0).tolist(),
                 corrected_accel_norm=stats(np.linalg.norm(corrected_a, axis=1)),
                 corrected_gyro_mean=corrected_g.mean(axis=0).tolist(),
                 corrected_gyro_std=corrected_g.std(axis=0).tolist(),
                 temperature_c=stats([r['temperature'] for r in raw]),
                 relative_host_drift_ppm=float(drift)),
        lidar=dict(scans=len(scans), hardware_scan_hz=rate(scan_ns), receive_scan_hz=rate(scans['receive_monotonic_ns']),
                   scan_interval_ms=stats(np.diff(scan_us) / 1000), packets_per_scan=stats(scans['packets']),
                   packets=len(packets), packet_interval_us=stats(packet_dt),
                   packet_gaps_gt_1_5_period=int((packet_dt > 1.5 * np.median(packet_dt)).sum()),
                   nonincreasing_packet_times=int((packet_dt <= 0).sum()),
                   first_to_last_packet_ms=stats((scans['last_epoch_us'] - scan_us) / 1000),
                   clouds=len(clouds), cloud_hz=rate(cloud_ns), points_per_cloud=stats(clouds['points']),
                   nonfinite_xyz=int((clouds['points'] - clouds['finite_xyz']).sum()),
                   nonfinite_point_time=int((clouds['points'] - clouds['finite_times']).sum()),
                   invalid_point_times=int(clouds['invalid_times'].sum()),
                   point_time_min_s=float(clouds['min_time'].min()), point_time_max_s=float(clouds['max_time'].max()),
                   ring_count=stats(clouds['ring_count']), ring_min=int(clouds['min_ring'].min()),
                   ring_max=int(clouds['max_ring'].max()), range_m=[float(clouds['range_min'].min()),float(clouds['range_max'].max())],
                   cloud_nonincreasing=int((cloud_steps <= 0).sum()),
                   cloud_header_unmatched=int((~cloud_matched).sum()),
                   raw_driver_header_nonincreasing=int((np.diff(scans['driver_header_ns']) <= 0).sum()),
                   raw_driver_header_ns_range=[int(scans['driver_header_ns'].min()),int(scans['driver_header_ns'].max())]),
        alignment=dict(covered_scans=int(covered.sum()), nearest_imu_to_scan_start_ms=stats(nearest / 1000),
                       imu_samples_between_scan_starts=stats(samples_per_scan),
                       common_epoch_start=datetime.fromtimestamp(int(epoch[0]) / 1e6, timezone.utc).isoformat(),
                       driver_minus_shared_header_s=stats((scans['driver_header_ns'].astype(np.int64) - scan_ns) / 1e9),
                       cloud_callback_minus_raw_scan_callback_ms=stats([
                           (int(clouds['receive_monotonic_ns'][i]) - int(scans['receive_monotonic_ns'][j])) / 1e6
                           for i, j in enumerate(np.searchsorted(scan_ns, cloud_ns)) if j < len(scan_ns) and cloud_ns[i] == scan_ns[j]]),
                       physical_sync_accuracy='Not measured; nearest-sample spacing is not clock offset accuracy'),
        firmware=dict(first=firmware[0], last=firmware[-1], counter_delta=fw_delta),
        nmea=dict(packets=len(gps), valid_checksums=valid_checksums,
                  first=gps[0]['sentence'] if gps else None, last=gps[-1]['sentence'] if gps else None),
        errors=json.loads((d / 'diagnostic_errors.json').read_text()))
    (d / 'analysis.json').write_text(json.dumps(result, indent=2, ensure_ascii=False))
    plt.rcParams.update({'font.family':'Noto Sans CJK JP', 'axes.unicode_minus':False, 'font.size':10})
    origin = epoch[0]
    t = (epoch - origin) / 1e6
    ti = (im['stamp_ns'].astype(np.int64) - origin * 1000) / 1e9
    ts = (scan_us - origin) / 1e6
    fig, axes = plt.subplots(3, 2, figsize=(14, 11), constrained_layout=True)
    fig.suptitle('雷达 + IMU 实测诊断｜' + meta['started_beijing'][:19].replace('T', ' ') + ' 北京时间', fontsize=16)
    axes[0,0].plot(t[1:], dt / 1000, lw=.5, label=f"IMU {result['imu']['hardware_hz']:.3f} Hz")
    axes[0,0].axhline(5, color='gray', ls='--', label='标称 5 ms')
    axes[0,0].set(title='IMU 硬件采样间隔', ylabel='ms', xlabel='硬件相对时间 / s')
    axes[0,0].legend()
    axes[0,1].plot(ts[1:], np.diff(scan_us) / 1000, lw=.7,
                   label=f"扫描消息 {result['lidar']['hardware_scan_hz']:.3f} Hz")
    axes[0,1].axhline(100, color='gray', ls='--', label='标称 100 ms')
    axes[0,1].set(title='雷达扫描消息间隔（固定 76 包）', ylabel='ms', xlabel='硬件相对时间 / s')
    axes[0,1].legend()
    for i, axis in enumerate('XYZ'):
        axes[1,0].plot(ti, corrected_a[:,i], lw=.5, label=axis)
        axes[1,1].plot(ti, corrected_g[:,i], lw=.5, label=axis)
    axes[1,0].set(title='校正后加速度（静止时包含重力）', ylabel='m/s²', xlabel='硬件相对时间 / s')
    axes[1,1].set(title='扣除启动 10 秒均值后的角速度', ylabel='rad/s', xlabel='硬件相对时间 / s')
    axes[1,0].legend(); axes[1,1].legend()
    window = (t >= 20) & (t < 20.5)
    scan_window = (ts >= 20) & (ts < 20.5)
    axes[2,0].vlines(t[window] - 20, .55, 1, color='#2563eb', lw=.7, label='IMU 采样')
    axes[2,0].vlines(ts[scan_window] - 20, 0, .45, color='#059669', lw=2, label='雷达首包')
    axes[2,0].set(title='共同硬件时基上的冲激图（0.5 秒）', xlabel='窗口相对时间 / s', yticks=[.2,.8], yticklabels=['雷达','IMU'])
    axes[2,1].plot((scan_covered - origin) / 1e6, nearest / 1000, '.', ms=3)
    axes[2,1].set(title='雷达首包到最近 IMU 样本的距离', ylabel='ms', xlabel='硬件相对时间 / s')
    axes[2,1].text(.02,.97,'此距离反映采样相位，不能代表物理同步精度', transform=axes[2,1].transAxes, va='top', fontsize=9)
    for ax in axes.flat:
        ax.grid(alpha=.2)
    fig.savefig(d / 'sensor_diagnostics.png', dpi=145)
    plt.close(fig)
    points = np.load(d / 'last_cloud.npz')
    x,y,z = [points[a] for a in 'xyz']
    fig = plt.figure(figsize=(14,6), constrained_layout=True)
    fig.suptitle(f"VLP-16 最后一帧｜{len(x):,} 点｜ring 0–15｜时间偏移 {float(points['time'].min()):.6f}–{float(points['time'].max()):.6f} s", fontsize=14)
    ax = fig.add_subplot(121)
    sc = ax.scatter(x,y,c=z,s=.3,cmap='turbo',vmin=-2,vmax=3)
    ax.set(xlim=(-15,15),ylim=(-15,15),xlabel='X / m',ylabel='Y / m',title='俯视点云（颜色为高度）')
    ax.set_aspect('equal'); ax.grid(alpha=.2)
    fig.colorbar(sc, ax=ax, label='Z / m', shrink=.7)
    ax = fig.add_subplot(122, projection='3d')
    sel = np.arange(0,len(x),2)
    ax.scatter(x[sel],y[sel],z[sel],c=points['ring'][sel],s=.5,cmap='turbo',vmin=0,vmax=15)
    ax.set(xlim=(-12,12),ylim=(-12,12),zlim=(-3,5),xlabel='X / m',ylabel='Y / m',zlabel='Z / m',title='三维点云（颜色为激光线编号）')
    ax.view_init(elev=28, azim=-65)
    fig.savefig(d / 'lidar_pointcloud.png',dpi=145)
    plt.close(fig)
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
