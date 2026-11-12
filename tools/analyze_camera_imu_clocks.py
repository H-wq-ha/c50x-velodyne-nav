#!/usr/bin/env python3
"""Analyze recorded clock pairs; preserve measurement and arrival time separately."""
import argparse
import csv
from datetime import datetime
from decimal import Decimal
import json
from pathlib import Path
from zoneinfo import ZoneInfo

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def lower_hull_clock(x, y):
    """Same lower hull and time-midpoint edge selection as Kalibr TimestampCorrector."""
    hull = []
    for xi, yi in zip(x, y):
        if hull and xi <= hull[-1][0]:
            raise ValueError('Device clock must be strictly increasing across sessions')
        while len(hull) >= 2:
            a, b = hull[-2:]
            cross = (b[0]-a[0])*(yi-a[1])-(b[1]-a[1])*(xi-a[0])
            if cross > 0:
                break
            hull.pop()
        hull.append((float(xi), float(yi)))
    midpoint = (x[0]+x[-1])/2
    j = next(i for i in range(len(hull)-1) if hull[i][0] <= midpoint <= hull[i+1][0])
    p, q = hull[j:j+2]
    slope = (q[1]-p[1])/(q[0]-p[0])
    return dict(slope=slope, intercept_relative_s=p[1]-slope*p[0],
                hull_points=hull, selected_edge=[p, q])


def quantiles(values):
    return dict(zip(('min','p01','median','p95','p99','max'),
                    map(float,np.quantile(values,[0,.01,.5,.95,.99,1]))))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('sessions', type=Path, nargs='+')
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    imu, cameras, session_ranges = [], [], []
    for folder in args.sessions:
        samples = [json.loads(line) for line in (folder/'imu.jsonl').read_text().splitlines()]
        for item in samples:
            item['session'] = folder.name
        imu.extend(samples)
        session_ranges.append((folder.name, len(imu)-len(samples), len(imu)))
        with (folder/'camera.csv').open() as f:
            images = list(csv.DictReader(f))
        for item in images:
            item['session'] = folder.name
        cameras.extend(images)
    imu.sort(key=lambda row: row['monotonic_us'])
    s0_us = imu[0]['monotonic_us']
    host0_ns = imu[0]['receive_ns']
    x = np.array([r['monotonic_us']-s0_us for r in imu], dtype=float)/1e6
    y = np.array([r['receive_ns']-host0_ns for r in imu], dtype=float)/1e9
    host_mono = np.array([r['receive_monotonic_ns'] for r in imu],dtype=np.int64)
    wall = np.array([r['receive_ns'] for r in imu],dtype=np.int64)
    # Detect time steps rather than treating wall-clock jumps as oscillator drift.
    if np.ptp(wall-host_mono) > 1_000_000:
        raise ValueError('Host wall clock changed by >1 ms relative to monotonic clock')
    offsets = {r['epoch_us']-r['monotonic_us'] for r in imu}
    if len(offsets) != 1:
        raise ValueError('Simulation epoch changed across sessions')
    fit = lower_hull_clock(x, y)
    mapped = fit['slope']*x+fit['intercept_relative_s']
    residual = (y-mapped)*1000
    individual, holdout = {}, None
    for name, _, _ in session_ranges:
        mask = np.array([r['session']==name for r in imu])
        sub = lower_hull_clock(x[mask],y[mask])
        individual[name] = dict(slope=sub['slope'], ppm=(sub['slope']-1)*1e6,
            arrival_residual_ms=quantiles((y[mask]-(sub['slope']*x[mask]+sub['intercept_relative_s']))*1000))
    if len(session_ranges)>1:
        name=session_ranges[0][0]
        train=np.array([r['session']==name for r in imu])
        test=~train
        initial=lower_hull_clock(x[train],y[train])
        holdout=dict(train=name, test_samples=int(test.sum()),
            arrival_residual_ms=quantiles((y[test]-(initial['slope']*x[test]+initial['intercept_relative_s']))*1000),
            slope_difference_from_combined_ppm=(initial['slope']-fit['slope'])*1e6)
    imu_file=args.output/'imu_mapped.csv'
    with imu_file.open('w',newline='') as f:
        out=csv.writer(f)
        out.writerow(['session','sequence','monotonic_us','original_epoch_us','receive_host_ns',
                      'mapped_host_ns_approx','common_relative_s','arrival_residual_ms'])
        for row,relative,error in zip(imu,mapped,residual):
            out.writerow([row['session'],row['sequence'],row['monotonic_us'],row['epoch_us'],
                          row['receive_ns'],host0_ns+round(relative*1e9),relative,error])
    camera_summary={}
    for folder in args.sessions:
        for name in ('stream.infrared','stream.depth','stream.color'):
            rows=[r for r in cameras if r['session']==folder.name and r['stream']==name]
            t0=int(rows[0]['receive_monotonic_ns'])
            rows=[r for r in rows if int(r['receive_monotonic_ns'])-t0>5e9]
            raw=np.array([int(r['frame_timestamp_raw']) for r in rows],dtype=np.int64)
            sdk=np.array([float(r['timestamp_ms']) for r in rows])
            cx=(raw-raw[0])/1e6
            cy=(sdk-sdk[0])/1000
            ca,cb=np.linalg.lstsq(np.column_stack((cx,np.ones(len(cx)))),cy,rcond=None)[0]
            shifts=np.array([int(r['frame_timestamp_raw'])-int(r['sensor_timestamp_raw']) for r in rows])/1000
            full_rows=[r for r in cameras if r['session']==folder.name and r['stream']==name]
            all_shifts=np.array([int(r['frame_timestamp_raw'])-int(r['sensor_timestamp_raw']) for r in full_rows])/1000
            camera_summary[folder.name+'/'+name]=dict(steady_samples=len(rows),
                raw_to_sdk_ols_slope=float(ca), sdk_mapping_line_residual_ms=quantiles((cy-ca*cx-cb)*1000),
                exposure_midpoint_shift_ms_all=quantiles(all_shifts),
                exposure_midpoint_shift_ms_steady=quantiles(shifts))
    camera_file=args.output/'camera_mapped.csv'
    with camera_file.open('w',newline='') as f:
        out=csv.writer(f)
        out.writerow(['session','stream','frame_number','sdk_frame_host_ns',
            'exposure_midpoint_host_ns_approx','frame_to_exposure_midpoint_us',
            'common_frame_relative_s','common_exposure_relative_s_approx','receive_host_ns'])
        for row in cameras:
            ns=int(Decimal(row['timestamp_ms'])*1_000_000)
            shift=int(row['frame_timestamp_raw'])-int(row['sensor_timestamp_raw'])
            scale=camera_summary[row['session']+'/'+row['stream']]['raw_to_sdk_ols_slope']
            exposure=ns-round(scale*shift*1000)
            out.writerow([row['session'],row['stream'],row['frame_number'],ns,exposure,shift,
                (ns-host0_ns)/1e9,(exposure-host0_ns)/1e9,row['receive_ns']])
    reference=host0_ns+round(fit['intercept_relative_s']*1e9)
    report=dict(sessions=[str(p.resolve()) for p in args.sessions], imu_samples=len(imu),
        observation_span_s=float(x[-1]), observed_duration_sum_s=float(sum(
            (max(r['monotonic_us'] for r in imu if r['session']==name)-
             min(r['monotonic_us'] for r in imu if r['session']==name))/1e6
            for name,_,_ in session_ranges)),
        model='host_ns_approx = host_reference_ns + round(slope * (monotonic_us - device_reference_us) * 1000)',
        device_reference_us=s0_us, host_reference_ns=reference, host_origin_ns=host0_ns,
        host_reference_beijing=datetime.fromtimestamp(reference/1e9,ZoneInfo('Asia/Shanghai')).isoformat(),
        slope=fit['slope'], slope_ppm=(fit['slope']-1)*1e6,
        offset_only_error_over_span_ms=float((1-fit['slope'])*x[-1]*1000),
        arrival_residual_ms=quantiles(residual), per_session=individual,
        held_out_validation=holdout, wall_minus_monotonic_variation_us=float(np.ptp(wall-host_mono)/1000),
        lower_hull_selected_edge=fit['selected_edge'], camera=camera_summary,
        limitations=['Arrival residuals do not measure true camera-IMU synchronization error.',
            'Fixed minimum serial delay and IMU internal filter delay remain unobservable here.',
            'Coefficients apply only to these sessions and this uninterrupted device clock.',
            'Exposure midpoint conversion uses each session camera clock slope as a local approximation.',
            'No visual-inertial motion time-offset calibration has been performed.'])
    (args.output/'mapping.json').write_text(json.dumps(report,indent=2))
    fig,axes=plt.subplots(3,1,figsize=(10,9),constrained_layout=True)
    for name,_,_ in session_ranges:
        mask=np.array([r['session']==name for r in imu])
        label=name.rsplit('_',1)[-1]
        axes[0].scatter(x[mask],residual[mask],s=2,label='Capture '+label)
        axes[1].scatter(x[mask],(y[mask]-x[mask]-fit['intercept_relative_s'])*1000,s=2)
    axes[0].set(title='IMU arrival time minus mapped device time',ylabel='Arrival residual (ms)',yscale='symlog')
    axes[0].legend()
    axes[1].plot([0,x[-1]],[0,(fit['slope']-1)*x[-1]*1000],color='black',label='Estimated clock-rate difference')
    axes[1].set(title='A constant offset alone leaves a clock-rate trend',ylabel='Offset change (ms)')
    axes[1].legend()
    for name,_,_ in session_ranges:
        rows=[r for r in cameras if r['session']==name and r['stream']=='stream.infrared']
        cx=np.array([(int(r['receive_ns'])-host0_ns)/1e9 for r in rows])
        shift=np.array([int(r['frame_timestamp_raw'])-int(r['sensor_timestamp_raw']) for r in rows])/1000
        axes[2].scatter(cx,shift,s=3,label='Capture '+name.rsplit('_',1)[-1])
    axes[2].set(title='Left IR: frame timestamp minus exposure midpoint',ylabel='Difference (ms)')
    axes[2].legend()
    for ax in axes:
        ax.set_xlabel('Time since first capture (s)')
        ax.grid(alpha=.25)
    fig.savefig(args.output/'clock_analysis.png',dpi=170)
    plt.close(fig)
    print(json.dumps({k:v for k,v in report.items() if k not in ('camera','per_session')},indent=2))


if __name__=='__main__':
    main()
