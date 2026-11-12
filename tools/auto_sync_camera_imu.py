#!/usr/bin/env python3
"""Estimate a fresh D435i/fake_gps clock map, validate it, and save usable parameters.

Run after every device restart. Arrival residuals are NOT physical synchronization
accuracy. --motion-refine additionally needs rigid mounting and varied rotation.
"""
import argparse
from dataclasses import asdict, dataclass
from datetime import datetime
from decimal import Decimal
import importlib.util
import json
import os
from pathlib import Path
import sys
import threading
import time
from zoneinfo import ZoneInfo

import numpy as np
import pyrealsense2 as rs
import serial
from serial.tools import list_ports
from scipy.optimize import minimize_scalar


ROOT = Path(__file__).resolve().parents[1]


def quantiles(values):
    values = np.asarray(values, dtype=float)
    if not len(values):
        return None
    return dict(zip(('min', 'median', 'p95', 'p99', 'max'),
                    map(float, np.quantile(values, [0, .5, .95, .99, 1]))))


def host_stamp():
    before = time.monotonic_ns()
    wall = time.time_ns()
    after = time.monotonic_ns()
    return wall, (before + after) // 2


@dataclass(frozen=True)
class ClockMap:
    device_reference_us: int
    host_reference_monotonic_ns: int
    host_reference_epoch_ns: int
    slope: float

    def map_us(self, hardware_us):
        """Map this boot's original DRDY timestamp to approximate host epoch ns."""
        return self.host_reference_epoch_ns + round(
            self.slope * (int(hardware_us) - self.device_reference_us) * 1000)

    def mono_ns(self, hardware_us):
        return self.host_reference_monotonic_ns + round(
            self.slope * (int(hardware_us) - self.device_reference_us) * 1000)

    def map_record(self, record, max_arrival_age_s=.5):
        """Reject a stale boot's model when mapping a freshly received packet."""
        if record['flags'] & 0x23 != 0x23:
            raise ValueError('This packet has no valid hardware capture timestamp')
        mapped = self.map_us(record['monotonic_us'])
        age = (record['receive_ns']-mapped)/1e9
        if age < -.002 or age > max_arrival_age_s:
            raise ValueError('Clock map is stale or reception stalled; recalibrate this connection')
        return mapped

    @classmethod
    def load(cls, filename):
        config = json.loads(Path(filename).read_text())
        if not config['clock_mapping_ready']:
            raise ValueError('Saved calibration was not accepted')
        return cls(**config['imu_clock_map'])


def fit_clock(rows, wall_anchor, mono_anchor):
    """Kalibr's lower-convex-hull/time-midpoint-edge clock estimator."""
    if len(rows) < 100:
        raise ValueError('Not enough IMU samples to estimate a clock')
    d0, h0 = rows[0]['monotonic_us'], rows[0]['receive_monotonic_ns']
    points = [(float(r['monotonic_us'] - d0) / 1e6,
               float(r['receive_monotonic_ns'] - h0) / 1e9) for r in rows]
    hull = []
    for x, y in points:
        if hull and x <= hull[-1][0]:
            raise ValueError('IMU timer reset or nonincreasing hardware timestamps')
        while len(hull) >= 2:
            p, q = hull[-2:]
            if (q[0]-p[0])*(y-p[1]) - (q[1]-p[1])*(x-p[0]) > 0:
                break
            hull.pop()
        hull.append((x, y))
    middle = (points[0][0] + points[-1][0]) / 2
    index = next(i for i in range(len(hull)-1)
                 if hull[i][0] <= middle <= hull[i+1][0])
    p, q = hull[index:index+2]
    a = (q[1]-p[1])/(q[0]-p[0])
    b = p[1] - a*p[0]
    mono_ref = h0 + round(b * 1e9)
    return ClockMap(d0, mono_ref, wall_anchor + mono_ref - mono_anchor, a)


def arrival_residual_ms(model, rows):
    return [(r['receive_monotonic_ns'] - model.mono_ns(r['monotonic_us'])) / 1e6
            for r in rows]


def find_serial(explicit):
    if explicit:
        return explicit
    candidates = sorted(Path('/dev/serial/by-id').glob('*'))
    preferred = [str(p) for p in candidates if 'FIREPHX' in p.name]
    if len(preferred) == 1:
        return preferred[0]
    ports = [p.device for p in list_ports.comports()]
    if len(ports) == 1:
        return ports[0]
    raise ValueError('Cannot choose one IMU serial port; specify --port')


class Acquisition:
    def __init__(self, args):
        self.args = args
        self.wall_anchor, self.mono_anchor = host_stamp()
        self.imu, self.images, self.camera_gyro, self.statuses = [], [], [], []
        self.errors = []
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.pipeline = rs.pipeline()
        self.running = False
        self.thread = None
        self.capture_origin_ns = None
        spec = importlib.util.spec_from_file_location('fake_gps_receiver', args.decoder)
        receiver = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(receiver)
        self.parser = receiver.FrameParser()
        self.serial = serial.Serial(port=None, baudrate=args.baud, timeout=.05, exclusive=True)
        self.serial.dtr = False
        self.serial.rts = False
        self.serial.port = find_serial(args.port)
        self.crc_after_warmup = 0
        self.device = {}
        self.seen_camera_frames = set()

    def serial_loop(self):
        try:
            while not self.stop.is_set():
                data = self.serial.read(max(1, min(self.serial.in_waiting, 32768)))
                wall, mono = host_stamp()
                records = self.parser.feed(data)
                for r in records:
                    r.update(receive_ns=wall, receive_monotonic_ns=mono)
                with self.lock:
                    for r in records:
                        (self.imu if r['kind'] == 'imu' else self.statuses).append(r)
        except Exception as error:
            self.errors.append('serial: ' + repr(error))
            self.stop.set()

    def camera_callback(self, frame):
        try:
            wall, mono = host_stamp()
            frames = list(frame.as_frameset()) if frame.is_frameset() else [frame]
            for f in frames:
                kind = f.profile.stream_type()
                if kind not in (rs.stream.infrared, rs.stream.gyro):
                    continue
                if f.get_frame_timestamp_domain() != rs.timestamp_domain.global_time:
                    continue  # SDK mapping may not be ready during startup.
                item = dict(sdk_epoch_ns=int(Decimal(str(f.get_timestamp())) * 1_000_000),
                            receive_ns=wall, receive_monotonic_ns=mono,
                            frame_number=f.get_frame_number())
                key = (str(kind), item['frame_number'], item['sdk_epoch_ns'])
                with self.lock:
                    if key in self.seen_camera_frames:
                        continue
                    self.seen_camera_frames.add(key)
                if kind == rs.stream.gyro:
                    g = f.as_motion_frame().get_motion_data()
                    item['gyro'] = [g.x, g.y, g.z]
                    with self.lock:
                        self.camera_gyro.append(item)
                else:
                    for name in ('frame_timestamp', 'sensor_timestamp', 'actual_exposure'):
                        key = getattr(rs.frame_metadata_value, name)
                        item[name] = f.get_frame_metadata(key) if f.supports_frame_metadata(key) else None
                    with self.lock:
                        self.images.append(item)
        except Exception as error:
            self.errors.append('camera callback: ' + repr(error))
            self.stop.set()

    def capture(self):
        try:
            devices = list(rs.context().query_devices())
            if self.args.camera_serial:
                devices = [d for d in devices if d.get_info(rs.camera_info.serial_number) == self.args.camera_serial]
            if len(devices) != 1:
                raise ValueError('Cannot choose one RealSense camera; specify --camera-serial')
            device = devices[0]
            self.device = {str(k): device.get_info(k) for k in
                           (rs.camera_info.name, rs.camera_info.serial_number, rs.camera_info.firmware_version)}
            self.serial.open()
            self.thread = threading.Thread(target=self.serial_loop, daemon=True)
            self.thread.start()
            for sensor in device.query_sensors():
                if sensor.supports(rs.option.global_time_enabled):
                    sensor.set_option(rs.option.global_time_enabled, 1)
            config = rs.config()
            config.enable_device(device.get_info(rs.camera_info.serial_number))
            config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
            if self.args.motion_refine:
                config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 400)
            self.pipeline.start(config, self.camera_callback)
            self.running = True
            self.capture_origin_ns = time.monotonic_ns()
            duration = self.args.warmup + self.args.fit_seconds + self.args.validate_seconds
            print(f'已连接相机和 {self.serial.port}；预热 {self.args.warmup:g}s，'
                  f'拟合 {self.args.fit_seconds:g}s，独立验证 {self.args.validate_seconds:g}s。', flush=True)
            if self.args.motion_refine:
                print('先保持静止至少 2 秒，之后让刚性固定的相机与外置 IMU 一起变速转动约 30 秒。', flush=True)
            next_message = 10
            while not self.stop.wait(.05):
                elapsed = (time.monotonic_ns()-self.capture_origin_ns)/1e9
                if elapsed <= self.args.warmup:
                    self.crc_after_warmup = self.parser.crc_errors
                if elapsed >= duration:
                    if not self.args.motion_refine or self.motion_ready():
                        break
                    if elapsed >= duration + self.args.motion_wait_seconds:
                        break
                if elapsed > 5 and (not self.imu or not self.images
                                    or (self.args.motion_refine and not self.camera_gyro)):
                    raise ValueError('Missing external IMU, infrared images, or camera gyro data')
                if elapsed >= next_message:
                    print(json.dumps(dict(elapsed_s=round(elapsed, 1), imu=len(self.imu),
                                          images=len(self.images), camera_gyro=len(self.camera_gyro))), flush=True)
                    next_message += 10
            if self.errors:
                raise ValueError('; '.join(self.errors))
        finally:
            self.stop.set()
            if self.running:
                self.pipeline.stop()
            if self.thread:
                self.thread.join(timeout=2)
                if self.thread.is_alive():
                    raise RuntimeError('Serial reader did not stop')
            if self.serial.is_open:
                self.serial.close()

    def elapsed(self, row):
        return (row['receive_monotonic_ns'] - self.capture_origin_ns)/1e9

    def motion_ready(self):
        with self.lock:
            rows = [r for r in self.imu if self.elapsed(r) >= self.args.warmup]
        if len(rows) < 100:
            return False
        speed = np.linalg.norm(np.array([r['gyro_rads'] for r in rows]), axis=1)
        active = np.flatnonzero(speed > .18)
        return (len(active) >= 1600 and
                self.elapsed(rows[active[-1]])-self.elapsed(rows[active[0]]) >= 15
                and speed.std() >= .10)


def refine_motion(acq, model):
    """A correction added to mapped external IMU times to match camera gyro time."""
    if not acq.args.motion_refine:
        return dict(status='not_requested', external_imu_time_correction_s=None)
    initial_i = [r for r in acq.imu if 0 <= acq.elapsed(r) < acq.args.warmup]
    initial_c = [r for r in acq.camera_gyro if 0 <= acq.elapsed(r) < acq.args.warmup]
    if min(len(initial_i), len(initial_c)) < 100:
        return dict(status='insufficient_stationary_baseline', external_imu_time_correction_s=None)
    baselines = []
    for start in np.arange(0, acq.args.warmup-1.9, .5):
        ii = [r['gyro_rads'] for r in initial_i if start <= acq.elapsed(r) < start+2]
        cc = [r['gyro'] for r in initial_c if start <= acq.elapsed(r) < start+2]
        if min(len(ii), len(cc)) < 200:
            continue
        bi, bc = np.asarray(ii), np.asarray(cc)
        score = max(np.std(bi, axis=0).max(), np.std(bc, axis=0).max())
        baselines.append((score, bi, bc))
    if not baselines or min(v[0] for v in baselines) > .03:
        return dict(status='warmup_was_not_stationary', external_imu_time_correction_s=None)
    _, bi, bc = min(baselines, key=lambda v: v[0])
    ir = [r for r in acq.imu if acq.elapsed(r) >= acq.args.warmup]
    cr = [r for r in acq.camera_gyro if acq.elapsed(r) >= acq.args.warmup]
    it = np.array([(model.mono_ns(r['monotonic_us'])-acq.mono_anchor)/1e9 for r in ir])
    ct = np.array([(r['sdk_epoch_ns']-acq.wall_anchor)/1e9 for r in cr])
    iv = np.linalg.norm(np.array([r['gyro_rads'] for r in ir])-bi.mean(axis=0), axis=1)
    cv = np.linalg.norm(np.array([r['gyro'] for r in cr])-bc.mean(axis=0), axis=1)
    if np.any(np.diff(ct) <= 0):
        return dict(status='camera_gyro_clock_not_monotonic', external_imu_time_correction_s=None)
    excitation = dict(external_speed_std_rad_s=float(iv.std()), camera_speed_std_rad_s=float(cv.std()))
    if min(iv.std(), cv.std()) < .10 or min(np.ptp(iv), np.ptp(cv)) < .50:
        return dict(status='insufficient_rotation', external_imu_time_correction_s=None, **excitation)
    margin = acq.args.max_motion_shift
    lo, hi = max(it[0], ct[0])+margin, min(it[-1], ct[-1])-margin
    grid = np.arange(lo, hi, .005)
    if len(grid) < 1000:
        return dict(status='insufficient_motion_overlap', external_imu_time_correction_s=None, **excitation)
    external = np.interp(grid, it, iv)
    active = np.flatnonzero(external > .15)
    if len(active) < 1600 or grid[active[-1]]-grid[active[0]] < 15:
        return dict(status='insufficient_rotation_duration', external_imu_time_correction_s=None, **excitation)
    motion_window = (grid >= grid[active[0]]-.5) & (grid <= grid[active[-1]]+.5)
    grid, external = grid[motion_window], external[motion_window]

    def estimate(mask):
        t, e = grid[mask], external[mask]
        if np.std(e) < .10:
            return None
        e = (e-e.mean())/e.std()
        def cost(shift):
            c = np.interp(t+shift, ct, cv)
            if c.std() < .02:
                return 2.
            return float(1-np.mean(e*(c-c.mean())/c.std()))
        offsets = np.arange(-margin, margin+.0005, .001)
        costs = np.array([cost(v) for v in offsets])
        j = int(np.argmin(costs))
        if j in (0, len(offsets)-1):
            return None
        opt = minimize_scalar(cost, bounds=(offsets[j-1], offsets[j+1]), method='bounded',
                              options={'xatol': 1e-5})
        return dict(shift_s=float(opt.x), correlation=1-float(opt.fun))

    split = (grid[0]+grid[-1])/2
    estimates = [estimate(mask) for mask in (np.ones(len(grid), dtype=bool), grid<split, grid>=split)]
    if any(v is None for v in estimates):
        return dict(status='ambiguous_motion', external_imu_time_correction_s=None, **excitation)
    consistency = abs(estimates[1]['shift_s']-estimates[2]['shift_s'])
    accepted = min(v['correlation'] for v in estimates) >= .90 and consistency <= .005
    return dict(status='estimated_against_camera_gyro' if accepted else 'motion_check_failed',
        external_imu_time_correction_s=estimates[0]['shift_s'] if accepted else None,
        candidates=estimates, split_consistency_ms=consistency*1000,
        note='Relative to camera gyro; camera/IMU internal filter delays remain part of this estimate.',
        **excitation)


def analyze(acq):
    args = acq.args
    end_fit = args.warmup + args.fit_seconds
    steady = [r for r in acq.imu if acq.elapsed(r) >= args.warmup]
    train = [r for r in steady if acq.elapsed(r) < end_fit]
    holdout = [r for r in steady if acq.elapsed(r) >= end_fit]
    if len(train) < args.fit_seconds*150 or len(holdout) < args.validate_seconds*150:
        raise ValueError('External IMU sample coverage is insufficient')
    if any(r['flags'] & 0x23 != 0x23 for r in steady):
        raise ValueError('IMU timestamps are not valid hardware DRDY captures')
    if len({r['epoch_us']-r['monotonic_us'] for r in steady}) != 1:
        raise ValueError('IMU clock origin changed during capture')
    deltas = [b['monotonic_us']-a['monotonic_us'] for a,b in zip(steady, steady[1:])]
    steps = [(b['sequence']-a['sequence']) & 0xffffffff for a,b in zip(steady, steady[1:])]
    if any(d <= 0 for d in deltas) or any(s == 0 or s >= 0x80000000 for s in steps):
        raise ValueError('IMU clock/sequence restarted during capture')
    missing = sum(s-1 for s in steps)
    wall_errors = [r['receive_ns']-r['receive_monotonic_ns']
                   -(acq.wall_anchor-acq.mono_anchor) for r in steady]
    if max(abs(v) for v in wall_errors) > 2_000_000:
        raise ValueError('Host system clock jumped relative to monotonic clock')
    initial = fit_clock(train, acq.wall_anchor, acq.mono_anchor)
    second = fit_clock(holdout, acq.wall_anchor, acq.mono_anchor)
    final = fit_clock(steady, acq.wall_anchor, acq.mono_anchor)
    validation = quantiles(arrival_residual_ms(initial, holdout))
    failures = []
    if abs(final.slope-1) > .005:
        failures.append('implausible clock slope')
    if abs(initial.slope-second.slope)*1e6 > 20:
        failures.append('clock slope changed by >20 ppm')
    if abs(validation['median']) > 1 or validation['p95'] > 3 or validation['min'] < -2:
        failures.append('held-out clock mapping is unstable')
    if missing/max(len(steady), 1) > .001 or max(deltas) > 50_000:
        failures.append('too many missing IMU samples')
    if acq.parser.crc_errors != acq.crc_after_warmup:
        failures.append('serial CRC error after warmup')
    images = [r for r in acq.images if acq.elapsed(r) >= args.warmup]
    if len(images) < (args.fit_seconds+args.validate_seconds)*20:
        failures.append('insufficient image coverage')
    global_delta = [b['sdk_epoch_ns']-a['sdk_epoch_ns'] for a,b in zip(images, images[1:])]
    if any(d <= 0 for d in global_delta):
        failures.append('camera global time is not strictly increasing')
    if not images or any(r['frame_timestamp'] is None or r['sensor_timestamp'] is None for r in images):
        failures.append('camera hardware timestamp metadata is missing')
    if failures:
        raise ValueError('; '.join(failures))
    camera_raw = np.array([r['frame_timestamp']-images[0]['frame_timestamp'] for r in images])/1e6
    if np.any(np.diff(camera_raw) <= 0):
        raise ValueError('Camera hardware clock reset/wrapped; acquire a new calibration segment')
    sdk = np.array([r['sdk_epoch_ns']-images[0]['sdk_epoch_ns'] for r in images])/1e9
    ca, cb = np.linalg.lstsq(np.column_stack((camera_raw, np.ones(len(camera_raw)))), sdk, rcond=None)[0]
    motion = refine_motion(acq, final)
    time_correction = motion['external_imu_time_correction_s']
    before = acq.imu[-1]
    image = images[-1]
    midpoint_shift = image['frame_timestamp']-image['sensor_timestamp']
    mapped = [final.map_us(r['monotonic_us']) for r in steady]
    if any(b <= a for a,b in zip(mapped, mapped[1:])):
        raise ValueError('Mapped IMU timestamps are not strictly increasing')
    return dict(schema_version=1, calibrated_at_beijing=datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),
        clock_mapping_ready=True, measurement_offset_calibrated=time_correction is not None,
        camera=acq.device, port=acq.serial.port, imu_clock_map=asdict(final),
        clock_map_formula='epoch_ns = host_reference_epoch_ns + round(slope * (monotonic_us - device_reference_us) * 1000)',
        clock_slope_ppm=(final.slope-1)*1e6,
        camera_time=dict(domain='global_time', timestamp_ms_to_ns=1000000,
            image_event='sensor exposure midpoint', local_device_to_sdk_slope=float(ca),
            midpoint_formula='frame_global_ns - local_device_to_sdk_slope * (frame_timestamp_us - sensor_timestamp_us) * 1000',
            sdk_mapping_line_residual_ms=quantiles((sdk-ca*camera_raw-cb)*1000)),
        validation=dict(train_samples=len(train), holdout_samples=len(holdout),
            held_out_arrival_residual_ms=validation,
            fitted_arrival_residual_ms=quantiles(arrival_residual_ms(final, steady)),
            train_slope=initial.slope, holdout_slope=second.slope,
            slope_difference_ppm=(second.slope-initial.slope)*1e6,
            host_clock_variation_us=(max(wall_errors)-min(wall_errors))/1000),
        acquisition=dict(imu_samples=len(acq.imu), image_frames=len(acq.images),
            camera_gyro_samples=len(acq.camera_gyro), missing_imu_samples=missing,
            crc_errors_after_warmup=acq.parser.crc_errors-acq.crc_after_warmup,
            imu_dt_us=quantiles(deltas), firmware_status=acq.statuses[-1] if acq.statuses else None),
        motion_refinement=motion,
        examples=dict(last_imu_hardware_us=before['monotonic_us'],
            last_imu_host_ns_approx=final.map_us(before['monotonic_us']),
            last_image_global_ns=image['sdk_epoch_ns'],
            last_image_exposure_midpoint_ns_approx=image['sdk_epoch_ns']-round(ca*midpoint_shift*1000)),
        validity=dict(camera_serial=acq.device.get('camera_info.serial_number'),
            observed_imu_min_us=steady[0]['monotonic_us'], observed_imu_max_us=steady[-1]['monotonic_us'],
            observed_epoch_minus_monotonic_us=before['epoch_us']-before['monotonic_us'],
            instruction='Rerun after reconnect/restart; do not apply this model to another boot. Monitor drift for longer runs.'),
        limits=['Arrival residuals do not measure physical camera-IMU synchronization accuracy.',
            'One-way timestamps leave unknown fixed minimum transport delay.',
            'Image exposure-midpoint conversion is a local approximation of SDK clock mapping.',
            'A camera-gyro motion estimate is not a full visual-inertial Kalibr calibration.'])


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', help='Default: automatically select FIREPHX or the only serial port')
    ap.add_argument('--camera-serial')
    ap.add_argument('--baud', type=int, default=460800)
    ap.add_argument('--decoder', type=Path, default=ROOT.parent/'fake_gps/fake_gps/tools/imu_receive.py')
    ap.add_argument('--warmup', type=float, default=5)
    ap.add_argument('--fit-seconds', type=float, default=30)
    ap.add_argument('--validate-seconds', type=float, default=15)
    ap.add_argument('--motion-refine', action='store_true', help='Requires rigid mounting, still warmup, then varied rotations')
    ap.add_argument('--motion-wait-seconds', type=float, default=90,
                    help='Extra wait for sufficient movement when motion refinement is requested')
    ap.add_argument('--max-motion-shift', type=float, default=.25)
    ap.add_argument('--output', type=Path, default=ROOT/'tool/calibration/camera_imu_time_sync.json')
    ap.add_argument('--no-save', action='store_true')
    args = ap.parse_args()
    if args.warmup < 2 or args.fit_seconds < 10 or args.validate_seconds < 10:
        ap.error('Use warmup>=2, fit-seconds>=10, validate-seconds>=10')
    if not 0 < args.max_motion_shift <= 1:
        ap.error('max-motion-shift must be in (0, 1] seconds')
    if args.motion_wait_seconds < 0:
        ap.error('motion-wait-seconds must be nonnegative')
    try:
        acq = Acquisition(args)
        acq.capture()
        result = analyze(acq)
        if not args.no_save:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            temporary = args.output.with_name(args.output.name + f'.{os.getpid()}.tmp')
            try:
                temporary.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
                if os.geteuid() == 0 and 'SUDO_UID' in os.environ:
                    os.chown(temporary, int(os.environ['SUDO_UID']), int(os.environ['SUDO_GID']))
                os.replace(temporary, args.output)
            finally:
                temporary.unlink(missing_ok=True)
            result['saved_parameters'] = str(args.output.resolve())
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    except (Exception, KeyboardInterrupt) as error:
        print(json.dumps(dict(clock_mapping_ready=False, error=str(error),
            action='Fresh calibration failed; do not use an older saved mapping for this connection.'),
            ensure_ascii=False), file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
