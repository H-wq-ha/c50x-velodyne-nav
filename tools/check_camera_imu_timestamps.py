#!/usr/bin/env python3
"""Record live RealSense and fake_gps timestamps without replacing device times."""
import argparse
import csv
import importlib.util
import json
from pathlib import Path
import statistics
import threading
import time
from datetime import datetime
from zoneinfo import ZoneInfo

import pyrealsense2 as rs
import serial
import cv2
import numpy as np


def stats(values):
    if not values:
        return None
    return dict(min=min(values), median=statistics.median(values), max=max(values))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=30)
    parser.add_argument('--port', default='/dev/serial/by-id/usb-FIREPHX_USB_SER_FX2348N-if00')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    spec = importlib.util.spec_from_file_location('fake_gps_receiver',
        '/home/y/arm_car/fake_gps/fake_gps/tools/imu_receive.py')
    receiver = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(receiver)
    decoder = receiver.FrameParser()
    stop = threading.Event()
    imu_rows, status_rows, errors = [], [], []
    camera_rows, seen = [], set()
    sample_images = {}
    camera_metadata = {}
    stream = serial.Serial(port=None, baudrate=460800, timeout=0.05, exclusive=True)
    stream.dtr = False
    stream.rts = False
    stream.port = args.port
    pipeline = rs.pipeline()
    started_pipeline = False
    thread = None
    started = time.monotonic()
    start_label = datetime.now(ZoneInfo('Asia/Shanghai')).isoformat()

    def read_serial():
        try:
            with (args.output / 'serial_rx.bin').open('wb') as raw:
                while not stop.is_set():
                    data = stream.read(max(1, min(stream.in_waiting, 32768)))
                    host, mono = time.time_ns(), time.monotonic_ns()
                    if not data:
                        continue
                    raw.write(data)
                    for item in decoder.feed(data):
                        item.update(receive_ns=host, receive_monotonic_ns=mono)
                        if item['kind'] == 'imu':
                            imu_rows.append(item)
                        else:
                            status_rows.append(item)
        except Exception as error:
            errors.append('serial: ' + repr(error))
            stop.set()

    try:
        stream.open()
        thread = threading.Thread(target=read_serial, daemon=True)
        thread.start()
        config = rs.config()
        config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
        profile = pipeline.start(config)
        started_pipeline = True
        device = profile.get_device()
        camera_metadata['device'] = {str(key): device.get_info(key) for key in
            (rs.camera_info.name, rs.camera_info.serial_number, rs.camera_info.firmware_version)
            if device.supports(key)}
        camera_metadata['sensors'] = []
        for sensor in device.query_sensors():
            info = {'name': sensor.get_info(rs.camera_info.name)}
            if sensor.supports(rs.option.global_time_enabled):
                info['global_time_enabled'] = sensor.get_option(rs.option.global_time_enabled)
            camera_metadata['sensors'].append(info)
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline and not stop.is_set():
            frames = pipeline.wait_for_frames(2000)
            host, mono = time.time_ns(), time.monotonic_ns()
            for frame in frames:
                stream_name = str(frame.profile.stream_type())
                index = frame.profile.stream_index()
                number = frame.get_frame_number()
                key = (stream_name, index, number)
                if key in seen:
                    continue
                seen.add(key)
                item = dict(stream=stream_name, stream_index=index, frame_number=number,
                    timestamp_ms=frame.get_timestamp(), domain=str(frame.get_frame_timestamp_domain()),
                    receive_ns=host, receive_monotonic_ns=mono)
                for name in ('frame_timestamp', 'sensor_timestamp', 'backend_timestamp', 'actual_exposure'):
                    field = getattr(rs.frame_metadata_value, name)
                    item[name + '_raw'] = frame.get_frame_metadata(field) if frame.supports_frame_metadata(field) else None
                camera_rows.append(item)
                if stream_name not in sample_images:
                    image = np.asanyarray(frame.get_data()).copy()
                    if frame.profile.stream_type() == rs.stream.color:
                        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                    sample_images[stream_name] = image
    except Exception as error:
        errors.append('camera/main: ' + repr(error))
    finally:
        stop.set()
        if thread is not None:
            thread.join(timeout=2)
        if started_pipeline:
            pipeline.stop()
        if stream.is_open:
            stream.close()

    for name, image in sample_images.items():
        cv2.imwrite(str(args.output / ('sample_' + name.replace('stream.', '') + '.png')), image)
    for filename, rows in [('imu.jsonl', imu_rows), ('firmware_status.jsonl', status_rows)]:
        with (args.output / filename).open('w') as output:
            for row in rows:
                output.write(json.dumps(row) + '\n')
    with (args.output / 'camera.csv').open('w', newline='') as output:
        if camera_rows:
            writer = csv.DictWriter(output, fieldnames=list(camera_rows[0]))
            writer.writeheader()
            writer.writerows(camera_rows)
    camera_summary = {}
    for name in sorted({row['stream'] for row in camera_rows}):
        rows = [row for row in camera_rows if row['stream'] == name]
        # Group domains: startup can transition from hardware_clock to global_time.
        domains = {}
        for domain in sorted({r['domain'] for r in rows}):
            group = [row for row in rows if row['domain'] == domain]
            delta = [b['timestamp_ms']-a['timestamp_ms'] for a,b in zip(group, group[1:])]
            domains[domain] = dict(frames=len(group), first_ms=group[0]['timestamp_ms'],
                last_ms=group[-1]['timestamp_ms'], dt_ms=stats(delta),
                nonincreasing=sum(value <= 0 for value in delta))
        camera_summary[name] = dict(frames=len(rows), domains=domains,
            frame_number_gaps=sum(max(0,b['frame_number']-a['frame_number']-1)
                                 for a,b in zip(rows, rows[1:])),
            frame_number_resets=sum(b['frame_number']<=a['frame_number']
                                    for a,b in zip(rows, rows[1:])))
        steady = [row for row in rows if row['receive_monotonic_ns'] - rows[0]['receive_monotonic_ns'] > 5e9]
        if len(steady) > 1 and len({row['domain'] for row in steady}) == 1:
            delta = [b['timestamp_ms']-a['timestamp_ms'] for a,b in zip(steady,steady[1:])]
            camera_summary[name]['steady_after_5s'] = dict(frames=len(steady), dt_ms=stats(delta),
                nonincreasing=sum(value <= 0 for value in delta),
                frame_number_gaps=sum(max(0,b['frame_number']-a['frame_number']-1)
                                     for a,b in zip(steady,steady[1:])))
    imu_summary = dict(samples=len(imu_rows), crc_errors=decoder.crc_errors)
    if len(imu_rows) > 1:
        dt = [b['monotonic_us']-a['monotonic_us'] for a,b in zip(imu_rows, imu_rows[1:])]
        duration = (imu_rows[-1]['monotonic_us']-imu_rows[0]['monotonic_us'])/1e6
        imu_summary.update(device_hz=(len(imu_rows)-1)/duration if duration > 0 else None,
            dt_us=stats(dt), nonincreasing=sum(value <= 0 for value in dt),
            missing_sequences=sum(max(0, ((b['sequence']-a['sequence']) & 0xffffffff)-1)
                for a,b in zip(imu_rows, imu_rows[1:])),
            valid_capture_samples=sum(row['flags'] & 3 == 3 for row in imu_rows),
            epoch_matches_monotonic=len({row['epoch_us']-row['monotonic_us'] for row in imu_rows}) == 1,
            first=imu_rows[0], last=imu_rows[-1])
    report = dict(started_beijing=start_label, elapsed_seconds=time.monotonic()-started,
        port=args.port, camera=camera_metadata, camera_streams=camera_summary,
        imu=imu_summary, firmware_status=status_rows[-1] if status_rows else None,
        host_timestamp_semantics='Application receive time per serial read or frameset; includes buffering.',
        errors=errors)
    (args.output/'summary.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    return 1 if errors or not imu_rows or not camera_rows else 0


if __name__ == '__main__':
    raise SystemExit(main())
