"""Hardware clock and fixed IMU correction; no ROS or host clock dependency."""
import binascii
import json
import math
import statistics
import struct
from pathlib import Path

HOUR_US = 3_600_000_000
SAMPLE = struct.Struct('<IQQII7fII')


class FrameParser:
    def __init__(self):
        self.buffer = bytearray()
        self.crc_errors = 0
        self.startup_crc_errors = None

    def feed(self, data):
        self.buffer.extend(data)
        records = []
        while True:
            start = self.buffer.find(b'\xa5\x5a')
            if start < 0:
                self.buffer[:] = b'\xa5' if self.buffer.endswith(b'\xa5') else b''
                break
            del self.buffer[:start]
            if len(self.buffer) < 6:
                break
            if self.buffer[2:3] != b'\x02' or self.buffer[3] not in (1, 2) or self.buffer[4:6] != b'\x40\x00':
                del self.buffer[0]
                continue
            if len(self.buffer) < 72:
                break
            frame = bytes(self.buffer[:72])
            if binascii.crc_hqx(frame[2:70], 0xFFFF) != int.from_bytes(frame[70:], 'little'):
                self.crc_errors += 1
                del self.buffer[0]
                continue
            del self.buffer[:72]
            if frame[3] == 2:
                records.append(dict(kind='status', values=struct.unpack_from('<16I', frame, 6)))
                continue
            if self.startup_crc_errors is None:
                self.startup_crc_errors = self.crc_errors
            f = SAMPLE.unpack_from(frame, 6)
            records.append(dict(kind='imu', sequence=f[0], monotonic_us=f[1], epoch_us=f[2],
                                hour_us=f[3], pps_count=f[4], accel=f[5:8], gyro=f[8:11],
                                temperature=f[11], flags=f[12], lost_samples=f[13]))
        return records


def lidar_epoch_us(hour_us, imu_epoch_us):
    """Recover the packet's hour exclusively from the shared hardware epoch."""
    if not 0 <= hour_us < HOUR_US:
        raise ValueError('雷达硬件小时计数越界')
    base = imu_epoch_us // HOUR_US * HOUR_US + hour_us
    result = min((base - HOUR_US, base, base + HOUR_US), key=lambda t: abs(t - imu_epoch_us))
    if abs(result - imu_epoch_us) > 500_000:
        raise ValueError('雷达与 IMU 硬件时间差超过 0.5 秒；检查 PPS/GPRMC 和同一授时板')
    return result


def packet_hour_us(data):
    if len(data) != 1206 or data[1205] != 0x22:
        raise ValueError('需要 VLP-16 原始 1206 字节数据包')
    if any(bytes(data[i:i + 2]) != b'\xff\xee' for i in range(0, 1200, 100)):
        raise ValueError('VLP-16 数据块头错误')
    return int.from_bytes(bytes(data[1200:1204]), 'little')


class ImuCorrection:
    def __init__(self, path, seconds=10.0, tolerate_gaps=False, max_gap_us=50_000):
        self.params = json.loads(Path(path).read_text())
        if self.params['units'] != {'acceleration': 'm/s^2', 'angular_rate': 'rad/s'}:
            raise ValueError('历史标定参数必须使用 SI 单位')
        a = self.params['accelerometer']['parameters']
        if len(a) != 9 or not all(math.isfinite(v) for v in a):
            raise ValueError('加速度标定参数错误')
        self.bias_acc = tuple(a[6:9])
        self.matrix_acc = ((a[3], -a[0] * a[4], a[1] * a[5]),
                           (0, a[4], -a[2] * a[5]), (0, 0, a[5]))
        self.seconds = seconds
        self.samples = []
        self.acc_samples = []
        self.first_us = None
        self.previous = None
        self.bias_gyro = None
        self.stationary_metrics = None
        self.last_epoch_us = None
        self.tolerate_gaps = tolerate_gaps
        self.max_gap_us = max_gap_us
        self.accepted_samples = 0
        self.missing_samples = self.gap_events = self.invalid_samples = 0
        self.duplicate_samples = self.firmware_losses = self.largest_gap_us = 0
        self.last_drop_reason = None

    @property
    def ready(self):
        return self.bias_gyro is not None

    def correct_accel(self, accel):
        v = tuple(x + b for x, b in zip(accel, self.bias_acc))
        return tuple(sum(x * y for x, y in zip(row, v)) for row in self.matrix_acc)

    def accept(self, record):
        self.last_drop_reason = None
        # Require valid mapping, DRDY hardware capture, HSE, and live IMU.
        if record['flags'] & 0x27 != 0x27 or record['pps_count'] < 1:
            if self.tolerate_gaps:
                self.invalid_samples += 1
                self.last_drop_reason = '缺少有效授时/硬件捕获/在线标志，丢弃本条'
                return None
            raise ValueError('IMU 缺少硬件捕获/有效授时/HSE/在线标志')
        t = record['epoch_us']
        if t <= 0 or record['hour_us'] != t % HOUR_US:
            raise ValueError('IMU epoch 与小时计数不一致')
        if not all(math.isfinite(x) for x in (*record['accel'], *record['gyro'])):
            if self.tolerate_gaps:
                self.invalid_samples += 1
                self.last_drop_reason = 'IMU 六轴包含非有限值，丢弃本条'
                return None
            raise ValueError('IMU 六轴存在无效数值')
        if self.previous is not None:
            dt = t - self.previous['epoch_us']
            dm = record['monotonic_us'] - self.previous['monotonic_us']
            step = (record['sequence'] - self.previous['sequence']) & 0xFFFFFFFF
            if self.tolerate_gaps and dt == dm == step == 0:
                self.duplicate_samples += 1
                self.last_drop_reason = '重复的 IMU 帧，丢弃本条'
                return None
            if dt <= 0 or dt != dm:
                raise ValueError('IMU 硬件时间重启、倒退或时基跳变；重新启动本次标定')
            if self.tolerate_gaps:
                if step == 0 or step >= 0x80000000:
                    raise ValueError('IMU 序号重启或倒退；重新启动本次标定')
                if dt > self.max_gap_us:
                    raise ValueError(f'IMU 有效采样间隔 {dt/1000:.2f} ms 超过 '
                                     f'{self.max_gap_us/1000:g} ms，停止本轮采集')
                lost = (record['lost_samples'] - self.previous['lost_samples']) & 0xFFFFFFFF
                if lost >= 0x80000000:
                    raise ValueError('固件丢样计数重置；重新启动本次标定')
                if step > 1:
                    if not 4000 <= dt / step <= 6000:
                        raise ValueError('IMU 跳号数量与 200 Hz 硬件采样时间不符')
                    self.missing_samples += step - 1
                    self.gap_events += 1
                self.firmware_losses += lost
                self.largest_gap_us = max(self.largest_gap_us, dt)
            elif step != 1:
                raise ValueError('IMU 缺失采样序号；检查串口/采集负载后重新启动')
            if not self.tolerate_gaps and (dt > 50_000 or record['lost_samples'] != self.previous['lost_samples']):
                raise ValueError('IMU 采样中断或固件新增丢帧')
        self.previous = record
        self.last_epoch_us = t
        self.accepted_samples += 1
        a = self.correct_accel(record['accel'])
        if not self.ready:
            if self.first_us is None:
                self.first_us = t
            # Use [first, first + 10 s); never include the next moving sample.
            if t - self.first_us < int(self.seconds * 1_000_000):
                self.samples.append(record['gyro'])
                self.acc_samples.append(a)
                return None
            if len(self.samples) < self.seconds * 150:
                raise ValueError('前 10 秒有效 IMU 样本不足 150 Hz')
            self.bias_gyro = tuple(statistics.fmean(v) for v in zip(*self.samples))
            gyro_stds = tuple(statistics.stdev(v) for v in zip(*self.samples))
            acc_stds = tuple(statistics.stdev(v) for v in zip(*self.acc_samples))
            mean_acc = tuple(statistics.fmean(v) for v in zip(*self.acc_samples))
            mean_norm = math.sqrt(sum(x*x for x in mean_acc))
            self.stationary_metrics = dict(gyro_std_rad_s=gyro_stds, acc_std_m_s2=acc_stds,
                mean_acc_norm_m_s2=mean_norm, gyro_std_limit_rad_s=0.01,
                acc_std_limit_m_s2=0.15, gravity_deviation_limit_m_s2=0.5)
            reasons = []
            for axis, gyro_std, acc_std in zip('XYZ', gyro_stds, acc_stds):
                if gyro_std > 0.01:
                    reasons.append(f'{axis} 轴角速度标准差 {gyro_std:.5f} rad/s > 0.01000')
                if acc_std > 0.15:
                    reasons.append(f'{axis} 轴加速度标准差 {acc_std:.4f} m/s² > 0.1500')
            if abs(mean_norm - 9.80665) > 0.5:
                reasons.append(f'平均加速度模长 {mean_norm:.4f} m/s² 偏离重力超过 0.5')
            if reasons:
                self.bias_gyro = None
                raise ValueError('前 10 秒静止检查失败；'+'；'.join(reasons)+'；请检查静止状态、底座紧固和振动后重新启动')
        # Deliberately ignore historical gyro bias, scale, and misalignment.
        g = tuple(x - b for x, b in zip(record['gyro'], self.bias_gyro))
        return a, g

    def summary(self):
        return dict(accelerometer=self.params['accelerometer'],
                    stationary_check=self.stationary_metrics,
                    gyro_bias_rads=self.bias_gyro, gyro_samples=len(self.samples),
                    gyro_window_seconds=self.seconds, first_epoch_us=self.first_us,
                    last_epoch_us=self.last_epoch_us, timestamp_source='STM32 DRDY capture + PPS epoch')
