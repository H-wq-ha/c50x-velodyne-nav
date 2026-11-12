"""Capture guidance heuristics; these do not certify calibration accuracy."""
from collections import deque
import math
import numpy as np


def validate_cloud(message):
    """Validate decoded VLP16 points without changing any point or timestamp."""
    fields = {f.name: f for f in message.fields}
    expected = dict(x=7, y=7, z=7, time=7, ring=4)
    if any(k not in fields or fields[k].datatype != v or fields[k].count != 1
           for k, v in expected.items()):
        raise ValueError('点云缺少正确的 x/y/z/time(float32)、ring(uint16) 字段')
    if message.width * message.height < 1000 or message.point_step <= 0:
        raise ValueError('点云有效点数不足 1000')
    if message.row_step < message.width*message.point_step or len(message.data) < message.height*message.row_step:
        raise ValueError('点云数据长度与布局不符')
    if any(fields[k].offset < 0 or fields[k].offset + (2 if k == 'ring' else 4)
           > message.point_step for k in expected):
        raise ValueError('点云字段偏移越界')
    endian = '>' if message.is_bigendian else '<'
    dtype = np.dtype(dict(names=list(expected),
        formats=[endian+('u2' if k == 'ring' else 'f4') for k in expected],
        offsets=[fields[k].offset for k in expected], itemsize=message.point_step))
    points = np.ndarray((message.height, message.width), dtype=dtype,
        buffer=message.data, strides=(message.row_step, message.point_step)).reshape(-1)
    if any(not np.isfinite(points[k]).all() for k in ('x','y','z','time')):
        raise ValueError('点云坐标或逐点时间包含非有限值')
    if (points['time'] < 0).any() or (points['time'] > .2).any() or (points['ring'] > 15).any():
        raise ValueError('点云逐点时间或 ring 越界')
    sample = points[::max(1, len(points)//256)]
    xyz = np.column_stack([sample[k] for k in 'xyz'])
    return np.linalg.eigvalsh(np.cov(xyz.T)).tolist()


class CaptureReadiness:
    def __init__(self, min_seconds=90., min_clouds=600, max_loss_ratio=.01):
        if not math.isfinite(min_seconds) or min_seconds < 30 or min_clouds < 100:
            raise ValueError('采集建议阈值至少为 30 秒、100 帧点云')
        self.min_seconds, self.min_clouds = min_seconds, min_clouds
        self.max_loss_ratio = max_loss_ratio
        self.imu_count = self.cloud_count = self.geometry_count = 0
        self.first_us = self.last_us = self.last_cloud_us = None
        self.valid_seconds = self.accel_active_seconds = 0.
        self.positive = np.zeros(3)
        self.negative = np.zeros(3)
        self.active = np.zeros(3)
        self.sum_w = np.zeros(3)
        self.sum_ww = np.zeros((3,3))
        self.recent = deque()

    def imu(self, epoch_us, accel, gyro):
        dt = min((epoch_us-self.last_us)*1e-6, .006) if self.last_us is not None else 0.
        self.first_us = epoch_us if self.first_us is None else self.first_us
        self.last_us = epoch_us
        self.imu_count += 1
        self.valid_seconds += max(0., dt)
        w = np.array(gyro)
        self.positive += np.where(w > .15, w*dt, 0.)
        self.negative += np.where(w < -.15, -w*dt, 0.)
        self.active += (np.abs(w) > .15)*dt
        self.sum_w += w
        self.sum_ww += np.outer(w,w)
        if abs(np.linalg.norm(accel)-9.80665) > .25:
            self.accel_active_seconds += dt

    def cloud(self, epoch_us, eigenvalues):
        self.last_cloud_us = epoch_us
        self.cloud_count += 1
        if min(eigenvalues) >= .01:
            self.geometry_count += 1

    def summary(self, quality):
        count = max(1, self.imu_count)
        covariance = self.sum_ww/count - np.outer(self.sum_w/count, self.sum_w/count)
        eigenvalues = np.maximum(0, np.linalg.eigvalsh(covariance))
        angle = self.positive+self.negative
        progress = np.minimum.reduce([angle/2., self.positive/.5, self.negative/.5, self.active/3., np.ones(3)])
        accepted = quality['accepted_samples']
        missing = max(quality['missing_samples'], quality['firmware_losses'])
        crc = quality['crc_errors']
        invalid = quality['invalid_samples']+quality['duplicate_samples']
        self.recent.append((self.last_us or 0, accepted, missing, crc, invalid))
        cutoff = (self.last_us or 0)-10_000_000
        while len(self.recent)>2 and self.recent[1][0] <= cutoff:
            self.recent.popleft()
        old = self.recent[0]
        recent_accepted, recent_missing = accepted-old[1], missing-old[2]
        loss_ratio = missing/max(1,accepted+missing)
        recent_loss = recent_missing/max(1,recent_accepted+recent_missing)
        bad_ratio = (crc+invalid)/max(1,accepted+crc+invalid)
        recent_bad = (crc-old[3]+invalid-old[4])/max(1,recent_accepted+crc-old[3]+invalid-old[4])
        reasons = []
        if self.valid_seconds < self.min_seconds: reasons.append('有效采集时长不足')
        if self.imu_count < self.min_seconds*150: reasons.append('有效 IMU 样本不足')
        if self.cloud_count < self.min_clouds: reasons.append('有效点云帧数不足')
        if self.cloud_count/max(.001,self.valid_seconds) < 8: reasons.append('有效点云频率不足 8 Hz')
        if self.geometry_count < self.min_clouds//2: reasons.append('三维结构帧不足，请同时看到墙面和地面')
        for k, score in zip('XYZ',progress):
            if score < 1: reasons.append(f'{k} 轴往复转动不足')
        if eigenvalues[0] < .01: reasons.append('三轴独立旋转激励不足')
        if self.accel_active_seconds < 2: reasons.append('加减速激励不足，请补多方向平移')
        if max(loss_ratio,recent_loss,bad_ratio,recent_bad) > self.max_loss_ratio:
            reasons.append('累计或最近约 10 秒的丢样/坏帧比例超过 1%，请检查链路')
        if self.last_us is None or self.last_cloud_us is None or abs(self.last_us-self.last_cloud_us) > 300_000:
            reasons.append('IMU 与有效点云不够新鲜/未对齐')
        return dict(data_sufficient=not reasons, reasons=reasons,
            valid_seconds=self.valid_seconds, target_seconds=self.min_seconds,
            imu_samples=self.imu_count, cloud_frames=self.cloud_count,
            target_clouds=self.min_clouds, geometry_frames=self.geometry_count,
            rotation_progress_percent={k:round(float(v)*100) for k,v in zip('XYZ',progress)},
            positive_rotation_deg=np.degrees(self.positive).tolist(),
            negative_rotation_deg=np.degrees(self.negative).tolist(),
            gyro_covariance_eigenvalues=eigenvalues.tolist(),
            acceleration_excitation_seconds=self.accel_active_seconds,
            missing_ratio=loss_ratio, recent_missing_ratio=recent_loss,
            bad_frame_ratio=bad_ratio, recent_bad_frame_ratio=recent_bad,
            criterion='采集时长、输入质量与运动/场景启发式；非完整可观测性或真实标定精度证明')
