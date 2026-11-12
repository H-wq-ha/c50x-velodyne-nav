"""Apply fitted SI calibration once, after raw packet decoding."""
from __future__ import annotations

import json
import math
from dataclasses import replace
from pathlib import Path
import sys

DEFAULT_CALIBRATION = Path(__file__).with_name("imu_calibration.json")
CORRECTED_FIELDS = tuple("corrected_" + name for name in
                         ("ax_mps2", "ay_mps2", "az_mps2", "gx_rads", "gy_rads", "gz_rads"))


class Calibration:
    def __init__(self, path):
        self.path = Path(path).resolve()
        data = json.loads(self.path.read_text(encoding="utf-8"))
        if data["units"] != {"acceleration": "m/s^2", "angular_rate": "rad/s"}:
            raise ValueError("标定参数必须使用 m/s^2 和 rad/s")
        a = self._parameters(data["accelerometer"]["parameters"], 9)
        g = self._parameters(data["gyroscope"]["parameters"], 12)
        if any(v <= 0 for v in (*a[3:6], *g[6:9])):
            raise ValueError("比例因子必须为正")
        self.acc_bias, self.gyro_bias = a[6:9], g[9:12]
        self.acc_matrix = self._scaled(((1, -a[0], a[1]), (0, 1, -a[2]), (0, 0, 1)), a[3:6])
        self.gyro_matrix = self._scaled(((1, -g[0], g[1]), (g[2], 1, -g[3]),
                                         (-g[4], g[5], 1)), g[6:9])

    @staticmethod
    def _parameters(values, length):
        if len(values) != length:
            raise ValueError("标定参数数量错误")
        values = tuple(float(v) for v in values)
        if not all(math.isfinite(v) for v in values):
            raise ValueError("标定参数包含非有限值")
        return values

    @staticmethod
    def _scaled(matrix, scales):
        return tuple(tuple(v * s for v, s in zip(row, scales)) for row in matrix)

    def apply(self, sample):
        a = tuple(v + b for v, b in zip(sample.values[:3], self.acc_bias))
        g = tuple(v - b for v, b in zip(sample.values[3:6], self.gyro_bias))
        values = tuple(sum(x * y for x, y in zip(row, a)) for row in self.acc_matrix)
        values += tuple(sum(x * y for x, y in zip(row, g)) for row in self.gyro_matrix)
        return replace(sample, values=(*values, sample.values[6]))


def add_calibration_args(parser, *, csv_output=False):
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--calibration", default=str(DEFAULT_CALIBRATION),
                       help="标定参数 JSON，默认 tools/imu_calibration.json（calibration03）")
    group.add_argument("--raw", action="store_true", help="关闭标定，显示原始 SI 数据")
    if csv_output:
        parser.add_argument("--corrected-csv", help="保存原始值及 corrected_* 校正值到新 CSV")


def load_calibration(args):
    if args.raw:
        if getattr(args, "corrected_csv", None):
            raise ValueError("--raw 不能与 --corrected-csv 同时使用")
        print("数据模式：原始 SI 值，未应用标定。", file=sys.stderr)
        return None
    calibration = Calibration(args.calibration)
    print(f"数据模式：已校正（零偏、比例因子、轴间误差）；参数：{calibration.path}", file=sys.stderr)
    return calibration


def corrected_row(raw_row, corrected_sample):
    return {**raw_row, **dict(zip(CORRECTED_FIELDS, corrected_sample.values[:6]))}
