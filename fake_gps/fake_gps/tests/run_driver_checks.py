#!/usr/bin/env python3
"""在电脑上检查真实驱动的连续读取和解析逻辑；不连接或写入MCU。"""
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
compiler = shutil.which("cc")
if compiler is None:
    raise SystemExit("需要电脑上的 C 编译器（cc/gcc）。")
with tempfile.TemporaryDirectory(prefix="imu-burst-check-") as folder:
    executable = Path(folder) / "driver_check"
    subprocess.run([
        compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
        "-fsanitize=undefined", "-fno-sanitize-recover=undefined",
        "-I" + str(root / "tests/native_stubs"), "-I" + str(root / "include"),
        str(root / "tests/test_mpu6050_burst.c"),
        str(root / "src/mpu6050.c"), str(root / "src/imu_units.c"),
        "-lm", "-o", str(executable),
    ], check=True)
    subprocess.run([str(executable)], check=True)
