#!/usr/bin/env python3
"""Convert a new capture and run OA-LICalib in an isolated results directory."""
import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import uuid
from zoneinfo import ZoneInfo

import yaml

ROOT = Path(__file__).resolve().parents[2]


def container_path(path):
    try:
        return str(Path('/project')/path.resolve().relative_to(ROOT))
    except ValueError:
        raise ValueError('结果目录必须位于本项目内，便于容器访问') from None


def prepare(source, out, iterations=5, knot=.05):
    source = source.expanduser().resolve()
    capture_state = None
    if (source/'bag/metadata.yaml').is_file():
        if (source/'capture_failure.json').exists():
            raise ValueError('采集进程提前退出，请检查 capture_failure.json 后重新采集')
        status = source/'frontend/status.json'
        if status.exists():
            state = json.loads(status.read_text())
            if not state.get('ready') or state.get('failure'):
                raise ValueError(f'采集未通过输入检查：{state.get("failure") or "未完成静止初始化"}')
            capture_state = state
        source = source/'bag'
    if not (source/'metadata.yaml').is_file():
        raise ValueError(f'找不到 ROS2 录包 metadata.yaml：{source}')
    if iterations < 2 or not math.isfinite(knot) or knot <= 0:
        raise ValueError('iterations 至少为 2，knot 必须为正数')
    out = out.expanduser().resolve()
    container_path(out)
    python = ROOT/'tool/oa_licalib_venv/bin/python'
    if not python.exists():
        raise ValueError('缺少转换环境，请按 README 中的首次安装步骤准备')
    out.mkdir(parents=True, exist_ok=False)
    target = out/'dataset.bag'
    with (out/'conversion.log').open('w') as log:
        subprocess.run([str(python), str(ROOT/'tools/convert_oa_calibration_bag.py'),
            str(source), str(target), '--imu-csv', str(out/'inputs/imu.csv')],
            stdout=log, stderr=subprocess.STDOUT, check=True)
    evidence = json.loads(target.with_suffix('.conversion.json').read_text())
    if capture_state and capture_state.get('tolerant'):
        if not capture_state.get('capture_readiness',{}).get('data_sufficient'):
            print('提示：本轮尚未满足采集结束建议条件，请结合报告判断是否需要补采。', flush=True)
        for key, topic in [('imu_published','/calibration/imu'),
                           ('pointclouds_published','/calibration/velodyne_points')]:
            expected = capture_state.get(key, 0)
            received = evidence[topic]['count']
            if expected < 1 or received > expected+2 or expected-received > max(2,expected*.01):
                raise ValueError(f'录包接收数量异常：{topic} 前端发布 {expected}，录包收到 {received}')
        (out/'capture_state.json').write_text(json.dumps(capture_state,ensure_ascii=False,indent=2))
    streams = [evidence[name] for name in ('/calibration/imu', '/calibration/velodyne_points')]
    if any(x['count'] < 2 or x['nonincreasing'] for x in streams):
        raise ValueError('有效输入不足或时间未递增，请检查 conversion.log')
    duration = (max(x['last_stamp_ns'] for x in streams)
                - min(x['first_stamp_ns'] for x in streams))*1e-9
    config = yaml.safe_load((ROOT/'tool/oa_calibration/config.yaml').read_text())
    config.update(iteration_num=iterations, knot_distance=knot, segment_num=1,
        selected_segment=[dict(start_time=0, end_time=duration+.05,
                               path_bag=container_path(target))])
    (out/'config.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
    (out/'request.json').write_text(json.dumps(dict(source_bag=str(source),
        output=str(out), hardware_duration_s=duration, iterations=iterations,
        knot_s=knot, expected_counts={k:evidence[k]['count'] for k in
            ('/calibration/imu', '/calibration/velodyne_points')}), indent=2))
    return out


def run_backend(out):
    name = 'oa-calibration-'+uuid.uuid4().hex[:12]
    script = '''set -eo pipefail
source /project/tool/oa_licalib_ws/devel/setup.bash
export LD_LIBRARY_PATH=/project/tool/oa_licalib_deps/lib:${LD_LIBRARY_PATH:-}
roscore > "$2/roscore.log" 2>&1 &
core_pid=$!
trap 'kill "$core_pid" 2>/dev/null || true' EXIT
for attempt in {1..50}; do rosparam list >/dev/null 2>&1 && break; sleep 0.2; done
rosrun oa_licalib li_calib_node _config_path:="$1"
'''
    cmd = ['docker', 'run', '--rm', '--init', '--name', name,
        '--user', f'{os.getuid()}:{os.getgid()}',
        '-e', 'ROS_HOME='+container_path(out/'ros_home'), '-e', 'OMP_NUM_THREADS=8',
        '-e', 'OPENBLAS_NUM_THREADS=1', '-v', str(ROOT)+':/project',
        '-w', '/project/tool/oa_licalib_ws', 'oa-licalib:noetic', 'bash', '-c',
        script, 'bash', container_path(out/'config.yaml'), container_path(out)]
    with (out/'run.log').open('w') as log:
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                   text=True, start_new_session=True)
        try:
            for line in process.stdout:
                log.write(line)
                log.flush()
                print(line, end='', flush=True)
            result = process.wait()
        except KeyboardInterrupt:
            subprocess.run(['docker', 'stop', '--time', '20', name],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            process.wait(timeout=30)
            result = 130
    (out/'run_status.json').write_text(json.dumps(dict(exit_code=result)))
    return result


def finish(out):
    with (out/'analysis.log').open('w') as analysis_log:
        subprocess.run(['/usr/bin/python3', str(ROOT/'tools/evaluate_oa_calibration.py'),
            str(out), '--bag-stem', 'dataset'], stdout=analysis_log,
            stderr=subprocess.STDOUT, check=True)
    data = json.loads((out/'evaluation.json').read_text())
    request = json.loads((out/'request.json').read_text())
    log = (out/'run.log').read_text()
    terminations = re.findall(r'Termination: (\w+)', log)
    stages = len(data['iteration_records'])
    evidence = json.loads((out/'dataset.conversion.json').read_text())
    expected = dict(imu=evidence['/calibration/imu']['count'],
        clouds=evidence['/calibration/velodyne_points']['count']
               - evidence['clouds_before_first_imu'])
    counts = {}
    for key, topic in [('imu', '/calibration/imu'), ('clouds', '/calibration/velodyne_points')]:
        match = re.search(re.escape(topic)+r': (\d+)', log)
        counts[key] = int(match[1]) if match else None
    finite = all(math.isfinite(x) for row in data['final']['T_imu_lidar'] for x in row)
    valid = (stages == request['iterations'] and len(terminations) >= stages
             and all(x == 'CONVERGENCE' for x in terminations)
             and finite and counts == expected)
    check = dict(numerical_test_passed=bool(valid), absolute_accuracy_validated=False,
        requested_iterations=request['iterations'], recorded_iterations=stages,
        solver_terminations=terminations, final=data['final'],
        expected_backend_counts=expected, actual_backend_counts=counts,
        parameters_finite=finite,
        accuracy_limit=data['accuracy_limit'])
    (out/'verification.json').write_text(json.dumps(check, indent=2))
    final = data['final']
    report = f'''# OA-LICalib 本轮离线标定

数值收敛检查：{'通过' if valid else '未通过，请检查 run.log'}。真实外参精度尚未验证。

源录包：`{request['source_bag']}`；硬件时间跨度 {request['hardware_duration_s']:.3f} 秒。

变换方向：`p_imu = R * p_lidar + t`。

| 项目 | 结果 |
|---|---|
| 平移 XYZ / m | `{final['translation_m']}` |
| ROS RPY / deg | `{final['rpy_xyz_deg']}` |
| 时间偏移 / s | `{final['time_offset_s']}`，`t_imu = t_lidar + offset` |

完整矩阵见 `evaluation.json` 中 `final.T_imu_lidar`。外参没有自动应用到 TF。

![诊断图]({out/'diagnostics.png'})

每轮改变是同一份数据上的收敛情况，不代表多次独立标定的重复性或真实安装精度。
'''
    (out/'report.md').write_text(report)
    return 0 if valid else 2


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('bag', type=Path, help='run.sh 采集目录，或任意 ROS2 标定 bag 目录')
    ap.add_argument('--output', type=Path, help='尚不存在的结果目录，默认自动创建')
    ap.add_argument('--iterations', type=int, default=5)
    ap.add_argument('--knot', type=float, default=.05)
    ap.add_argument('--prepare-only', action='store_true', help='只验证并转换数据，不运行优化')
    args = ap.parse_args()
    stamp = datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f')
    out = args.output or ROOT/'output'/f'oa_licalib_{stamp}'
    try:
        if not args.prepare_only:
            subprocess.run(['docker', 'image', 'inspect', 'oa-licalib:noetic'],
                stdout=subprocess.DEVNULL, check=True)
            if not (ROOT/'tool/oa_licalib_ws/devel/lib/oa_licalib/li_calib_node').exists():
                raise ValueError('尚未编译 OA-LICalib，请先运行 tool/build_oa_licalib.sh')
        print(f'本轮结果目录：{out.resolve()}', flush=True)
        out = prepare(args.bag, out, args.iterations, args.knot)
        if args.prepare_only:
            print('转换和配置已完成；本次没有运行优化。', flush=True)
            return 0
        print('开始离线优化。详细日志：'+str(out/'run.log'), flush=True)
        code = run_backend(out)
        if code:
            print('优化未完成，请检查 run.log。', file=sys.stderr)
            return code
        code = finish(out)
        print('报告：'+str(out/'report.md'), flush=True)
        return code
    except (ValueError, OSError, subprocess.SubprocessError) as error:
        print(f'标定未完成：{error}。已有文件保留在 {out}。', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
