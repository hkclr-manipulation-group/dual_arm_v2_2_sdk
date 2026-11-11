"""按固定顺序执行双臂组合运动。每一步两边同时开始，这一步结束后才进入下一步。

左臂 movej 的默认目标角是 103.512, -17.218, -29.020, 95.323, -1.659, 46.885, 8.550。
右臂 movej 的默认目标角是 -98.08, -8.92, 1.46, 79.27, -0.64, 60.30, 0.01。
movel 使用与 dual_two_control_api.py 相同的工具系增量，相对该步开始时的当前位姿。

阶段：
    1. 左 movej，右静止
    2. 左静止，右 movej
    3. 左 movel，右静止
    4. 左 movej，右 movel
    5. 左 movel，右 movej
    6. 左静止，右 movel
    7. 左 movej，右 movej
    8. 左 movel，右 movel
    9. 左 movej，右 movej

用法：
    python3 examples/python/dual_two_control_sequence.py
    python3 examples/python/dual_two_control_sequence.py <config_left> <config_right>
"""

import sys
import threading

import dual_arm_v2_2_sdk as sdk

from dual_two_control_api import (
    DEFAULT_DPOS_M,
    DEFAULT_DRPY_DEG,
    DEFAULT_JOINT_SPEED,
    DEFAULT_LEFT_CONFIG,
    DEFAULT_LINE_ACC_S,
    DEFAULT_LINE_SPEED,
    DEFAULT_RIGHT_CONFIG,
    StartGate,
    enable_arm,
    format_floats,
    log_err,
    log_line,
    parse_floats,
    run_move_joint,
    run_move_line,
    take_value,
)
from example_common import wait_for_joints

STATE_TIMEOUT_S = 5.0
LEFT_JOINTS_DEG = [103.512, -17.218, -29.020, 95.323, -1.659, 46.885, 8.550]
RIGHT_JOINTS_DEG = [-98.08, -8.92, 1.46, 79.27, -0.64, 60.30, 0.01]

# 与图中表格自上而下一致。静止对应 hold，不发新运动。
STAGES = [
    ("movej", "hold"),
    ("hold", "movej"),
    ("movel", "hold"),
    ("movej", "movel"),
    ("movel", "movej"),
    ("hold", "movel"),
    ("movej", "movej"),
    ("movel", "movel"),
    ("movej", "movej"),
]


def print_usage(argv0):
    lines = [
        f"Usage: {argv0} [config_left config_right] [options]",
        f"  config_left                  default {DEFAULT_LEFT_CONFIG}",
        f"  config_right                 default {DEFAULT_RIGHT_CONFIG}",
        "  --joints-left  j1..j7        default "
        + format_floats(LEFT_JOINTS_DEG, 3),
        "  --joints-right j1..j7        default "
        + format_floats(RIGHT_JOINTS_DEG, 2),
        "  --dpos-left/--dpos-right     dx,dy,dz meters in the tool frame",
        f"      default {format_floats(DEFAULT_DPOS_M, 2)}",
        "  --drpy-left/--drpy-right     rx,ry,rz degrees, tool-frame XYZ",
        f"      default {format_floats(DEFAULT_DRPY_DEG, 1)}",
        f"  --speed-joint 0..100         default {DEFAULT_JOINT_SPEED}"
        "; 0 keeps the full joint velocity limit",
        f"  --speed-line  0..100         default {DEFAULT_LINE_SPEED}",
        f"  --acc-line seconds           default {DEFAULT_LINE_ACC_S}"
        "; 0 uses config.yaml acc_time",
        "Stages:",
    ]
    for index, (left_motion, right_motion) in enumerate(STAGES, start=1):
        lines.append(f"  {index}. left {left_motion}, right {right_motion}")
    print("\n".join(lines), file=sys.stderr)


def parse_args(argv):
    left = {
        "label": "left_rt",
        "config": DEFAULT_LEFT_CONFIG,
        "group": sdk.Group.LEFT_ARM,
        "joints_deg": list(LEFT_JOINTS_DEG),
        "dpos_m": list(DEFAULT_DPOS_M),
        "drpy_deg": list(DEFAULT_DRPY_DEG),
    }
    right = {
        "label": "right_rt",
        "config": DEFAULT_RIGHT_CONFIG,
        "group": sdk.Group.RIGHT_ARM,
        "joints_deg": list(RIGHT_JOINTS_DEG),
        "dpos_m": list(DEFAULT_DPOS_M),
        "drpy_deg": list(DEFAULT_DRPY_DEG),
    }
    limits = {
        "joint_speed": DEFAULT_JOINT_SPEED,
        "line_speed": DEFAULT_LINE_SPEED,
        "line_acc_s": DEFAULT_LINE_ACC_S,
    }
    float_targets = {
        "--joints-left": (left, "joints_deg", 7),
        "--joints-right": (right, "joints_deg", 7),
        "--dpos-left": (left, "dpos_m", 3),
        "--dpos-right": (right, "dpos_m", 3),
        "--drpy-left": (left, "drpy_deg", 3),
        "--drpy-right": (right, "drpy_deg", 3),
    }
    help_only = False
    positionals = []
    index = 1
    while index < len(argv):
        arg = argv[index]
        if arg in ("--help", "-h"):
            help_only = True
            index += 1
            continue
        if arg in ("--left", "--right"):
            log_err(f"{arg} is fixed by the stage list; omit it")
            return None
        if arg in float_targets:
            value, index = take_value(argv, index, arg)
            if value is None:
                return None
            numbers = parse_floats(value)
            job, field, count = float_targets[arg]
            if numbers is None:
                log_err(f"{arg} is not a comma-separated number list")
                return None
            if len(numbers) != count:
                log_err(f"{arg} needs {count} comma-separated numbers")
                return None
            job[field] = numbers
            index += 1
            continue
        if arg in ("--speed-joint", "--speed-line", "--acc-line"):
            value, index = take_value(argv, index, arg)
            if value is None:
                return None
            try:
                number = float(value)
            except ValueError:
                log_err(f"{arg} is not a number")
                return None
            if arg == "--acc-line":
                if number < 0.0:
                    log_err("--acc-line must be >= 0")
                    return None
                limits["line_acc_s"] = number
            elif number < 0.0 or number > 100.0:
                log_err(f"{arg} must be 0..100")
                return None
            elif arg == "--speed-joint":
                limits["joint_speed"] = int(number)
            else:
                limits["line_speed"] = int(number)
            index += 1
            continue
        if arg.startswith("--"):
            log_err(f"unknown option {arg}")
            return None
        positionals.append(arg)
        index += 1

    if help_only:
        print_usage(argv[0])
        return ("help", None, None, None) if not positionals else None
    if len(positionals) == 2:
        left["config"] = positionals[0]
        right["config"] = positionals[1]
    elif positionals:
        print_usage(argv[0])
        return None
    return ("run", left, right, limits)


def connect_arm(job, gate, apis, ok):
    try:
        log_line("[%s] ControlApi(%s) %s"
                 % (job["label"], job["config"], job["group"].name))
        api = sdk.ControlApi(job["config"])
        if wait_for_joints(api, job["group"], STATE_TIMEOUT_S) is None:
            log_err("[%s] wait joint state timeout" % job["label"])
            ok["value"] = False
            gate.abort()
            return
        if not enable_arm(api, job):
            ok["value"] = False
            gate.abort()
            return
        if not gate.arrive_and_wait():
            log_err("[%s] cancelled before motion" % job["label"])
            ok["value"] = False
            return
        apis[job["label"]] = api
    except Exception as ex:
        log_err("[%s] exception: %s" % (job["label"], ex))
        ok["value"] = False
        gate.abort()


def run_side(api, job, motion, limits, gate, ok):
    try:
        if not gate.arrive_and_wait():
            log_err("[%s] cancelled before motion" % job["label"])
            ok["value"] = False
            return
        if motion == "hold":
            log_line("[%s] hold" % job["label"])
            moved = True
        elif motion == "movej":
            moved = run_move_joint(api, job, limits["joint_speed"])
        else:
            moved = run_move_line(api, job, limits)
        if not moved:
            ok["value"] = False
            return
        log_line("[%s] done" % job["label"])
    except Exception as ex:
        log_err("[%s] exception: %s" % (job["label"], ex))
        ok["value"] = False
        gate.abort()


def run_parallel(workers):
    ok = {"value": True}
    threads = [threading.Thread(target=worker, args=(ok,)) for worker in workers]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    return ok["value"]


def main(argv):
    parsed = parse_args(argv)
    if parsed is None:
        return 2
    action, left, right, limits = parsed
    if action == "help":
        return 0

    log_line("left joints (deg): " + format_floats(left["joints_deg"], 3))
    log_line("right joints (deg): " + format_floats(right["joints_deg"], 3))
    apis = {}
    gate = StartGate()
    connected = run_parallel([
        lambda ok: connect_arm(left, gate, apis, ok),
        lambda ok: connect_arm(right, gate, apis, ok),
    ])
    if not connected or "left_rt" not in apis or "right_rt" not in apis:
        log_err("One or both arms failed.")
        return 1

    for index, (left_motion, right_motion) in enumerate(STAGES, start=1):
        log_line("stage %d: left %s, right %s"
                 % (index, left_motion, right_motion))
        stage_gate = StartGate()
        finished = run_parallel([
            lambda ok, motion=left_motion: run_side(
                apis["left_rt"], left, motion, limits, stage_gate, ok),
            lambda ok, motion=right_motion: run_side(
                apis["right_rt"], right, motion, limits, stage_gate, ok),
        ])
        if not finished:
            log_err("stage %d failed" % index)
            log_err("One or both arms failed.")
            return 1
        log_line("stage %d finished" % index)

    log_line("Both arms finished.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
