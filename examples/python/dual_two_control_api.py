"""两个 rt_control：各一个 ControlApi，并行测试 MoveJ / MoveL / 静止的任意组合。

左 config 只命令 Group.LEFT_ARM（arm 0）；右 config 只命令 Group.RIGHT_ARM（arm 2）。
默认关节角和工具系增量与 examples/cpp/test_movel.cpp 相同。

用法：
    python3 examples/python/dual_two_control_api.py
    python3 examples/python/dual_two_control_api.py --left movel --right hold
    python3 examples/python/dual_two_control_api.py <config_left> <config_right> \\
        --left movej|movel|hold --right movej|movel|hold

关节角为绝对角度（度）。dpos 为工具系位置增量（米），drpy 为工具系 XYZ 欧拉增量（度）。
不写 --left / --right 时两边都做 movej。静止侧只使能并保持当前关节，不发新运动。
不写配置目录时使用下面的默认路径。
"""

import math
import sys
import threading

import dual_arm_v2_2_sdk as sdk

from example_common import wait_for_joints

STATE_TIMEOUT_S = 5.0
DEFAULT_LEFT_CONFIG = (
    "/home/delia/dual_arm/cuarm_panel_control_left/cuarm_configuration/dual_v2_2")
DEFAULT_RIGHT_CONFIG = (
    "/home/delia/dual_arm/cuarm_panel_control_right/cuarm_configuration/dual_v2_2")

# 与 test_movel.cpp 相同。
DEFAULT_JOINTS_DEG = [103.512, -17.218, -29.020, 95.323, -1.659, 46.885, 8.550]
DEFAULT_DPOS_M = [0.06, 0.06, 0.06]
DEFAULT_DRPY_DEG = [50.0, -40.0, -60.0]
DEFAULT_JOINT_SPEED = 30
DEFAULT_LINE_SPEED = 30
DEFAULT_LINE_ACC_S = 3.0

DEG_TO_RAD = math.pi / 180.0
MOTIONS = ("hold", "movej", "movel")

_LOG_LOCK = threading.Lock()


def log_line(line):
    with _LOG_LOCK:
        print(line, flush=True)


def log_err(line):
    with _LOG_LOCK:
        print(line, file=sys.stderr, flush=True)


def code_name(code):
    return getattr(code, "name", str(code))


def group_name(group):
    return getattr(group, "name", str(group))


def format_floats(values, precision):
    return ", ".join(f"{value:.{precision}f}" for value in values)


class StartGate:
    """两边都使能后一起起步。任一侧提前失败时放行另一侧并让它退出。"""

    def __init__(self):
        self._cond = threading.Condition()
        self._arrived = 0
        self._go = False
        self._cancel = False

    def arrive_and_wait(self):
        with self._cond:
            self._arrived += 1
            if self._arrived >= 2:
                self._go = True
                self._cond.notify_all()
            else:
                while not self._go and not self._cancel:
                    self._cond.wait()
            return not self._cancel

    def abort(self):
        with self._cond:
            self._cancel = True
            self._cond.notify_all()


def euler_xyz_to_quat(rx, ry, rz):
    cx, sx = math.cos(rx * 0.5), math.sin(rx * 0.5)
    cy, sy = math.cos(ry * 0.5), math.sin(ry * 0.5)
    cz, sz = math.cos(rz * 0.5), math.sin(rz * 0.5)
    w = cx * cy * cz - sx * sy * sz
    x = sx * cy * cz + cx * sy * sz
    y = cx * sy * cz - sx * cy * sz
    z = cx * cy * sz + sx * sy * cz
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm > 1.0e-8:
        w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return [w, x, y, z]


def quat_mul(a, b):
    q = [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ]
    norm = math.sqrt(sum(component * component for component in q))
    if norm > 1.0e-8:
        q = [component / norm for component in q]
    return q


def offset_pose(pose, xyz_offset_m, euler_delta_rad):
    """与 test_movel.cpp 相同：位置加增量，姿态右乘工具系 XYZ 欧拉增量。"""
    target = list(pose)
    target[0] += xyz_offset_m[0]
    target[1] += xyz_offset_m[1]
    target[2] += xyz_offset_m[2]
    q_new = quat_mul(target[3:7], euler_xyz_to_quat(*euler_delta_rad))
    target[3:7] = q_new
    return target


def pose_text(pose):
    return ("xyz=(%.4f, %.4f, %.4f) quat=(%.4f, %.4f, %.4f, %.4f)"
            % tuple(pose[:7]))


def print_usage(argv0):
    print(
        f"Usage: {argv0} [config_left config_right] [options]\n"
        f"  config_left                  default {DEFAULT_LEFT_CONFIG}\n"
        f"  config_right                 default {DEFAULT_RIGHT_CONFIG}\n"
        "  --left  movej|movel|hold     default movej, commands LEFT_ARM\n"
        "  --right movej|movel|hold     default movej, commands RIGHT_ARM\n"
        "  --joints-left/--joints-right j1..j7 degrees\n"
        f"      default {format_floats(DEFAULT_JOINTS_DEG, 3)}\n"
        "  --dpos-left/--dpos-right     dx,dy,dz meters in the tool frame\n"
        f"      default {format_floats(DEFAULT_DPOS_M, 2)}\n"
        "  --drpy-left/--drpy-right     rx,ry,rz degrees, tool-frame XYZ\n"
        f"      default {format_floats(DEFAULT_DRPY_DEG, 1)}\n"
        f"  --speed-joint 0..100         default {DEFAULT_JOINT_SPEED}"
        "; 0 keeps the full joint velocity limit\n"
        f"  --speed-line  0..100         default {DEFAULT_LINE_SPEED}\n"
        f"  --acc-line seconds           default {DEFAULT_LINE_ACC_S}"
        "; 0 uses config.yaml acc_time",
        file=sys.stderr)


def parse_floats(text):
    out = []
    for item in text.split(","):
        if not item:
            return None
        try:
            out.append(float(item))
        except ValueError:
            return None
    return out or None


def take_value(argv, index, name):
    if index + 1 >= len(argv):
        log_err(f"{name} needs a value")
        return None, index
    return argv[index + 1], index + 1


def parse_args(argv):
    left = {
        "label": "left_rt",
        "config": "",
        "group": sdk.Group.LEFT_ARM,
        "motion": "movej",
        "joints_deg": list(DEFAULT_JOINTS_DEG),
        "dpos_m": list(DEFAULT_DPOS_M),
        "drpy_deg": list(DEFAULT_DRPY_DEG),
    }
    right = {
        "label": "right_rt",
        "config": "",
        "group": sdk.Group.RIGHT_ARM,
        "motion": "movej",
        "joints_deg": list(DEFAULT_JOINTS_DEG),
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
            value, index = take_value(argv, index, arg)
            if value is None:
                return None
            if value not in MOTIONS:
                log_err(f"{arg} must be movej, movel, or hold")
                return None
            (left if arg == "--left" else right)["motion"] = value
            index += 1
            continue
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
    if not positionals:
        left["config"] = DEFAULT_LEFT_CONFIG
        right["config"] = DEFAULT_RIGHT_CONFIG
        return ("run", left, right, limits)
    if len(positionals) != 2:
        print_usage(argv[0])
        return None
    left["config"] = positionals[0]
    right["config"] = positionals[1]
    return ("run", left, right, limits)


def enable_arm(api, job):
    code = api.set_group(job["group"], True)
    if code != sdk.RetCode.SUCCESS:
        log_err("[%s] set_group(%s, true) -> %s"
                % (job["label"], group_name(job["group"]), code_name(code)))
        return False
    api.set_group(sdk.Group.HEAD, False)
    return True


def run_move_joint(api, job, speed_ratio):
    target = [degree * DEG_TO_RAD for degree in job["joints_deg"]]
    log_line("[%s] move_joint target (deg): %s speed_ratio=%s"
             % (job["label"], format_floats(job["joints_deg"], 3), speed_ratio))
    code = api.move_joint(job["group"], target, False, 0, 0, speed_ratio)
    if code != sdk.RetCode.SUCCESS:
        log_err("[%s] move_joint -> %s" % (job["label"], code_name(code)))
        return False
    return True


def run_move_line(api, job, limits):
    code, pose = api.get_tool_pose(job["group"])
    if code != sdk.RetCode.SUCCESS or len(pose) < 7:
        log_err("[%s] get_tool_pose -> %s" % (job["label"], code_name(code)))
        return False
    euler_rad = [degree * DEG_TO_RAD for degree in job["drpy_deg"]]
    target = offset_pose(pose, job["dpos_m"], euler_rad)
    log_line("[%s] current %s" % (job["label"], pose_text(pose)))
    log_line("[%s] move_line dpos(m)=%s drpy(deg)=%s speed=%s acc=%s"
             % (job["label"], format_floats(job["dpos_m"], 3),
                format_floats(job["drpy_deg"], 1), limits["line_speed"],
                limits["line_acc_s"]))
    log_line("[%s] target %s" % (job["label"], pose_text(target)))
    line = api.move_line(job["group"], target, limits["line_speed"],
                          limits["line_acc_s"])
    if line != sdk.RetCode.SUCCESS:
        log_err("[%s] move_line -> %s" % (job["label"], code_name(line)))
        return False
    return True


def run_arm(job, limits, gate, ok):
    try:
        log_line("[%s] %s  ControlApi(%s) %s"
                 % (job["label"], job["motion"], job["config"],
                    group_name(job["group"])))
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
        if job["motion"] == "hold":
            log_line("[%s] hold" % job["label"])
            moved = True
        elif job["motion"] == "movej":
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


def main(argv):
    parsed = parse_args(argv)
    if parsed is None:
        return 2
    action, left, right, limits = parsed
    if action == "help":
        return 0

    log_line("left %s, right %s" % (left["motion"], right["motion"]))
    gate = StartGate()
    ok = {"value": True}
    threads = [
        threading.Thread(target=run_arm, args=(left, limits, gate, ok)),
        threading.Thread(target=run_arm, args=(right, limits, gate, ok)),
    ]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    if not ok["value"]:
        log_err("One or both arms failed.")
        return 1
    log_line("Both arms finished.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
