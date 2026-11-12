"""单臂蓝莓采摘流程：sensing → MoveL 伸出 → MoveJ 回 sensing → place（暂跳过 init）。

关节角单位为弧度。默认左臂，单个 ControlApi。每段 speed_ratio / acc_time
在文件顶部 SEGMENT_SPEEDS 中配置，也可用命令行覆盖。

用法：
    python3 examples/python/blueberry_pick.py
        # 默认左臂 + DEFAULT_LEFT_CONFIG
    python3 examples/python/blueberry_pick.py --arm right
        # 右臂 + DEFAULT_RIGHT_CONFIG
    python3 examples/python/blueberry_pick.py /path/to/cuarm_configuration/dual_v2_2
    python3 examples/python/blueberry_pick.py --speed-joint-to-sensing 50
"""

import argparse
import math
import sys
import time

import dual_arm_v2_2_sdk as sdk

from example_common import check, config_path, wait_for_joints

DEFAULT_LEFT_CONFIG = (
    "/home/delia/dual_arm/cuarm_panel_control_left/cuarm_configuration/dual_v2_2")
DEFAULT_RIGHT_CONFIG = (
    "/home/delia/dual_arm/cuarm_panel_control_right/cuarm_configuration/dual_v2_2")

# --- 路点（rad）---
# INIT_POSE = [1.57, -0.52, -0.0, 1.72, 0.0, 1.3, 0.0]
SENSING_POSE = [1.57, -0.197, -0.197, 1.52, 0.12, 1.03, 0.174]
PLACE_POSE = [0.5909, -0.2404, -0.1558, 1.9624, 0.1199, 1.1174, 0.1201]

# MoveL 往返增量，与 dual_two_control_api.py 默认一致（位置 m，欧拉 deg，工具系 XYZ）。
DPOS_M = [0.0, 0.10, 0.0]
DRPY_DEG = [0.0, 0.0, 0.0]

# 每段速度。joint_speed / line_speed 为 move_joint / move_line 的 speed_ratio（0..100）。
# line_acc 为 move_line 的 acc_time（秒）；0 表示用 config.yaml。
SEGMENT_SPEEDS = {
    "to_init": {"joint_speed": 100},
    "to_sensing": {"joint_speed": 100},
    "movel_out": {"line_speed": 100, "line_acc": 0.0},
    "back_to_sensing": {"joint_speed": 50},
    "to_place": {"joint_speed": 100},
}

ARMS = {
    "left": sdk.Group.LEFT_ARM,
    "right": sdk.Group.RIGHT_ARM,
}

DEG_TO_RAD = math.pi / 180.0


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
    norm = math.sqrt(sum(c * c for c in q))
    if norm > 1.0e-8:
        q = [c / norm for c in q]
    return q


def offset_pose(pose, xyz_offset_m, euler_delta_rad):
    target = list(pose)
    target[0] += xyz_offset_m[0]
    target[1] += xyz_offset_m[1]
    target[2] += xyz_offset_m[2]
    target[3:7] = quat_mul(pose[3:7], euler_xyz_to_quat(*euler_delta_rad))
    return target


class Timings:
    def __init__(self):
        self.rows = []

    def add(self, name, seconds):
        self.rows.append((name, seconds))
        print(f"[time] {name}: {seconds:.3f} s")

    def total(self):
        return sum(sec for _, sec in self.rows)

    def print_summary(self):
        print("\n=== 分段耗时 ===")
        for name, sec in self.rows:
            print(f"  {name:<22} {sec:8.3f} s")
        print(f"  {'TOTAL':<22} {self.total():8.3f} s")


def timed_move_joint(api, group, joints, speed_ratio, name, timings):
    t0 = time.monotonic()
    code = api.move_joint(group, joints, False, 0, 0, speed_ratio)
    elapsed = time.monotonic() - t0
    timings.add(name, elapsed)
    return check(code, name)


def print_tool_pose_delta_mm(api, group, target_pose, label):
    """超时后打印目标位姿与当前 get_tool_pose 的位置差（mm）：Δ = target − feedback。"""
    code, current = api.get_tool_pose(group)
    if code != sdk.RetCode.SUCCESS or len(current) < 3 or len(target_pose) < 3:
        log_err = f"[{label}] TIMEOUT: get_tool_pose -> {code_name(code)}"
        print(log_err, file=sys.stderr)
        return
    dx_mm = (target_pose[0] - current[0]) * 1000.0
    dy_mm = (target_pose[1] - current[1]) * 1000.0
    dz_mm = (target_pose[2] - current[2]) * 1000.0
    print(
        f"[{label}] TIMEOUT: Δx={dx_mm:.3f} mm Δy={dy_mm:.3f} mm Δz={dz_mm:.3f} mm "
        f"(movel_target − get_tool_pose)",
        file=sys.stderr,
    )


def code_name(code):
    return getattr(code, "name", str(code))


def timed_move_line(api, group, pose, line_speed, line_acc, name, timings):
    t0 = time.monotonic()
    code = api.move_line(group, pose, line_speed, line_acc)
    elapsed = time.monotonic() - t0
    timings.add(name, elapsed)
    if code == sdk.RetCode.TIMEOUT:
        print_tool_pose_delta_mm(api, group, pose, name)
    return check(code, name)


def parse_args(argv):
    parser = argparse.ArgumentParser(description="单臂蓝莓采摘流程示例")
    parser.add_argument("config", nargs="?", help="含 config.yaml 的配置目录")
    parser.add_argument("--arm", choices=tuple(ARMS), default="left")
    parser.add_argument("--speed-joint-to-init", type=int,
                        default=SEGMENT_SPEEDS["to_init"]["joint_speed"])
    parser.add_argument("--speed-joint-to-sensing", type=int,
                        default=SEGMENT_SPEEDS["to_sensing"]["joint_speed"])
    parser.add_argument("--speed-line-out", type=int,
                        default=SEGMENT_SPEEDS["movel_out"]["line_speed"])
    parser.add_argument("--speed-joint-back", type=int,
                        default=SEGMENT_SPEEDS["back_to_sensing"]["joint_speed"])
    parser.add_argument("--acc-line-out", type=float,
                        default=SEGMENT_SPEEDS["movel_out"]["line_acc"])
    parser.add_argument("--speed-joint-to-place", type=int,
                        default=SEGMENT_SPEEDS["to_place"]["joint_speed"])
    return parser.parse_args(argv[1:])


def resolve_config(arm_name, explicit):
    if explicit:
        return config_path(["", explicit])
    return DEFAULT_LEFT_CONFIG if arm_name == "left" else DEFAULT_RIGHT_CONFIG


def main():
    args = parse_args(sys.argv)
    speeds = {
        "to_init": args.speed_joint_to_init,
        "to_sensing": args.speed_joint_to_sensing,
        "movel_out": (args.speed_line_out, args.acc_line_out),
        "back_to_sensing": args.speed_joint_back,
        "to_place": args.speed_joint_to_place,
    }

    config = resolve_config(args.arm, args.config)
    group = ARMS[args.arm]
    api = sdk.ControlApi(config)
    timings = Timings()

    if wait_for_joints(api, group, 5.0) is None:
        print("等待关节反馈超时。rt_control 是否已启动？", file=sys.stderr)
        return 1
    if not check(api.set_group(group, True), "set_group"):
        return 1
    api.set_group(sdk.Group.HEAD, False)

    print(f"arm={args.arm} config={config}")
    t_all = time.monotonic()

    # if not timed_move_joint(api, group, INIT_POSE, speeds["to_init"],
    #                         "to_init", timings):
    #     return 1
    if not timed_move_joint(api, group, SENSING_POSE, speeds["to_sensing"],
                            "to_sensing", timings):
        return 1

    code, sensing_tool = api.get_tool_pose(group)
    if not check(code, "get_tool_pose@sensing") or len(sensing_tool) < 7:
        return 1
    euler_rad = [d * DEG_TO_RAD for d in DRPY_DEG]
    movel_target = offset_pose(sensing_tool, DPOS_M, euler_rad)
    line_out_sp, line_out_acc = speeds["movel_out"]
    if not timed_move_line(api, group, movel_target, line_out_sp, line_out_acc,
                           "movel_out", timings):
        return 1
    if not timed_move_joint(api, group, SENSING_POSE, speeds["back_to_sensing"],
                            "back_to_sensing", timings):
        return 1

    if not timed_move_joint(api, group, PLACE_POSE, speeds["to_place"],
                            "to_place", timings):
        return 1

    wall = time.monotonic() - t_all
    timings.print_summary()
    print(f"  {'wall_clock':<22} {wall:8.3f} s")
    print("蓝莓采摘流程完成")
    return 0


if __name__ == "__main__":
    sys.exit(main())
