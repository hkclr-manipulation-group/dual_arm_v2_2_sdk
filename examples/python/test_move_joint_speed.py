"""单臂 move_joint 速度对比。低跟随下 speed_ratio 缩放关节速度软限位。

每次先走到目标角，再回到开始时的关节角，并打印两段耗时。
0 表示 100% 软限位，1..100 按百分比缩放，大于 100 会被控制器拒绝。

改下面两个常量，或用命令行覆盖：

    python3 examples/python/test_move_joint_speed.py
    python3 examples/python/test_move_joint_speed.py --arm right --speeds 20,50,100
    python3 examples/python/test_move_joint_speed.py --joints 10,10,10,45,5,5,5
    python3 examples/python/test_move_joint_speed.py /path/to/cuarm_configuration/dual_v2_2
"""

import argparse
import math
import sys
import time

import dual_arm_v2_2_sdk as sdk

from example_common import check, config_path, wait_for_joints

# 目标关节角，单位：度。dual_v2_2 单臂 7 个关节。
TARGET_DEG = [10.0, 10.0, 10.0, 15.0, 5.0, 5.0, 5.0]
# 依次测试的 speed_ratio。
SPEED_RATIOS = [40, 50, 60]

ARMS = {
    "left": sdk.Group.LEFT_ARM,
    "right": sdk.Group.RIGHT_ARM,
}


def deg_to_rad(degrees):
    return [value * math.pi / 180.0 for value in degrees]


def format_deg(joints):
    return ", ".join(f"{value * 180.0 / math.pi:.2f}" for value in joints)


def parse_number_list(text, cast):
    values = []
    for item in text.split(","):
        item = item.strip()
        if not item:
            continue
        values.append(cast(item))
    return values


def parse_args():
    parser = argparse.ArgumentParser(description="单臂 move_joint 不同速度对比")
    parser.add_argument("config", nargs="?", help="含 config.yaml 的配置目录")
    parser.add_argument("--arm", choices=tuple(ARMS), default="left",
                        help="要动的臂，默认 left")
    parser.add_argument("--joints",
                        help="目标关节角（度，逗号分隔）。默认脚本里的 TARGET_DEG")
    parser.add_argument("--speeds",
                        help="speed_ratio 列表（逗号分隔，0..100）。默认脚本里的 SPEED_RATIOS")
    return parser.parse_args()


def move_and_time(api, group, target_rad, speed_ratio, label):
    started = time.monotonic()
    code = api.move_joint(group, target_rad, False, 0, 0, speed_ratio)
    elapsed = time.monotonic() - started
    if not check(code, label):
        return None
    print(f"  {label}: {elapsed:.2f} s")
    return elapsed


def main():
    args = parse_args()
    if args.joints:
        target_deg = parse_number_list(args.joints, float)
    else:
        target_deg = list(TARGET_DEG)
    if args.speeds:
        speed_ratios = parse_number_list(args.speeds, int)
    else:
        speed_ratios = list(SPEED_RATIOS)
    if not target_deg or not speed_ratios:
        print("目标角和速度列表都不能为空", file=sys.stderr)
        return 1
    if any(speed < 0 or speed > 100 for speed in speed_ratios):
        print("speed_ratio 必须在 0..100", file=sys.stderr)
        return 1

    group = ARMS[args.arm]
    config = config_path(["", args.config] if args.config else [""])
    api = sdk.ControlApi(config)
    if wait_for_joints(api, group) is None:
        print("3 秒内没有收到状态。rt_control 是否已启动？", file=sys.stderr)
        return 1
    if not check(api.set_group(group, True), f"使能{args.arm}臂"):
        return 1

    start = wait_for_joints(api, group)
    if start is None or len(start) != len(target_deg):
        print(f"目标角应有 {0 if start is None else len(start)} 个，"
              f"实际 {len(target_deg)} 个", file=sys.stderr)
        return 1

    target_rad = deg_to_rad(target_deg)
    print(f"臂: {args.arm}")
    print("起始 (deg):", format_deg(start))
    print("目标 (deg):", ", ".join(f"{value:.2f}" for value in target_deg))
    print("speed_ratio:", ", ".join(str(speed) for speed in speed_ratios))

    for speed_ratio in speed_ratios:
        print(f"\nspeed_ratio={speed_ratio}"
              + ("（100% 软限位）" if speed_ratio == 0 else ""))
        if move_and_time(api, group, target_rad, speed_ratio, "去程") is None:
            return 1
        if move_and_time(api, group, start, speed_ratio, "回程") is None:
            return 1
    print("\n速度对比结束，臂已回到起始关节角")
    return 0


if __name__ == "__main__":
    sys.exit(main())
