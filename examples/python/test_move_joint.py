"""低跟随测试：左臂循环执行 Home、Pregrasp，共 100 轮。关节角单位为弧度。"""

import argparse
from pathlib import Path
import sys
import time

import dual_arm_v2_2_sdk as sdk

from example_common import check, config_path, wait_for_joints


REPEAT_COUNT = 100
WAYPOINTS = (
    ("Home", [1.19573523, -0.67596774, 0.06985025, 1.52213409, 0.29127778, 1.29567500, 0.00000000]),
    ("Pregrasp", [0.41902900, -0.43728500, 0.72938900, 1.51622200, 0.66806600, 0.98204000, 1.15699400]),
)


def main():
    parser = argparse.ArgumentParser(
        description=f"左臂低跟随 MoveJ 测试，循环执行 Home、Pregrasp，共 {REPEAT_COUNT} 轮"
    )
    parser.add_argument("config", nargs="?", type=Path, help="含 config.yaml 的配置目录")
    args = parser.parse_args()
    if args.config is not None and not (args.config / "config.yaml").is_file():
        parser.error(f"找不到配置文件：{args.config / 'config.yaml'}")
    config = str(args.config.resolve()) if args.config is not None else config_path([""])

    group = sdk.Group.LEFT_ARM
    api = sdk.ControlApi(config)
    if wait_for_joints(api, group) is None:
        print("3 秒内没有收到左臂关节反馈，请检查 rt_control。", file=sys.stderr)
        return 1
    if not check(api.set_group(group, True), "使能左臂"):
        return 1

    for cycle in range(1, REPEAT_COUNT + 1):
        for index, (name, q_target) in enumerate(WAYPOINTS):
            if cycle > 1 or index > 0:
                time.sleep(0.5)
            started = time.monotonic()
            result = api.move_joint(group, q_target, follow=False, trajectory_mode=1,
                                    radio=100, speed_ratio=100)
            if not check(result, f"MoveJ {name}"):
                return 1
            print(f"[{cycle}/{REPEAT_COUNT}] MoveJ {name} 到位，耗时 {time.monotonic() - started:.3f} s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
