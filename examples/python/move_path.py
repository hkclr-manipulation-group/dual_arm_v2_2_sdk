"""用低跟随 MoveJ 依次运行十个左臂路点。关节角单位为弧度。"""

import argparse
from math import pi
from pathlib import Path
import sys
import time

import dual_arm_v2_2_sdk as sdk

from example_common import check, config_path, wait_for_joints


WAYPOINTS = (
    ("Local 拍摄位", [0.697810, -0.795287, 1.032950, 1.478360, 0.265192, 1.228070, 0.901480]),
    # 原表的 J6 = 1.570800 是 90° 的舍入值，使用精确角度以符合软限位。
    ("预抓取", [-0.662065, -0.806410, 0.904507, 0.810773, 1.370690, pi / 2, 0.279369]),
    ("抓取", [-0.801347, -0.899486, 1.053790, 0.798753, 1.429230, 1.432290, 0.280792]),
    ("放果位", [1.981130, -0.675968, 0.0698503, 1.522130, 0.291278, 1.295680, 0]),
    ("Local 拍摄位", [1.505040, -0.561505, 0.0110074, 1.750210, -0.0427598, 1.157560, 0.462956]),
    ("Local 拍摄位", [1.437790, -0.182933, 0.636826, 1.057700, -0.308258, 1.457330, 0.937227]),
    ("预抓取", [2.073010, -0.239888, -0.513027, 1.654910, 0.166348, 0.149981, 0.204663]),
    ("抓取", [2.054410, 0.104262, -0.650918, 1.359460, 1.011330, 0.243523, -0.445847]),
    ("放果位", [1.981130, -0.675968, 0.0698503, 1.522130, 0.291278, 1.295680, 0]),
    ("返回拍摄位", [1.437810, -0.182904, 0.636830, 1.057700, -0.308246, 1.457320, 0.937225]),
)


def main():
    parser = argparse.ArgumentParser(description="左臂低跟随 MoveJ 路点回放")
    parser.add_argument("config", nargs="?", type=Path, help="含 config.yaml 的配置目录")
    args = parser.parse_args()
    if args.config is not None and not (args.config / "config.yaml").is_file():
        parser.error(f"找不到配置文件：{args.config / 'config.yaml'}")
    config = str(args.config.resolve()) if args.config is not None else config_path([""])

    # group = sdk.Group.LEFT_ARM
    group = sdk.Group.RIGHT_ARM
    api = sdk.ControlApi(config)
    if wait_for_joints(api, group) is None:
        print("3 秒内没有收到左臂关节反馈，请检查 rt_control。", file=sys.stderr)
        return 1
    if not check(api.set_group(group, True), "使能左臂"):
        return 1

    for index, (name, target) in enumerate(WAYPOINTS, start=1):
        if index > 1:
            time.sleep(0.5)
        label = f"[{index}/{len(WAYPOINTS)}] {name}"
        result = api.move_joint(group, target, follow=False, trajectory_mode=1,
                                radio=100, speed_ratio=50)
        if not check(result, label):
            return 1
        print(f"{label} 到位")
    return 0


if __name__ == "__main__":
    sys.exit(main())
