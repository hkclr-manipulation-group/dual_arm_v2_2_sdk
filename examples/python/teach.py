"""重力补偿示教：拖动双臂，输入 s 记录当前关节角（度），直接回车结束。

用法：
    python3 examples/python/teach.py
    python3 examples/python/teach.py /path/to/cuarm_configuration/dual_v2_2
"""

import math
import sys

import dual_arm_v2_2_sdk as sdk

from example_common import check, config_path, wait_for_joints


def format_deg(joints):
    return ", ".join(f"{value * 180.0 / math.pi:.2f}" for value in joints)


def main():
    api = sdk.ControlApi(config_path())
    if wait_for_joints(api, sdk.Group.LEFT_ARM) is None:
        print("3 秒内没有收到状态。rt_control 是否已启动？", file=sys.stderr)
        return 1
    if not check(api.set_group(sdk.Group.LEFT_ARM, True), "使能左臂") or not check(
            api.set_group(sdk.Group.RIGHT_ARM, True), "使能右臂"):
        return 1

    if not check(api.start_teach_mode(True), "start_teach_mode"):
        return 1
    print("已进入重力补偿示教，可以拖动双臂。")
    print("输入 s 回车记录当前关节角，直接回车结束。")

    count = 0
    try:
        while True:
            line = input("> ")
            if line.strip() != "s":
                break
            left_code, left = api.get_joint(sdk.Group.LEFT_ARM)
            right_code, right = api.get_joint(sdk.Group.RIGHT_ARM)
            if left_code != sdk.RetCode.SUCCESS or right_code != sdk.RetCode.SUCCESS:
                print("读取关节失败，这一点未记录")
                continue
            count += 1
            print(f"#{count} 左臂 (deg): {format_deg(left)}")
            print(f"#{count} 右臂 (deg): {format_deg(right)}")
    finally:
        print("请松开双臂。")
        if not check(api.stop_teach_mode(), "stop_teach_mode"):
            return 1
    print(f"示教结束，记录了 {count} 个点")
    return 0


if __name__ == "__main__":
    sys.exit(main())
