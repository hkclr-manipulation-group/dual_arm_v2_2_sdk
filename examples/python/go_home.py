"""双臂回零：左/右臂各关节置 0，头部保持当前角度。

用法：
    python3 examples/python/go_home.py
    python3 examples/python/go_home.py /path/to/cuarm_configuration/dual_v2_2
"""

import sys

import dual_arm_v2_2_sdk as sdk

from example_common import check, config_path, wait_for_joints


def main():
    api = sdk.ControlApi(config_path())
    if wait_for_joints(api, sdk.Group.LEFT_ARM) is None:
        print("3 秒内没有收到状态。rt_control 是否已启动？", file=sys.stderr)
        return 1
    if not check(api.set_group(sdk.Group.LEFT_ARM, True), "使能左臂") or not check(
            api.set_group(sdk.Group.RIGHT_ARM, True), "使能右臂"):
        return 1

    head = wait_for_joints(api, sdk.Group.HEAD)
    left = wait_for_joints(api, sdk.Group.LEFT_ARM)
    if head is None or left is None:
        print("读取关节状态失败", file=sys.stderr)
        return 1

    zero = [0.0] * len(left)
    if not check(api.move_joint(sdk.Group.ALL, zero + head + zero, False),
                 "move_joint"):
        return 1
    print("双臂正在回零（手臂关节为 0）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
