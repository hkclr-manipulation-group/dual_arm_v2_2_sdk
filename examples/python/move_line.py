"""左臂末端位姿的 z 增加 5 cm，再回到原位。

move_line 会阻塞到末端到位（0.5 mm / 0.01°），不用再轮询 get_tool_pose。

用法：
    python3 examples/python/move_line.py
    python3 examples/python/move_line.py /path/to/cuarm_configuration/dual_v2_2
"""

import sys

import dual_arm_v2_2_sdk as sdk

from example_common import check, config_path, wait_for_joints

Z_OFFSET_M = 0.05
SPEED_RATIO = 30
ACC_TIME_S = 3.0


def main():
    api = sdk.ControlApi(config_path())
    if wait_for_joints(api, sdk.Group.LEFT_ARM) is None:
        print("3 秒内没有收到状态。rt_control 是否已启动？", file=sys.stderr)
        return 1
    if not check(api.set_group(sdk.Group.LEFT_ARM, True), "使能左臂"):
        return 1

    code, start = api.get_tool_pose(sdk.Group.LEFT_ARM)
    if not check(code, "get_tool_pose") or len(start) != 7:
        print(f"末端位姿长度应为 7，实际 {len(start)}", file=sys.stderr)
        return 1
    print("起始位姿 (m, quat):", ", ".join(f"{value:.4f}" for value in start))

    forward = list(start)
    forward[2] += Z_OFFSET_M
    if not check(
            api.move_line(sdk.Group.LEFT_ARM, forward, SPEED_RATIO, ACC_TIME_S),
            "move_line 前进"):
        return 1
    if not check(
            api.move_line(sdk.Group.LEFT_ARM, start, SPEED_RATIO, ACC_TIME_S),
            "move_line 返回"):
        return 1
    print("左臂已沿 z 往返 %.0f mm" % (Z_OFFSET_M * 1000.0))
    return 0


if __name__ == "__main__":
    sys.exit(main())
