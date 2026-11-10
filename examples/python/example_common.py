"""示例共用的配置路径和状态等待。不是 SDK 的一部分。"""

import os
import sys
import time
from pathlib import Path

import dual_arm_v2_2_sdk as sdk


def config_path(argv=None):
    """命令行首参、DUAL_ARM_CONFIG_PREFIX，否则相对仓库查找 dual_v2_2。"""
    argv = sys.argv if argv is None else argv
    if len(argv) > 1:
        candidate = Path(argv[1])
        if (candidate / "config.yaml").is_file():
            return str(candidate.resolve())

    env = os.environ.get("DUAL_ARM_CONFIG_PREFIX") or os.environ.get(
        "DUAL_ARM_V2_2_CONFIG")
    if env and (Path(env) / "config.yaml").is_file():
        return str(Path(env).resolve())

    sdk_root = Path(__file__).resolve().parents[2]
    candidates = [
        sdk_root / "cuarm_configuration" / "dual_v2_2",
        sdk_root.parent / "cuarm_configuration" / "dual_v2_2",
    ]
    for directory in candidates:
        if (directory / "config.yaml").is_file():
            return str(directory.resolve())
    return str(candidates[0])


def wait_for_joints(api, group, timeout_s=3.0):
    """等到 get_joint 返回非空反馈。超时返回 None。"""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        code, joints = api.get_joint(group)
        if code == sdk.RetCode.SUCCESS and joints:
            return joints
        time.sleep(0.02)
    return None


def check(code, what):
    if code != sdk.RetCode.SUCCESS:
        print(f"{what}: {code}", file=sys.stderr)
        return False
    return True
