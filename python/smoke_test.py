"""Offline smoke test for the dual_arm_v2_2_sdk Python binding.

No robot / planner is required: it only checks the exposed API surface and the
constructor error path. Set DUAL_ARM_V2_2_CONFIG=/path/to/dual_v2_2 to also open
a real UDP link and read the left-arm feedback.

    PYTHONPATH=build/python python3 python/smoke_test.py
"""

import os
import sys

import dual_arm_v2_2_sdk as sdk

EXPECTED_METHODS = {
    # lifecycle / enable
    "set_group",
    "get_group_state",
    # joint streaming
    "set_joint_target",
    "set_left_joint_target",
    "set_right_joint_target",
    "flush_joint_command",
    "reset_joint_target_cache",
    "start_dual_arm_streaming",
    "stop_dual_arm_streaming",
    "is_dual_arm_streaming",
    # tool-line streaming
    "set_left_tool_line_target",
    "set_right_tool_line_target",
    "flush_tool_line_command",
    "reset_tool_line_target_cache",
    "start_dual_arm_tool_line_auto_flush",
    "stop_dual_arm_tool_line_auto_flush",
    "is_dual_arm_tool_line_auto_flush",
    # feedback
    "get_joint",
    "get_joint_velocity",
    "get_tool_pose",
    # direct joint / Cartesian move + joint limits (re-exported in 0.1.3+)
    "move_joint",
    "move_line",
    "set_joint_max_speed",
    "get_joint_max_limit",
    "get_joint_min_limit",
    # gripper
    "set_gripper_config",
    "set_gripper_position",
    "get_gripper_state",
    # teach mode (gravity compensation)
    "start_teach_mode",
    "stop_teach_mode",
}

# Still available in C++/C, intentionally hidden in Python.
HIDDEN_METHODS = {
    "set_joint_max_acc",
}


def check(name, condition, detail=""):
    status = "ok" if condition else "FAIL"
    print(f"[{status}] {name}{(' - ' + detail) if detail else ''}")
    return condition


def main():
    ok = True

    ok &= check("module import", hasattr(sdk, "ControlApi"))
    ok &= check("__version__", sdk.__version__ == "1.0.0", sdk.__version__)
    ok &= check("enums", all(hasattr(sdk, e) for e in ("Group", "RetCode", "State")))
    # pybind11 enums are not iterable; inspect __members__ instead.
    ok &= check(
        "Group members",
        set(sdk.Group.__members__) == {"ALL", "LEFT_ARM", "RIGHT_ARM", "HEAD", "ELEVATOR"},
    )
    ok &= check(
        "RetCode members",
        set(sdk.RetCode.__members__)
        == {"SUCCESS", "CONTROLLER_ERROR", "SEND_FAILED", "RECEIVE_FAILED",
            "PARSE_FAILED", "TIMEOUT"},
    )
    ok &= check(
        "State members",
        set(sdk.State.__members__) == {"SHUTDOWN", "OPERATIONAL", "ERROR"},
    )

    methods = {name for name in dir(sdk.ControlApi) if not name.startswith("_")}
    ok &= check("exposed method count == 30", len(methods) == 30, str(len(methods)))
    ok &= check("exposed method set", methods == EXPECTED_METHODS,
                "missing=%s extra=%s" % (sorted(EXPECTED_METHODS - methods),
                                         sorted(methods - EXPECTED_METHODS)))
    ok &= check("hidden methods absent", not (methods & HIDDEN_METHODS),
                str(sorted(methods & HIDDEN_METHODS)))
    move_joint = sdk.ControlApi.move_joint
    ok &= check("move_joint exposes speed_ratio",
                "speed_ratio" in move_joint.__doc__, move_joint.__doc__.split("\n")[0])

    try:
        sdk.ControlApi("/nonexistent/dual_v2_2")
    except RuntimeError as exc:
        ok &= check("constructor rejects bad config path", True, str(exc)[:80])
    else:
        ok &= check("constructor rejects bad config path", False, "no exception raised")

    config = os.environ.get("DUAL_ARM_V2_2_CONFIG")
    if config:
        api = sdk.ControlApi(config)
        code, joints = api.get_joint(sdk.Group.LEFT_ARM)
        ok &= check("live get_joint returns (RetCode, list)",
                    isinstance(code, sdk.RetCode) and isinstance(joints, list),
                    f"{code}, len={len(joints)}")
        bad = api.set_joint_target(sdk.Group.LEFT_ARM, [0.0] * 16)
        ok &= check("set_joint_target rejects wrong length",
                    bad == sdk.RetCode.CONTROLLER_ERROR, str(bad))
        ok &= check("is_dual_arm_streaming default", api.is_dual_arm_streaming() is False)
        ok &= check("is_dual_arm_tool_line_auto_flush default",
                    api.is_dual_arm_tool_line_auto_flush() is False)
    else:
        print("[skip] live checks (set DUAL_ARM_V2_2_CONFIG to enable)")

    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
