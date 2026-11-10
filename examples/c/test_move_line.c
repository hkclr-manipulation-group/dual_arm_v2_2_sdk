/**
 * @file test_move_line.c
 * @brief MoveL demo: 每臂独立执行「初始位姿 -> MoveL(+Z) -> 笛卡尔回程 -> 回零」。
 *
 * 流程（启动时左右臂同时使能，头部 disable）：
 *   左臂：初始关节位姿 -> MoveL +Z -> MoveL 回程 -> 收敛等待 -> 关节回零
 *   右臂：初始关节位姿 -> MoveL -Z 0.20m -> MoveL 回程 -> 收敛等待 -> 关节回零
 *
 * MoveL 时 SDK 对静止臂发送当前 ToolPose（零增量 TaskCmd）。
 *
 * 用法：./test_move_line [config_prefix_path]
 */

#include "dual_arm_control_api.h"
#include "example_common.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

enum {
    kToolPoseSize = 7,
    kArmJointCount = 7,
    kSpeedRatio = 15,
    kMoveTimeoutMs = 40000,
    kInitialReachTimeoutMs = 40000,
    kHomeTimeoutMs = 40000,
    kSettleTimeoutMs = 15000,
    kPollUs = 50000,
    kSettleStableCycles = 5,
    kPoseStableCycles = 3,
};

static const float kMoveLineAccTimeS = 8.0f;
static const float kJointVelToleranceRadS = 1.0f * kDegToRad;

static const float kInitialJointsDeg[kArmJointCount] = {
    10.0f, 10.0f, 10.0f, 45.0f, 5.0f, 5.0f, 5.0f,
};

static const float kLeftZOffsetM = 0.30f;
static const float kRightZOffsetM = -0.20f;
static const float kPosToleranceM = 0.005f;
static const float kJointToleranceRad = 0.5f * kDegToRad;
static const float kMaxSpeedDegS = 12.0f;
static const float kHomeMaxSpeedDegS = 8.0f;

static void joints_deg_to_rad(const float* joints_deg, size_t count,
                              float* joints) {
    for (size_t i = 0; i < count; ++i) {
        joints[i] = joints_deg[i] * kDegToRad;
    }
}

static void print_joints_deg(const char* label, const float* joints_rad,
                             size_t count) {
    printf("%s (deg): ", label);
    for (size_t i = 0; i < count; ++i) {
        printf("J%zu=%.1f", i + 1, joints_rad[i] * kRadToDeg);
        if (i + 1 < count) printf(", ");
    }
    printf("\n");
}

static void print_tool_pose(const char* label, const float* pose) {
    printf("%s\n", label);
    printf("  position (m): %.6f, %.6f, %.6f\n",
           pose[0], pose[1], pose[2]);
    printf("  quaternion:   %.6f, %.6f, %.6f, %.6f\n",
           pose[3], pose[4], pose[5], pose[6]);
}

static void offset_z(const float* pose, float dz, float* out) {
    memcpy(out, pose, sizeof(float) * kToolPoseSize);
    out[2] += dz;
}

static int verify_joint_pose_deg(const char* label, const float* joints_rad,
                                 size_t count, const float* expected_deg) {
    int ok = 1;
    printf("%s\n", label);
    for (size_t i = 0; i < count; ++i) {
        const float actual_deg = joints_rad[i] * kRadToDeg;
        const float err_deg = fabsf(actual_deg - expected_deg[i]);
        if (err_deg > 0.5f) {
            printf("  ERROR J%zu: target %.1f deg, actual %.1f deg (err %.1f)\n",
                   i + 1, expected_deg[i], actual_deg, err_deg);
            ok = 0;
        }
    }
    return ok;
}

static int wait_for_motion_settle(dual_arm_control_api_t* api,
                                  dual_arm_group_t group,
                                  size_t joint_count, int timeout_ms) {
    const int poll_ms = kPollUs / 1000;
    int elapsed_ms = 0;
    int stable = 0;
    int have_prev = 0;
    float prev[kMaxJoints];

    while (elapsed_ms < timeout_ms) {
        float current[kMaxJoints];
        size_t count = 0;
        const dual_arm_ret_code_t code = dual_arm_get_joint(
            api, group, current, kMaxJoints, &count);
        if (code != DUAL_ARM_RET_SUCCESS || count != joint_count) {
            usleep(kPollUs);
            elapsed_ms += poll_ms;
            continue;
        }

        if (have_prev) {
            float max_vel = 0.0f;
            for (size_t i = 0; i < joint_count; ++i) {
                const float vel = fabsf(current[i] - prev[i]) /
                                  (poll_ms * 0.001f);
                if (vel > max_vel) max_vel = vel;
            }
            if (max_vel <= kJointVelToleranceRadS) {
                if (++stable >= kSettleStableCycles) return 1;
            } else {
                stable = 0;
            }
        }

        memcpy(prev, current, joint_count * sizeof(float));
        have_prev = 1;
        usleep(kPollUs);
        elapsed_ms += poll_ms;
    }
    return 0;
}

static int wait_for_joint_target(dual_arm_control_api_t* api,
                                 dual_arm_group_t group,
                                 const float* target, size_t joint_count,
                                 int timeout_ms) {
    const int poll_ms = kPollUs / 1000;
    int elapsed_ms = 0;
    int stable = 0;
    float current[kMaxJoints];
    size_t count = 0;

    while (elapsed_ms < timeout_ms) {
        const dual_arm_ret_code_t code = dual_arm_get_joint(
            api, group, current, kMaxJoints, &count);
        if (code != DUAL_ARM_RET_SUCCESS || count != joint_count) {
            usleep(kPollUs);
            elapsed_ms += poll_ms;
            continue;
        }

        const float err = max_joint_error(target, current, joint_count);
        if (err <= kJointToleranceRad) {
            if (++stable >= 3) return 1;
        } else {
            stable = 0;
        }
        usleep(kPollUs);
        elapsed_ms += poll_ms;
    }
    return 0;
}

static int configure_both_arms(dual_arm_control_api_t* api) {
    if (dual_arm_set_group(api, DUAL_ARM_GROUP_LEFT_ARM, 1) !=
            DUAL_ARM_RET_SUCCESS ||
        dual_arm_set_group(api, DUAL_ARM_GROUP_RIGHT_ARM, 1) !=
            DUAL_ARM_RET_SUCCESS ||
        dual_arm_set_group(api, DUAL_ARM_GROUP_HEAD, 0) !=
            DUAL_ARM_RET_SUCCESS) {
        printf("Failed to enable left + right arms (head disabled).\n");
        return 0;
    }
    printf("Left + right arms enabled, head disabled.\n");
    return 1;
}

static int setup_arm_speed_deg(dual_arm_control_api_t* api,
                               dual_arm_group_t group, float max_speed_deg_s) {
    float speed[kArmJointCount];
    float out_speed[kMaxJoints];
    for (size_t i = 0; i < kArmJointCount; ++i) {
        speed[i] = max_speed_deg_s * kDegToRad;
    }
    size_t out_count = 0;
    const dual_arm_ret_code_t code = dual_arm_set_joint_max_speed(
        api, group, speed, kArmJointCount, out_speed, kMaxJoints, &out_count);
    if (code != DUAL_ARM_RET_SUCCESS || out_count != kArmJointCount) {
        printf("set_joint_max_speed(%s) failed\n", dual_arm_group_name(group));
        return 0;
    }
    return 1;
}

static int setup_arm_speed(dual_arm_control_api_t* api, dual_arm_group_t group) {
    return setup_arm_speed_deg(api, group, kMaxSpeedDegS);
}

static int move_to_initial_pose(dual_arm_control_api_t* api,
                                dual_arm_group_t group) {
    float target[kArmJointCount];
    joints_deg_to_rad(kInitialJointsDeg, kArmJointCount, target);

    printf("\n==> move_joint initial pose (%s)\n", dual_arm_group_name(group));
    print_joints_deg("  target", target, kArmJointCount);

    const dual_arm_ret_code_t code = dual_arm_move_joint(
        api, group, target, kArmJointCount, 0, 0, 0, 0);
    printf("  move_joint: %s\n", dual_arm_ret_code_name(code));
    if (code != DUAL_ARM_RET_SUCCESS) return 0;

    if (!wait_for_joint_target(api, group, target, kArmJointCount,
                               kInitialReachTimeoutMs)) {
        printf("  WARNING: initial pose not reached within %d ms\n",
               kInitialReachTimeoutMs);
        return 0;
    }

    float actual[kMaxJoints];
    size_t count = 0;
    if (dual_arm_get_joint(api, group, actual, kMaxJoints, &count) !=
            DUAL_ARM_RET_SUCCESS ||
        count != kArmJointCount) {
        printf("  ERROR: failed to read joints after initial move\n");
        return 0;
    }

    print_joints_deg("  actual", actual, count);
    if (!verify_joint_pose_deg("  initial pose check:", actual, count,
                               kInitialJointsDeg)) {
        printf("  ERROR: initial pose verification failed for %s\n",
               dual_arm_group_name(group));
        return 0;
    }
    printf("==> initial pose done (%s)\n", dual_arm_group_name(group));
    return 1;
}

static int hold_current_joint_pose(dual_arm_control_api_t* api,
                                   dual_arm_group_t group) {
    float current[kMaxJoints];
    size_t count = 0;

    if (dual_arm_get_joint(api, group, current, kMaxJoints, &count) !=
            DUAL_ARM_RET_SUCCESS ||
        count != kArmJointCount) {
        printf("  ERROR: failed to read current joints (%s)\n",
               dual_arm_group_name(group));
        return 0;
    }

    printf("\n==> hold current joint pose (%s, TaskLine -> Joint handoff)\n",
           dual_arm_group_name(group));
    print_joints_deg("  hold", current, count);

    const dual_arm_ret_code_t code = dual_arm_move_joint(
        api, group, current, kArmJointCount, 0, 0, 0, 0);
    printf("  move_joint: %s\n", dual_arm_ret_code_name(code));
    if (code != DUAL_ARM_RET_SUCCESS) return 0;

    if (!wait_for_joint_target(api, group, current, kArmJointCount,
                               kInitialReachTimeoutMs)) {
        printf("  WARNING: hold pose not reached within %d ms\n",
               kInitialReachTimeoutMs);
        return 0;
    }
    return 1;
}

static int prepare_for_go_home(dual_arm_control_api_t* api,
                               dual_arm_group_t group) {
    printf("\n==> prepare for go_home (%s)\n", dual_arm_group_name(group));

    if (!wait_for_motion_settle(api, group, kArmJointCount, kSettleTimeoutMs)) {
        printf("  WARNING: motion did not settle within %d ms\n",
               kSettleTimeoutMs);
        return 0;
    }
    printf("  motion settled (joint vel <= %.1f deg/s)\n",
           kJointVelToleranceRadS * kRadToDeg);

    if (!hold_current_joint_pose(api, group)) return 0;

    if (!wait_for_motion_settle(api, group, kArmJointCount, kSettleTimeoutMs)) {
        printf("  WARNING: hold pose did not settle within %d ms\n",
               kSettleTimeoutMs);
        return 0;
    }
    printf("  ready for go_home\n");
    return 1;
}

static int go_home(dual_arm_control_api_t* api, dual_arm_group_t group) {
    float zero[kArmJointCount];
    for (size_t i = 0; i < kArmJointCount; ++i) {
        zero[i] = 0.0f;
    }

    if (!setup_arm_speed_deg(api, group, kHomeMaxSpeedDegS)) return 0;

    printf("\n==> go home (%s, all joints -> 0)\n", dual_arm_group_name(group));
    const dual_arm_ret_code_t code = dual_arm_move_joint(
        api, group, zero, kArmJointCount, 0, 0, 0, 0);
    printf("  move_joint: %s\n", dual_arm_ret_code_name(code));
    if (code != DUAL_ARM_RET_SUCCESS) return 0;

    if (!wait_for_joint_target(api, group, zero, kArmJointCount,
                               kHomeTimeoutMs)) {
        printf("  WARNING: home pose not reached within %d ms\n",
               kHomeTimeoutMs);
        return 0;
    }

    float actual[kMaxJoints];
    size_t count = 0;
    if (dual_arm_get_joint(api, group, actual, kMaxJoints, &count) ==
            DUAL_ARM_RET_SUCCESS &&
        count == kArmJointCount) {
        print_joints_deg("  actual", actual, count);
    }
    printf("==> home done (%s)\n", dual_arm_group_name(group));
    return setup_arm_speed(api, group);
}

static int wait_tool_pose_z(dual_arm_control_api_t* api, dual_arm_group_t group,
                            float target_z, float tolerance_m, int timeout_ms) {
    const int poll_ms = kPollUs / 1000;
    int elapsed_ms = 0;
    int stable = 0;
    float pose[kToolPoseSize];
    size_t count = 0;

    while (elapsed_ms < timeout_ms) {
        const dual_arm_ret_code_t code = dual_arm_get_tool_pose(
            api, group, pose, kToolPoseSize, &count);
        if (code == DUAL_ARM_RET_SUCCESS && count >= 3 &&
            fabsf(pose[2] - target_z) <= tolerance_m) {
            if (++stable >= kPoseStableCycles) return 1;
        } else {
            stable = 0;
        }
        usleep(kPollUs);
        elapsed_ms += poll_ms;
    }
    return 0;
}

static int run_move_line(dual_arm_control_api_t* api, dual_arm_group_t group,
                         const float* target) {
    const dual_arm_ret_code_t code = dual_arm_move_line(
        api, group, target, kToolPoseSize, kSpeedRatio, kMoveLineAccTimeS);
    printf("move_line(%s): %s\n", dual_arm_group_name(group),
           dual_arm_ret_code_name(code));
    if (code != DUAL_ARM_RET_SUCCESS) return 0;
    if (!wait_tool_pose_z(api, group, target[2], kPosToleranceM,
                          kMoveTimeoutMs)) {
        printf("Warning: ACK ok but ToolPose Z did not reach target within "
               "%d ms\n", kMoveTimeoutMs);
        return 0;
    }
    return 1;
}

static int demo_movel_and_return(dual_arm_control_api_t* api,
                                 dual_arm_group_t group, float z_offset_m) {
    float current[kToolPoseSize];
    size_t count = 0;
    dual_arm_ret_code_t code = dual_arm_get_tool_pose(
        api, group, current, kToolPoseSize, &count);
    if (code != DUAL_ARM_RET_SUCCESS || count < kToolPoseSize) {
        printf("get_tool_pose failed for %s\n", dual_arm_group_name(group));
        return 0;
    }

    print_tool_pose("Current tool pose:", current);

    float forward[kToolPoseSize];
    offset_z(current, z_offset_m, forward);
    printf("Target (Z %+.2f m):\n", z_offset_m);
    print_tool_pose(" ", forward);
    if (!run_move_line(api, group, forward)) return 0;

    float after_forward[kToolPoseSize];
    code = dual_arm_get_tool_pose(api, group, after_forward, kToolPoseSize,
                                  &count);
    if (code != DUAL_ARM_RET_SUCCESS || count < kToolPoseSize) {
        printf("get_tool_pose failed after MoveL\n");
        return 0;
    }
    print_tool_pose("After MoveL:", after_forward);

    float back[kToolPoseSize];
    offset_z(after_forward, -z_offset_m, back);
    print_tool_pose("Target (cartesian return):", back);
    if (!run_move_line(api, group, back)) return 0;

    float final_pose[kToolPoseSize];
    code = dual_arm_get_tool_pose(api, group, final_pose, kToolPoseSize,
                                  &count);
    if (code == DUAL_ARM_RET_SUCCESS) {
        print_tool_pose("After cartesian return:", final_pose);
    }
    return 1;
}

static int run_arm_cycle(dual_arm_control_api_t* api, dual_arm_group_t group,
                         const char* label, float z_offset_m) {
    printf("\n========================================\n");
    printf("=== %s ===\n", label);
    printf("========================================\n");

    if (!move_to_initial_pose(api, group)) return 0;

    printf("\n--- MoveL Z %+.2f m, then cartesian return ---\n", z_offset_m);
    if (!demo_movel_and_return(api, group, z_offset_m)) return 0;

    if (!prepare_for_go_home(api, group)) return 0;

    if (!go_home(api, group)) return 0;
    return 1;
}

int main(int argc, char* argv[]) {
    char config_path[kMaxConfigPathLen];
    config_path_from_args(argc, argv, config_path, sizeof(config_path));

    dual_arm_control_api_t* api = dual_arm_control_api_create(config_path);
    if (api == NULL) {
        fprintf(stderr, "create failed: %s\n",
                dual_arm_control_api_last_error(api));
        return 1;
    }

    if (!configure_both_arms(api) ||
        !setup_arm_speed(api, DUAL_ARM_GROUP_LEFT_ARM) ||
        !setup_arm_speed(api, DUAL_ARM_GROUP_RIGHT_ARM)) {
        dual_arm_control_api_destroy(api);
        return 1;
    }

    if (!run_arm_cycle(api, DUAL_ARM_GROUP_LEFT_ARM, "Left arm cycle",
                       kLeftZOffsetM)) {
        dual_arm_control_api_destroy(api);
        return 1;
    }

    if (!run_arm_cycle(api, DUAL_ARM_GROUP_RIGHT_ARM, "Right arm cycle",
                       kRightZOffsetM)) {
        dual_arm_control_api_destroy(api);
        return 1;
    }

    printf("\nDone.\n");
    dual_arm_control_api_destroy(api);
    return 0;
}
