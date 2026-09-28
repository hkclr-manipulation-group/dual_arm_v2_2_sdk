/**
 * @file test_arm_gripper_safe.c
 * @brief 安全流程演示：先开夹爪 -> 臂运动 -> 闭夹爪 -> 回 Home。
 *
 * 在文件顶部修改目标关节角（度）。需配置 robot.gripper。
 *
 * 用法（在 build/bin/ 目录下）：
 *   ./test_arm_gripper_safe
 */

#include "dual_arm_control_api.h"
#include "example_common.h"
#include "example_velocity_guard.h"

#include <stdio.h>
#include <time.h>

/* ============ Edit target joint angles here (degrees, left arm J1..J7) ============ */
static const float kPose1JointsDeg[7] = {15.0F, 10.0F, 12.0F, 8.0F,
                                         10.0F, 5.0F, 5.0F};
static const float kPose2JointsDeg[7] = {25.0F, 15.0F, 18.0F, 12.0F,
                                         15.0F, 8.0F, 8.0F};
static const float kHomeJointsDeg[7] = {0.0F, 0.0F, 0.0F, 0.0F,
                                      0.0F, 0.0F, 0.0F};

static const float kMaxSpeedDegS = 20.0F;
static const float kVelocityGuardDegS = 35.0F;

static const uint16_t kGripperOpen = 0;
static const uint16_t kGripperClosed = 1000;
static const uint16_t kGripperSpeed = 400;
static const uint16_t kGripperForce = 400;
static const int kGripperTimeoutSeconds = 10;
/* =================================================================================== */

static const float kMaxJointVelocity = kVelocityGuardDegS * kDegToRad;
static const float kTestMaxSpeed = kMaxSpeedDegS * kDegToRad;

static void joints_deg_to_rad(const float* joints_deg, size_t count,
                              float* joints) {
    for (size_t i = 0; i < count; ++i) {
        joints[i] = joints_deg[i] * kDegToRad;
    }
}

static void print_joints_deg(const char* label, const float* joints_deg,
                             size_t count) {
    printf("%s (deg): ", label);
    for (size_t i = 0; i < count; ++i) {
        printf("J%zu=%.0f", i + 1, joints_deg[i]);
        if (i + 1 < count) {
            printf(", ");
        }
    }
    printf("\n");
}

static void print_joints_rad(const float* joints, size_t count) {
    for (size_t i = 0; i < count; ++i) {
        printf("  J%zu: %.2f deg\n", i + 1, joints[i] * kRadToDeg);
    }
}

static int print_gripper_state(dual_arm_control_api_t* api) {
    dual_arm_gripper_state_t state;
    const dual_arm_ret_code_t code =
        dual_arm_get_gripper_state(api, DUAL_ARM_GROUP_LEFT_ARM, &state);
    if (!check_result("get_gripper_state", code)) {
        return 0;
    }
    printf("  position: %u / 1000\n", state.position);
    printf("  speed:    %u / 1000\n", state.speed);
    printf("  force:    %u / 1000\n", state.force);
    return 1;
}

static int wait_for_arm_joint_state(dual_arm_control_api_t* api,
                                    float* joints, size_t capacity,
                                    size_t* out_count, int timeout_ms) {
    struct timespec deadline;
    clock_gettime(CLOCK_MONOTONIC, &deadline);
    deadline.tv_sec += timeout_ms / 1000;
    deadline.tv_nsec += (long)(timeout_ms % 1000) * 1000000L;
    if (deadline.tv_nsec >= 1000000000L) {
        deadline.tv_sec += 1;
        deadline.tv_nsec -= 1000000000L;
    }

    while (1) {
        const dual_arm_ret_code_t code = dual_arm_get_joint(
            api, DUAL_ARM_GROUP_LEFT_ARM, joints, capacity, out_count);
        if (code == DUAL_ARM_RET_SUCCESS) {
            return 1;
        }
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        if (now.tv_sec > deadline.tv_sec ||
            (now.tv_sec == deadline.tv_sec &&
             now.tv_nsec >= deadline.tv_nsec)) {
            break;
        }
        sleep_ms(20);
    }
    return 0;
}

static int wait_for_gripper_state(dual_arm_control_api_t* api,
                                  int timeout_ms) {
    struct timespec deadline;
    clock_gettime(CLOCK_MONOTONIC, &deadline);
    deadline.tv_sec += timeout_ms / 1000;
    deadline.tv_nsec += (long)(timeout_ms % 1000) * 1000000L;
    if (deadline.tv_nsec >= 1000000000L) {
        deadline.tv_sec += 1;
        deadline.tv_nsec -= 1000000000L;
    }

    while (1) {
        dual_arm_gripper_state_t state;
        const dual_arm_ret_code_t code =
            dual_arm_get_gripper_state(api, DUAL_ARM_GROUP_LEFT_ARM, &state);
        if (code == DUAL_ARM_RET_SUCCESS) {
            printf("Gripper online: position=%u, speed=%u, force=%u\n",
                   state.position, state.speed, state.force);
            return 1;
        }
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        if (now.tv_sec > deadline.tv_sec ||
            (now.tv_sec == deadline.tv_sec &&
             now.tv_nsec >= deadline.tv_nsec)) {
            break;
        }
        sleep_ms(20);
    }
    return 0;
}

static int wait_until_joint_target(dual_arm_control_api_t* api,
                                   const example_velocity_guard_t* guard,
                                   const float* target, size_t target_count,
                                   int timeout_seconds) {
    const float kTolerance = 0.5F * kDegToRad;
    int elapsed_ms = 0;
    int stable_samples = 0;

    while (elapsed_ms < timeout_seconds * 1000) {
        if (example_velocity_guard_triggered(guard)) {
            return 0;
        }

        float position[kMaxJoints];
        size_t pos_count = 0;
        const dual_arm_ret_code_t code = dual_arm_get_joint(
            api, DUAL_ARM_GROUP_LEFT_ARM, position, kMaxJoints, &pos_count);
        if (code != DUAL_ARM_RET_SUCCESS || pos_count != target_count) {
            sleep_ms(10);
            elapsed_ms += 10;
            continue;
        }

        float largest_error = 0.0F;
        for (size_t i = 0; i < target_count; ++i) {
            largest_error = float_max(
                largest_error, float_abs(position[i] - target[i]));
        }

        stable_samples =
            largest_error <= kTolerance ? stable_samples + 1 : 0;
        if (stable_samples >= 10) {
            printf("Arm target reached.\n");
            return 1;
        }
        sleep_ms(10);
        elapsed_ms += 10;
    }

    fprintf(stderr, "Timed out waiting for the arm target.\n");
    return 0;
}

static int move_arm_low_follow(dual_arm_control_api_t* api,
                               const example_velocity_guard_t* guard,
                               const float* joints, size_t joint_count,
                               const char* label, int timeout_seconds) {
    if (example_velocity_guard_triggered(guard)) {
        return 0;
    }
    if (!check_result(label, dual_arm_move_joint(api, DUAL_ARM_GROUP_LEFT_ARM,
                                                 joints, joint_count, 0, 0,
                                                 0))) {
        return 0;
    }
    return wait_until_joint_target(api, guard, joints, joint_count,
                                   timeout_seconds);
}

static int move_gripper(dual_arm_control_api_t* api, uint16_t position,
                        const char* label) {
    if (!check_result(label, dual_arm_set_gripper_position(
                                  api, DUAL_ARM_GROUP_LEFT_ARM, position, 1,
                                  kGripperTimeoutSeconds))) {
        return 0;
    }
    return print_gripper_state(api);
}

int main(void) {
    char config_path[kMaxConfigPathLen];
    default_config_path(config_path, sizeof(config_path));

    dual_arm_control_api_t* api = dual_arm_control_api_create(config_path);
    if (api == NULL) {
        fprintf(stderr, "Fatal SDK error: %s\n",
                dual_arm_control_api_last_error(api));
        return 1;
    }

    float pose1[kMaxJoints];
    float pose2[kMaxJoints];
    float home[kMaxJoints];
    joints_deg_to_rad(kPose1JointsDeg, 7, pose1);
    joints_deg_to_rad(kPose2JointsDeg, 7, pose2);
    joints_deg_to_rad(kHomeJointsDeg, 7, home);

    printf("Safe arm + gripper demo\n"
           "  - edit kPose1JointsDeg / kPose2JointsDeg / "
           "kHomeJointsDeg at the top of this file\n"
           "  - each array is [J1, J2, J3, J4, J5, J6, J7] in degrees\n");
    print_joints_deg("Pose 1", kPose1JointsDeg, 7);
    print_joints_deg("Pose 2", kPose2JointsDeg, 7);
    print_joints_deg("Home", kHomeJointsDeg, 7);

    float current[kMaxJoints];
    size_t current_count = 0;
    printf("\nWaiting for rt_control arm state...\n");
    if (!wait_for_arm_joint_state(api, current, kMaxJoints, &current_count,
                                  3000)) {
        fprintf(stderr, "No arm state received within 3 seconds.\n");
        dual_arm_control_api_destroy(api);
        return 1;
    }
    if (current_count != 7) {
        fprintf(stderr, "Expected 7 left-arm joints, got %zu.\n",
                current_count);
        dual_arm_control_api_destroy(api);
        return 1;
    }
    printf("Current left-arm joints:\n");
    print_joints_rad(current, current_count);

    printf("\nWaiting for rt_control gripper state...\n");
    if (!wait_for_gripper_state(api, 3000)) {
        fprintf(stderr, "No gripper state received within 3 seconds.\n");
        dual_arm_control_api_destroy(api);
        return 1;
    }

    example_velocity_guard_t guard;
    if (!example_velocity_guard_start(&guard, api, DUAL_ARM_GROUP_LEFT_ARM,
                                      kMaxJointVelocity)) {
        fprintf(stderr, "Failed to start velocity guard thread.\n");
        dual_arm_control_api_destroy(api);
        return 1;
    }

    if (!wait_for_enter(
            "Stage 1/6: configure gripper with moderate speed/force.")) {
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 0;
    }
    if (!check_result("set_gripper_config",
                      dual_arm_set_gripper_config(
                          api, DUAL_ARM_GROUP_LEFT_ARM, kGripperSpeed,
                          kGripperForce, 1, 2))) {
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 1;
    }

    if (!wait_for_enter("Stage 2/6: open gripper before any arm motion.")) {
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 0;
    }
    if (!move_gripper(api, kGripperOpen, "set_gripper_position(open)")) {
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 1;
    }

    if (!wait_for_enter("Stage 3/6: enable left arm and move to Pose 1.")) {
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 0;
    }
    if (!check_result("set_group(enable)",
                      dual_arm_set_group(api, DUAL_ARM_GROUP_LEFT_ARM, 1))) {
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 1;
    }

    float requested_speed[kMaxJoints];
    float applied_speed[kMaxJoints];
    size_t applied_count = 0;
    for (size_t i = 0; i < current_count; ++i) {
        requested_speed[i] = kTestMaxSpeed;
    }
    if (dual_arm_set_joint_max_speed(api, DUAL_ARM_GROUP_LEFT_ARM,
                                     requested_speed, current_count,
                                     applied_speed, kMaxJoints,
                                     &applied_count) != DUAL_ARM_RET_SUCCESS ||
        applied_count != current_count) {
        fprintf(stderr, "set_joint_max_speed failed.\n");
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 1;
    }
    printf("set_joint_max_speed: %.0f deg/s for every left-arm joint\n",
           kMaxSpeedDegS);

    if (!move_arm_low_follow(api, &guard, pose1, 7, "move_joint(pose1)", 20)) {
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 1;
    }

    if (!wait_for_enter(
            "Stage 4/6: close gripper while the arm is stationary.")) {
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 0;
    }
    if (!move_gripper(api, kGripperClosed, "set_gripper_position(close)")) {
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 1;
    }

    if (!wait_for_enter("Stage 5/6: move to Pose 2.")) {
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 0;
    }
    if (!move_arm_low_follow(api, &guard, pose2, 7, "move_joint(pose2)", 20)) {
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 1;
    }

    if (!wait_for_enter("Stage 6/6: open gripper and return to Home.")) {
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 0;
    }
    if (!move_gripper(api, kGripperOpen, "set_gripper_position(release)")) {
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 1;
    }
    if (!move_arm_low_follow(api, &guard, home, 7, "move_joint(home)", 25)) {
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 1;
    }

    float final_joints[kMaxJoints];
    size_t final_count = 0;
    const dual_arm_ret_code_t joint_code = dual_arm_get_joint(
        api, DUAL_ARM_GROUP_LEFT_ARM, final_joints, kMaxJoints, &final_count);
    if (!check_result("get_joint(final)", joint_code)) {
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 1;
    }
    printf("Final left-arm joints:\n");
    print_joints_rad(final_joints, final_count);
    if (!print_gripper_state(api)) {
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 1;
    }

    example_velocity_guard_stop(&guard);
    printf("Safe arm + gripper demo completed. The arm remains enabled.\n");
    dual_arm_control_api_destroy(api);
    return 0;
}
