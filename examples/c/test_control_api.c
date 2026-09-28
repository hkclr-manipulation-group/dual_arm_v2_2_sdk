/**
 * @file test_control_api.c
 * @brief SDK 交互式冒烟测试：组控制、关节读写、低/高跟随、速度限位。
 *
 * 用法（在 build/bin/ 目录下）：
 *   ./test_control_api          # 默认不主动运动
 *   ./test_control_api 0.01     # J1 偏移 +0.01 rad 的小幅运动测试
 */

#include "dual_arm_control_api.h"
#include "example_common.h"
#include "example_velocity_guard.h"

#include <stdio.h>

static const float kMaxJointVelocity = 30.0F * kDegToRad;

static const char* state_name(dual_arm_state_t state) {
    switch (state) {
        case DUAL_ARM_STATE_SHUTDOWN:
            return "SHUTDOWN";
        case DUAL_ARM_STATE_OPERATIONAL:
            return "OPERATIONAL";
        case DUAL_ARM_STATE_ERROR:
            return "ERROR";
        default:
            return "UNKNOWN";
    }
}

static void print_joints(const float* joints, size_t count) {
    for (size_t i = 0; i < count; ++i) {
        printf("  J%zu: %.4f rad (%.4f deg)\n", i + 1, joints[i],
               joints[i] * kRadToDeg);
    }
}

static void print_joint_limits(const float* minimum, const float* maximum,
                               size_t count) {
    for (size_t i = 0; i < count; ++i) {
        printf("  J%zu: [%.2f, %.2f] rad  ([%.2f, %.2f] deg)\n", i + 1,
               minimum[i], maximum[i], minimum[i] * kRadToDeg,
               maximum[i] * kRadToDeg);
    }
}

static int wait_until_target(dual_arm_control_api_t* api,
                             const example_velocity_guard_t* velocity_guard,
                             const float* target, size_t target_count,
                             int timeout_seconds) {
    const float kPositionTolerance = 0.5F * kDegToRad;
    int elapsed_ms = 0;
    int stable_samples = 0;

    while (elapsed_ms < timeout_seconds * 1000) {
        if (example_velocity_guard_triggered(velocity_guard)) {
            return 0;
        }

        float position[kMaxJoints];
        size_t pos_count = 0;
        const dual_arm_ret_code_t code = dual_arm_get_joint(
            api, DUAL_ARM_GROUP_LEFT_ARM, position, kMaxJoints, &pos_count);
        if (code == DUAL_ARM_RET_SUCCESS && pos_count == target_count) {
            float largest_error = 0.0F;
            for (size_t i = 0; i < target_count; ++i) {
                largest_error = float_max(
                    largest_error, float_abs(position[i] - target[i]));
            }
            stable_samples =
                largest_error <= kPositionTolerance ? stable_samples + 1 : 0;
            if (stable_samples >= 10) {
                printf("Target reached.\n");
                return 1;
            }
        }
        sleep_ms(10);
        elapsed_ms += 10;
    }

    fprintf(stderr, "Timed out waiting for the target position.\n");
    return 0;
}

static void fill_constant(float* out, size_t count, float value) {
    for (size_t i = 0; i < count; ++i) {
        out[i] = value;
    }
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

    float current[kMaxJoints];
    size_t joint_count = 0;
    printf("Waiting for rt_control state...\n");
    if (!wait_for_joint_state(api, DUAL_ARM_GROUP_LEFT_ARM, current,
                              kMaxJoints, &joint_count, kDefaultStateTimeoutMs)) {
        fprintf(stderr,
                "No state received within 3 seconds. Is rt_control running, "
                "and do the UDP addresses/ports match config.yaml?\n");
        dual_arm_control_api_destroy(api);
        return 1;
    }

    printf("Current left-arm joints:\n");
    print_joints(current, joint_count);

    dual_arm_state_t states[kMaxJoints];
    size_t state_count = 0;
    const dual_arm_ret_code_t state_code = dual_arm_get_group_state(
        api, DUAL_ARM_GROUP_LEFT_ARM, states, kMaxJoints, &state_count);
    if (!check_result("get_group_state", state_code)) {
        dual_arm_control_api_destroy(api);
        return 1;
    }
    for (size_t i = 0; i < state_count; ++i) {
        printf("  J%zu: %s\n", i + 1, state_name(states[i]));
    }

    float minimum_position[kMaxJoints];
    float maximum_position[kMaxJoints];
    size_t min_count = 0;
    size_t max_count = 0;
    if (dual_arm_get_joint_min_limit(api, DUAL_ARM_GROUP_LEFT_ARM,
                                     minimum_position, kMaxJoints,
                                     &min_count) != DUAL_ARM_RET_SUCCESS ||
        dual_arm_get_joint_max_limit(api, DUAL_ARM_GROUP_LEFT_ARM,
                                     maximum_position, kMaxJoints,
                                     &max_count) != DUAL_ARM_RET_SUCCESS ||
        min_count != joint_count || max_count != joint_count) {
        fprintf(stderr, "Failed to read left-arm joint position limits.\n");
        dual_arm_control_api_destroy(api);
        return 1;
    }
    printf("Left-arm joint position limits:\n");
    print_joint_limits(minimum_position, maximum_position, joint_count);

    float target_10_deg[kMaxJoints];
    float target_20_deg[kMaxJoints];
    float target_0_deg[kMaxJoints];
    fill_constant(target_10_deg, joint_count, 10.0F * kDegToRad);
    fill_constant(target_20_deg, joint_count, 20.0F * kDegToRad);
    fill_constant(target_0_deg, joint_count, 0.0F);

    printf("Test sequence for every left-arm joint:\n"
           "  low-follow: current position -> 10 deg\n"
           "  high-follow direct: 10 deg -> 20 deg\n"
           "  high-follow 1 Hz filter: 20 deg -> 0 deg\n");
    printf("Safety guard: if any left-arm joint exceeds 30 deg/s, "
           "for 3 consecutive samples, hold the current position "
           "and stop the test.\n");

    example_velocity_guard_t velocity_guard;
    if (!example_velocity_guard_start(&velocity_guard, api,
                                      DUAL_ARM_GROUP_LEFT_ARM,
                                      kMaxJointVelocity)) {
        fprintf(stderr, "Failed to start velocity guard thread.\n");
        dual_arm_control_api_destroy(api);
        return 1;
    }

    if (!wait_for_enter(
            "Stage 1/3: enable the left arm and execute low-follow "
            "point-to-point motion.")) {
        example_velocity_guard_stop(&velocity_guard);
        dual_arm_control_api_destroy(api);
        return 0;
    }
    if (example_velocity_guard_triggered(&velocity_guard)) {
        example_velocity_guard_stop(&velocity_guard);
        dual_arm_control_api_destroy(api);
        return 1;
    }
    if (!check_result("set_group(enable)",
                      dual_arm_set_group(api, DUAL_ARM_GROUP_LEFT_ARM, 1))) {
        example_velocity_guard_stop(&velocity_guard);
        dual_arm_control_api_destroy(api);
        return 1;
    }

    const float kTestMaximumSpeed = 20.0F * kDegToRad;
    float requested_speed[kMaxJoints];
    float applied_speed[kMaxJoints];
    size_t applied_speed_count = 0;
    fill_constant(requested_speed, joint_count, kTestMaximumSpeed);
    if (dual_arm_set_joint_max_speed(api, DUAL_ARM_GROUP_LEFT_ARM,
                                     requested_speed, joint_count,
                                     applied_speed, kMaxJoints,
                                     &applied_speed_count) !=
            DUAL_ARM_RET_SUCCESS ||
        applied_speed_count != joint_count) {
        fprintf(stderr, "set_joint_max_speed failed.\n");
        example_velocity_guard_stop(&velocity_guard);
        dual_arm_control_api_destroy(api);
        return 1;
    }
    printf("set_joint_max_speed: 20 deg/s (%.6f rad/s) for every left-arm "
           "joint\n",
           kTestMaximumSpeed);

    const float kTestMaximumAcceleration = 50.0F * kDegToRad;
    float requested_acceleration[kMaxJoints];
    float applied_acceleration[kMaxJoints];
    size_t applied_acc_count = 0;
    fill_constant(requested_acceleration, joint_count,
                  kTestMaximumAcceleration);
    if (dual_arm_set_joint_max_acc(api, DUAL_ARM_GROUP_LEFT_ARM,
                                   requested_acceleration, joint_count,
                                   applied_acceleration, kMaxJoints,
                                   &applied_acc_count) != DUAL_ARM_RET_SUCCESS ||
        applied_acc_count != joint_count) {
        fprintf(stderr, "set_joint_max_acc failed.\n");
        example_velocity_guard_stop(&velocity_guard);
        dual_arm_control_api_destroy(api);
        return 1;
    }
    printf("set_joint_max_acc: 50 deg/s^2 (%.6f rad/s^2) for every left-arm "
           "joint [currently stored in SDK only]\n",
           kTestMaximumAcceleration);

    if (!check_result("move_joint(low-follow)",
                      dual_arm_move_joint(api, DUAL_ARM_GROUP_LEFT_ARM,
                                          target_10_deg, joint_count, 0, 0,
                                          0))) {
        example_velocity_guard_stop(&velocity_guard);
        dual_arm_control_api_destroy(api);
        return 1;
    }
    if (!wait_until_target(api, &velocity_guard, target_10_deg, joint_count,
                           15)) {
        example_velocity_guard_stop(&velocity_guard);
        dual_arm_control_api_destroy(api);
        return 1;
    }

    if (!wait_for_enter(
            "Stage 2/3: execute high-follow direct streaming for 2 seconds "
            "at 100 Hz.")) {
        example_velocity_guard_stop(&velocity_guard);
        dual_arm_control_api_destroy(api);
        return 0;
    }
    if (example_velocity_guard_triggered(&velocity_guard)) {
        example_velocity_guard_stop(&velocity_guard);
        dual_arm_control_api_destroy(api);
        return 1;
    }

    for (int i = 0; i < 200; ++i) {
        if (example_velocity_guard_triggered(&velocity_guard)) {
            example_velocity_guard_stop(&velocity_guard);
            dual_arm_control_api_destroy(api);
            return 1;
        }
        const float ratio = (float)(i + 1) / 200.0F;
        float target[kMaxJoints];
        fill_constant(target, joint_count,
                      (10.0F + 10.0F * ratio) * kDegToRad);
        const dual_arm_ret_code_t code = dual_arm_move_joint(
            api, DUAL_ARM_GROUP_LEFT_ARM, target, joint_count, 1, 0, 0);
        if (code != DUAL_ARM_RET_SUCCESS) {
            check_result("move_joint(high-follow, direct)", code);
            example_velocity_guard_stop(&velocity_guard);
            dual_arm_control_api_destroy(api);
            return 1;
        }
        sleep_ms(10);
    }
    check_result("move_joint(high-follow, direct)", DUAL_ARM_RET_SUCCESS);
    if (!wait_until_target(api, &velocity_guard, target_20_deg, joint_count,
                           3)) {
        example_velocity_guard_stop(&velocity_guard);
        dual_arm_control_api_destroy(api);
        return 1;
    }

    if (!wait_for_enter(
            "Stage 3/3: execute high-follow low-frequency filtering for "
            "2 seconds at 100 Hz (radio=999, cutoff=1 Hz).")) {
        example_velocity_guard_stop(&velocity_guard);
        dual_arm_control_api_destroy(api);
        return 0;
    }
    if (example_velocity_guard_triggered(&velocity_guard)) {
        example_velocity_guard_stop(&velocity_guard);
        dual_arm_control_api_destroy(api);
        return 1;
    }

    for (int i = 0; i < 200; ++i) {
        if (example_velocity_guard_triggered(&velocity_guard)) {
            example_velocity_guard_stop(&velocity_guard);
            dual_arm_control_api_destroy(api);
            return 1;
        }
        const float ratio = (float)(i + 1) / 200.0F;
        float target[kMaxJoints];
        fill_constant(target, joint_count,
                      (20.0F - 20.0F * ratio) * kDegToRad);
        const dual_arm_ret_code_t code = dual_arm_move_joint(
            api, DUAL_ARM_GROUP_LEFT_ARM, target, joint_count, 1, 2, 999);
        if (code != DUAL_ARM_RET_SUCCESS) {
            check_result("move_joint(high-follow, 1 Hz filter)", code);
            example_velocity_guard_stop(&velocity_guard);
            dual_arm_control_api_destroy(api);
            return 1;
        }
        sleep_ms(10);
    }
    check_result("move_joint(high-follow, 1 Hz filter)", DUAL_ARM_RET_SUCCESS);
    if (example_velocity_guard_triggered(&velocity_guard)) {
        example_velocity_guard_stop(&velocity_guard);
        dual_arm_control_api_destroy(api);
        return 1;
    }
    if (!wait_until_target(api, &velocity_guard, target_0_deg, joint_count,
                           10)) {
        example_velocity_guard_stop(&velocity_guard);
        dual_arm_control_api_destroy(api);
        return 1;
    }

    float final_joints[kMaxJoints];
    size_t final_count = 0;
    const dual_arm_ret_code_t joint_code = dual_arm_get_joint(
        api, DUAL_ARM_GROUP_LEFT_ARM, final_joints, kMaxJoints, &final_count);
    if (!check_result("get_joint(final)", joint_code)) {
        example_velocity_guard_stop(&velocity_guard);
        dual_arm_control_api_destroy(api);
        return 1;
    }
    printf("Final left-arm joints:\n");
    print_joints(final_joints, final_count);

    example_velocity_guard_stop(&velocity_guard);
    printf("Smoke test completed. The arm remains enabled.\n");
    dual_arm_control_api_destroy(api);
    return 0;
}
