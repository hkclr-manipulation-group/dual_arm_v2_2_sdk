/**
 * @file move_arms_demo.c
 * @brief 单臂 / 双臂运动演示（C API）。
 */

#include "dual_arm_control_api.h"
#include "example_common.h"

#include <stdio.h>

static const float kLeftOffsetDeg[7] = {10, 10, 10, 10, 10, 10, 10};
static const float kRightOffsetDeg[7] = {10, 10, 10, 10, 10, 10, 10};
static const float kMaxSpeedDegS = 20.0F;
static const float kPositionToleranceDeg = 0.5F;
static const int kReachTimeoutSeconds = 15;
static const int kPauseBeforeBothSeconds = 2;

static void current_plus_offset(const float* current, size_t count,
                                const float* offset_deg, float* target) {
    for (size_t i = 0; i < count && i < 7; ++i) {
        target[i] = current[i] + offset_deg[i] * kDegToRad;
    }
}

static void print_arm_report(const char* arm_label, const float* target,
                             const float* actual, size_t count) {
    printf("  %s actual (deg):  [", arm_label);
    for (size_t i = 0; i < count; ++i) {
        printf("%.3f", actual[i] * kRadToDeg);
        if (i + 1 < count) printf(", ");
    }
    printf("]\n  %s target (deg): [", arm_label);
    for (size_t i = 0; i < count; ++i) {
        printf("%.3f", target[i] * kRadToDeg);
        if (i + 1 < count) printf(", ");
    }
    printf("]\n  %s error   (deg): [", arm_label);
    float max_err = 0.0F;
    for (size_t i = 0; i < count; ++i) {
        const float err = (actual[i] - target[i]) * kRadToDeg;
        max_err = float_max(max_err, float_abs(err));
        printf("%.3f", err);
        if (i + 1 < count) printf(", ");
    }
    printf("]\n  %s max |error| = %.3f deg%s\n", arm_label, max_err,
           max_err <= kPositionToleranceDeg ? "  [OK]" : "  [NOT REACHED]");
}

static int wait_until_settled(dual_arm_control_api_t* api, int check_left,
                              int check_right, const float* left_target,
                              const float* right_target, size_t joint_count) {
    const float tolerance_rad = kPositionToleranceDeg * kDegToRad;
    int elapsed_ms = 0;
    int stable = 0;

    while (elapsed_ms <= kReachTimeoutSeconds * 1000) {
        float left_current[kMaxJoints];
        float right_current[kMaxJoints];
        size_t left_count = 0;
        size_t right_count = 0;
        const dual_arm_ret_code_t left_code = dual_arm_get_joint(
            api, DUAL_ARM_GROUP_LEFT_ARM, left_current, kMaxJoints,
            &left_count);
        const dual_arm_ret_code_t right_code = dual_arm_get_joint(
            api, DUAL_ARM_GROUP_RIGHT_ARM, right_current, kMaxJoints,
            &right_count);

        if ((check_left && left_code != DUAL_ARM_RET_SUCCESS) ||
            (check_right && right_code != DUAL_ARM_RET_SUCCESS) ||
            (check_left && left_count != joint_count) ||
            (check_right && right_count != joint_count)) {
            sleep_ms(200);
            elapsed_ms += 200;
            continue;
        }

        float max_error = 0.0F;
        if (check_left) {
            max_error = float_max(
                max_error, max_joint_error(left_target, left_current, joint_count));
        }
        if (check_right) {
            max_error = float_max(
                max_error,
                max_joint_error(right_target, right_current, joint_count));
        }

        if (max_error <= tolerance_rad) {
            if (++stable >= 3) {
                return 1;
            }
        } else {
            stable = 0;
        }
        sleep_ms(200);
        elapsed_ms += 200;
    }
    return 0;
}

static int move_and_report(dual_arm_control_api_t* api, dual_arm_group_t group,
                           const float* command, size_t command_count,
                           int check_left, int check_right,
                           const float* left_target, const float* right_target,
                           size_t joint_count, const char* label) {
    printf("\n==> %s\n", label);
    if (dual_arm_move_joint(api, group, command, command_count, 0, 0, 0) !=
        DUAL_ARM_RET_SUCCESS) {
        fprintf(stderr, "move_joint failed\n");
        return 0;
    }

    if (!wait_until_settled(api, check_left, check_right, left_target,
                            right_target, joint_count)) {
        printf("  WARNING: timed out after %d s waiting to settle\n",
               kReachTimeoutSeconds);
    }

    float left_actual[kMaxJoints];
    float right_actual[kMaxJoints];
    size_t left_count = 0;
    size_t right_count = 0;
    if (!wait_for_joint_state(api, DUAL_ARM_GROUP_LEFT_ARM, left_actual,
                              kMaxJoints, &left_count, kDefaultStateTimeoutMs) ||
        !wait_for_joint_state(api, DUAL_ARM_GROUP_RIGHT_ARM, right_actual,
                              kMaxJoints, &right_count, kDefaultStateTimeoutMs)) {
        fprintf(stderr, "Failed to read joint feedback after move\n");
        return 0;
    }

    if (check_left) {
        print_arm_report("LEFT ", left_target, left_actual, joint_count);
    }
    if (check_right) {
        print_arm_report("RIGHT", right_target, right_actual, joint_count);
    }
    if (!check_left && !check_right) {
        printf("  (no reach check for this step)\n");
    }
    return 1;
}

int main(void) {
    char config_path[kMaxConfigPathLen];
    default_config_path(config_path, sizeof(config_path));

    dual_arm_control_api_t* api = dual_arm_control_api_create(config_path);
    if (api == NULL) {
        fprintf(stderr, "Failed to create API: %s\n",
                dual_arm_control_api_last_error(api));
        return 1;
    }

    float left_now[kMaxJoints];
    float right_now[kMaxJoints];
    float head_now[kMaxJoints];
    size_t left_count = 0;
    size_t right_count = 0;
    size_t head_count = 0;

    if (!wait_for_joint_state(api, DUAL_ARM_GROUP_LEFT_ARM, left_now,
                              kMaxJoints, &left_count, kDefaultStateTimeoutMs) ||
        !wait_for_joint_state(api, DUAL_ARM_GROUP_RIGHT_ARM, right_now,
                              kMaxJoints, &right_count, kDefaultStateTimeoutMs) ||
        !wait_for_joint_state(api, DUAL_ARM_GROUP_HEAD, head_now, kMaxJoints,
                              &head_count, kDefaultStateTimeoutMs)) {
        fprintf(stderr, "No rt_control state within 3 seconds.\n");
        dual_arm_control_api_destroy(api);
        return 1;
    }

    if (dual_arm_set_group(api, DUAL_ARM_GROUP_LEFT_ARM, 1) !=
            DUAL_ARM_RET_SUCCESS ||
        dual_arm_set_group(api, DUAL_ARM_GROUP_RIGHT_ARM, 1) !=
            DUAL_ARM_RET_SUCCESS) {
        fprintf(stderr, "Failed to enable arms.\n");
        dual_arm_control_api_destroy(api);
        return 1;
    }

    float speed[kMaxJoints];
    float out_speed[kMaxJoints];
    size_t speed_count = left_count;
    for (size_t i = 0; i < speed_count; ++i) {
        speed[i] = kMaxSpeedDegS * kDegToRad;
    }
    size_t out_count = 0;
    if (dual_arm_set_joint_max_speed(api, DUAL_ARM_GROUP_LEFT_ARM, speed,
                                     speed_count, out_speed, kMaxJoints,
                                     &out_count) != DUAL_ARM_RET_SUCCESS ||
        out_count != speed_count ||
        dual_arm_set_joint_max_speed(api, DUAL_ARM_GROUP_RIGHT_ARM, speed,
                                     speed_count, out_speed, kMaxJoints,
                                     &out_count) != DUAL_ARM_RET_SUCCESS ||
        out_count != speed_count) {
        fprintf(stderr, "set_joint_max_speed failed.\n");
        dual_arm_control_api_destroy(api);
        return 1;
    }

    float left_target[kMaxJoints];
    float right_target[kMaxJoints];
    current_plus_offset(left_now, left_count, kLeftOffsetDeg, left_target);
    current_plus_offset(right_now, right_count, kRightOffsetDeg, right_target);

    if (!move_and_report(api, DUAL_ARM_GROUP_LEFT_ARM, left_target, left_count,
                         1, 0, left_target, right_target, left_count,
                         "Move LEFT arm only")) {
        dual_arm_control_api_destroy(api);
        return 1;
    }
    if (!move_and_report(api, DUAL_ARM_GROUP_RIGHT_ARM, right_target,
                         right_count, 0, 1, left_target, right_target,
                         right_count, "Move RIGHT arm only")) {
        dual_arm_control_api_destroy(api);
        return 1;
    }

    printf("\nPause %d s, then move both arms together from current pose...\n",
           kPauseBeforeBothSeconds);
    sleep_ms(kPauseBeforeBothSeconds * 1000);

    if (!wait_for_joint_state(api, DUAL_ARM_GROUP_LEFT_ARM, left_now,
                              kMaxJoints, &left_count, kDefaultStateTimeoutMs) ||
        !wait_for_joint_state(api, DUAL_ARM_GROUP_RIGHT_ARM, right_now,
                              kMaxJoints, &right_count, kDefaultStateTimeoutMs) ||
        !wait_for_joint_state(api, DUAL_ARM_GROUP_HEAD, head_now, kMaxJoints,
                              &head_count, kDefaultStateTimeoutMs)) {
        fprintf(stderr, "Failed to read joint state before BOTH move.\n");
        dual_arm_control_api_destroy(api);
        return 1;
    }

    float left_both[kMaxJoints];
    float right_both[kMaxJoints];
    current_plus_offset(left_now, left_count, kLeftOffsetDeg, left_both);
    current_plus_offset(right_now, right_count, kRightOffsetDeg, right_both);

    float all[kMaxJoints];
    size_t all_count = 0;
    concat_three_groups(left_both, left_count, head_now, head_count, right_both,
                        right_count, all, kMaxJoints, &all_count);
    if (!move_and_report(api, DUAL_ARM_GROUP_ALL, all, all_count, 1, 1,
                         left_both, right_both, left_count,
                         "Move BOTH arms together")) {
        dual_arm_control_api_destroy(api);
        return 1;
    }

    float zero[kMaxJoints];
    for (size_t i = 0; i < left_count; ++i) {
        zero[i] = 0.0F;
    }
    float home[kMaxJoints];
    size_t home_count = 0;
    concat_three_groups(zero, left_count, head_now, head_count, zero,
                        left_count, home, kMaxJoints, &home_count);
    if (!move_and_report(api, DUAL_ARM_GROUP_ALL, home, home_count, 1, 1, zero,
                         zero, left_count, "Go home (both arms zero)")) {
        dual_arm_control_api_destroy(api);
        return 1;
    }

    printf("\nDemo done.\n");
    dual_arm_control_api_destroy(api);
    return 0;
}
