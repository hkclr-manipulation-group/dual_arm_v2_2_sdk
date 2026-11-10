/**
 * @file test_control_api_various_command_frequency.c
 * @brief 高跟随模式下不同发送频率（20~30 Hz）的行为对比测试。
 *
 * 用法（在 build/bin/ 目录下）：
 *   ./test_control_api_various_command_frequency
 */

#include "dual_arm_control_api.h"
#include "example_common.h"
#include "example_velocity_guard.h"

#include <math.h>
#include <stdio.h>
#include <time.h>

static const float kMaxJointVelocity = 30.0F * kDegToRad;

/* 33 ms is about 30 Hz, 50 ms is 20 Hz. Both high-follow tests use the same
 * deterministic pattern, making their behavior easier to compare. */
static const int kVariablePeriodsMs[12] = {33, 50, 36, 47, 34, 44,
                                           39, 49, 35, 42, 38, 46};

static double monotonic_seconds(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

static void timespec_add_ms(struct timespec* ts, int ms) {
    ts->tv_sec += ms / 1000;
    ts->tv_nsec += (long)(ms % 1000) * 1000000L;
    if (ts->tv_nsec >= 1000000000L) {
        ts->tv_sec += 1;
        ts->tv_nsec -= 1000000000L;
    }
}

static int timespec_ge(const struct timespec* a, const struct timespec* b) {
    if (a->tv_sec != b->tv_sec) {
        return a->tv_sec > b->tv_sec;
    }
    return a->tv_nsec >= b->tv_nsec;
}

static int wait_until_target(dual_arm_control_api_t* api,
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
        if (code == DUAL_ARM_RET_SUCCESS && pos_count == target_count) {
            float maximum_error = 0.0F;
            for (size_t joint = 0; joint < target_count; ++joint) {
                maximum_error = float_max(
                    maximum_error,
                    float_abs(position[joint] - target[joint]));
            }
            stable_samples =
                maximum_error <= kTolerance ? stable_samples + 1 : 0;
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

static int send_variable_frequency_trajectory(
    dual_arm_control_api_t* api, const example_velocity_guard_t* guard,
    size_t joint_count, float start_deg, float finish_deg,
    uint8_t trajectory_mode, uint16_t radio, const char* label) {
    const float kDurationSeconds = 8.0F;
    const double start_time = monotonic_seconds();
    size_t period_index = 0;
    struct timespec next_report;
    clock_gettime(CLOCK_MONOTONIC, &next_report);

    while (1) {
        if (example_velocity_guard_triggered(guard)) {
            return 0;
        }

        const double now = monotonic_seconds();
        const float elapsed = (float)(now - start_time);
        const float progress =
            elapsed / kDurationSeconds < 1.0F ? elapsed / kDurationSeconds
                                              : 1.0F;

        /* This source trajectory is smooth in real time. Direct mode exposes
         * irregular command arrivals; filter mode smooths their steps. */
        const float smooth_progress = 0.5F - 0.5F * cosf(kPi * progress);
        const float command_deg =
            start_deg + (finish_deg - start_deg) * smooth_progress;
        float target[kMaxJoints];
        for (size_t i = 0; i < joint_count; ++i) {
            target[i] = command_deg * kDegToRad;
        }

        const dual_arm_ret_code_t code = dual_arm_move_joint(
            api, DUAL_ARM_GROUP_LEFT_ARM, target, joint_count, 1,
            trajectory_mode, radio, 0);
        if (code != DUAL_ARM_RET_SUCCESS) {
            fprintf(stderr, "%s send failed: %s\n", label,
                    dual_arm_ret_code_name(code));
            return 0;
        }

        struct timespec now_ts;
        clock_gettime(CLOCK_MONOTONIC, &now_ts);
        if (timespec_ge(&now_ts, &next_report)) {
            float position[kMaxJoints];
            size_t pos_count = 0;
            const dual_arm_ret_code_t position_code = dual_arm_get_joint(
                api, DUAL_ARM_GROUP_LEFT_ARM, position, kMaxJoints, &pos_count);
            printf("%s  t=%.2f s, period=%d ms, command=%.2f deg", label,
                   elapsed, kVariablePeriodsMs[period_index], command_deg);
            if (position_code == DUAL_ARM_RET_SUCCESS && pos_count > 0) {
                printf(", actual J1=%.2f deg", position[0] * kRadToDeg);
            }
            printf("\n");
            timespec_add_ms(&next_report, 1000);
        }

        if (progress >= 1.0F) {
            break;
        }
        sleep_ms(kVariablePeriodsMs[period_index]);
        period_index = (period_index + 1) % 12;
    }
    return 1;
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
        fprintf(stderr, "No rt_control state received within 3 seconds.\n");
        dual_arm_control_api_destroy(api);
        return 1;
    }

    printf("Current left-arm position:\n");
    for (size_t joint = 0; joint < joint_count; ++joint) {
        printf("  J%zu: %.3f deg\n", joint + 1,
               current[joint] * kRadToDeg);
    }

    printf("\nTiming pattern: 33-50 ms per command (about 30-20 Hz).\n"
           "Safety: hold position and stop if any joint exceeds "
           "30 deg/s for 3 consecutive samples.\n");

    float target_10[kMaxJoints];
    float target_20[kMaxJoints];
    fill_constant(target_10, joint_count, 10.0F * kDegToRad);
    fill_constant(target_20, joint_count, 20.0F * kDegToRad);

    example_velocity_guard_t guard;
    if (!example_velocity_guard_start(&guard, api, DUAL_ARM_GROUP_LEFT_ARM,
                                      kMaxJointVelocity)) {
        fprintf(stderr, "Failed to start velocity guard thread.\n");
        dual_arm_control_api_destroy(api);
        return 1;
    }

    if (!wait_for_enter(
            "Stage 1/4: low-follow point-to-point, current -> 10 deg.")) {
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 0;
    }

    dual_arm_ret_code_t code =
        dual_arm_set_group(api, DUAL_ARM_GROUP_LEFT_ARM, 1);
    printf("set_group(enable): %s\n", dual_arm_ret_code_name(code));
    if (code != DUAL_ARM_RET_SUCCESS) {
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 1;
    }

    code = dual_arm_move_joint(api, DUAL_ARM_GROUP_LEFT_ARM, target_10,
                               joint_count, 0, 0, 0, 0);
    printf("move_joint(low-follow): %s (the call only waits for command "
           "acknowledgement)\n",
           dual_arm_ret_code_name(code));
    if (code != DUAL_ARM_RET_SUCCESS ||
        !wait_until_target(api, &guard, target_10, joint_count, 20)) {
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 1;
    }

    if (!wait_for_enter(
            "Stage 2/4: high-follow direct, 10 -> 20 deg with variable "
            "frequency.")) {
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 0;
    }
    if (!send_variable_frequency_trajectory(api, &guard, joint_count, 10.0F,
                                            20.0F, 0, 0, "direct")) {
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 1;
    }
    if (!wait_until_target(api, &guard, target_20, joint_count, 5)) {
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 1;
    }

    if (!wait_for_enter("Stage 3/4: low-follow reset, 20 -> 10 deg.")) {
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 0;
    }
    code = dual_arm_move_joint(api, DUAL_ARM_GROUP_LEFT_ARM, target_10,
                               joint_count, 0, 0, 0, 0);
    printf("move_joint(low-follow reset): %s\n", dual_arm_ret_code_name(code));
    if (code != DUAL_ARM_RET_SUCCESS ||
        !wait_until_target(api, &guard, target_10, joint_count, 20)) {
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 1;
    }

    if (!wait_for_enter(
            "Stage 4/4: high-follow 1 Hz filter, 10 -> 20 deg with the same "
            "variable frequency.")) {
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 0;
    }
    if (!send_variable_frequency_trajectory(api, &guard, joint_count, 10.0F,
                                            20.0F, 2, 999, "filter")) {
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 1;
    }
    if (!wait_until_target(api, &guard, target_20, joint_count, 15)) {
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 1;
    }

    example_velocity_guard_stop(&guard);
    printf("Variable-frequency test completed. The arm remains enabled.\n");
    dual_arm_control_api_destroy(api);
    return 0;
}
