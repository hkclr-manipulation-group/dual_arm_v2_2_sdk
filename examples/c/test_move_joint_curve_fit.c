/**
 * @file test_move_joint_curve_fit.c
 * @brief move_joint 曲线拟合模式（trajectory_mode=1）演示。
 *
 * 映射到 rt_control kSinglePoint + kQuintic；InterpolationAccTime = 10ms + radio/100。
 * 流式发送时命令间隔应 >= acc_time。
 *
 * Part 1：单次曲线拟合，radio=20（快）vs radio=100（平滑），仅 J1 运动
 * Part 2：低速路点流（间隔 = 1.2 * acc_time）
 *
 * 用法（在 build/bin/ 目录下）：
 *   ./test_move_joint_curve_fit
 */

#include "dual_arm_control_api.h"
#include "example_common.h"
#include "example_velocity_guard.h"

#include <stdio.h>
#include <time.h>

static const float kTestMaxSpeedDegS = 50.0F;
static const float kTestMaxSpeedRadS = kTestMaxSpeedDegS * kDegToRad;
/* Guard slightly above the configured speed limit. */
static const float kGuardVelocityDegS = kTestMaxSpeedDegS + 5.0F;
static const float kGuardVelocityRadS = kGuardVelocityDegS * kDegToRad;
static const float kPositionTolerance = 0.5F * kDegToRad;

static const float kJ1PrepOffsetDeg = 15.0F;
static const float kJ1MidOffsetDeg = 25.0F;
static const float kJ1FinalOffsetDeg = 35.0F;
static const float kStreamStepDeg = 5.0F;
static const uint16_t kFastRadio = 20;
static const uint16_t kSmoothRadio = 100;

static float curve_fit_acc_time(uint16_t radio) {
    return 0.01F + (float)radio / 100.0F;
}

static int curve_fit_interval_ms(uint16_t radio) {
    return (int)(curve_fit_acc_time(radio) * 1200.0F);
}

static int configure_arm_speed(dual_arm_control_api_t* api, size_t joint_count) {
    float requested_speed[kMaxJoints];
    float applied_speed[kMaxJoints];
    size_t applied_count = 0;
    for (size_t i = 0; i < joint_count; ++i) {
        requested_speed[i] = kTestMaxSpeedRadS;
    }
    if (dual_arm_set_joint_max_speed(api, DUAL_ARM_GROUP_LEFT_ARM,
                                     requested_speed, joint_count,
                                     applied_speed, kMaxJoints,
                                     &applied_count) != DUAL_ARM_RET_SUCCESS ||
        applied_count != joint_count) {
        fprintf(stderr, "set_joint_max_speed failed.\n");
        return 0;
    }
    printf("set_joint_max_speed: %.0f deg/s for every left-arm joint\n",
           kTestMaxSpeedDegS);
    return 1;
}

static void j1_offset_target(const float* reference, size_t count,
                             float j1_offset_deg, float* target) {
    for (size_t i = 0; i < count; ++i) {
        target[i] = reference[i];
    }
    if (count > 0) {
        target[0] += j1_offset_deg * kDegToRad;
    }
}

static int timespec_ge(const struct timespec* a, const struct timespec* b) {
    if (a->tv_sec != b->tv_sec) {
        return a->tv_sec > b->tv_sec;
    }
    return a->tv_nsec >= b->tv_nsec;
}

static void timespec_add_ms(struct timespec* ts, int ms) {
    ts->tv_sec += ms / 1000;
    ts->tv_nsec += (long)(ms % 1000) * 1000000L;
    if (ts->tv_nsec >= 1000000000L) {
        ts->tv_sec += 1;
        ts->tv_nsec -= 1000000000L;
    }
}

static void print_motion_sample(const char* label, float elapsed_seconds,
                                const float* target, size_t target_count,
                                const float* position, size_t pos_count,
                                const float* velocity, size_t vel_count) {
    printf("%s  t=%.2f s", label, elapsed_seconds);
    if (pos_count > 0) {
        printf(", actual J1=%.2f deg", position[0] * kRadToDeg);
    }
    if (target_count > 0) {
        printf(", target J1=%.2f deg", target[0] * kRadToDeg);
    }
    if (vel_count > 0) {
        printf(", vel J1=%.2f deg/s", velocity[0] * kRadToDeg);
    }
    if (pos_count > 0 && target_count > 0) {
        printf(", err J1=%.2f deg",
               (position[0] - target[0]) * kRadToDeg);
    }
    printf("\n");
}

static int wait_until_target(dual_arm_control_api_t* api,
                             const example_velocity_guard_t* guard,
                             const float* target, size_t target_count,
                             int timeout_seconds, const char* label,
                             int print_progress) {
    struct timespec start_time;
    struct timespec next_report;
    clock_gettime(CLOCK_MONOTONIC, &start_time);
    next_report = start_time;
    int elapsed_ms = 0;
    int stable_samples = 0;

    while (elapsed_ms < timeout_seconds * 1000) {
        if (example_velocity_guard_triggered(guard)) {
            return 0;
        }

        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);

        float position[kMaxJoints];
        size_t pos_count = 0;
        const dual_arm_ret_code_t code = dual_arm_get_joint(
            api, DUAL_ARM_GROUP_LEFT_ARM, position, kMaxJoints, &pos_count);
        if (code == DUAL_ARM_RET_SUCCESS && pos_count == target_count) {
            const float error = max_joint_error(target, position, target_count);
            stable_samples =
                error <= kPositionTolerance ? stable_samples + 1 : 0;

            if (print_progress && label != NULL &&
                timespec_ge(&now, &next_report)) {
                float velocity[kMaxJoints];
                size_t vel_count = 0;
                const dual_arm_ret_code_t velocity_code =
                    dual_arm_get_joint_velocity(
                        api, DUAL_ARM_GROUP_LEFT_ARM, velocity, kMaxJoints,
                        &vel_count);
                const float elapsed =
                    (float)(now.tv_sec - start_time.tv_sec) +
                    (float)(now.tv_nsec - start_time.tv_nsec) / 1e9F;
                print_motion_sample(
                    label, elapsed, target, target_count, position, pos_count,
                    velocity_code == DUAL_ARM_RET_SUCCESS ? velocity : NULL,
                    velocity_code == DUAL_ARM_RET_SUCCESS ? vel_count : 0);
                timespec_add_ms(&next_report, 200);
            }

            if (stable_samples >= 10) {
                if (label != NULL) {
                    const float elapsed =
                        (float)(now.tv_sec - start_time.tv_sec) +
                        (float)(now.tv_nsec - start_time.tv_nsec) / 1e9F;
                    printf("%s reached target in %.2f s.\n", label, elapsed);
                } else {
                    printf("Target reached.\n");
                }
                return 1;
            }
        }
        sleep_ms(10);
        elapsed_ms += 10;
    }

    fprintf(stderr, "Timed out waiting for the target position.\n");
    return 0;
}

static int move_low_follow(dual_arm_control_api_t* api,
                           const example_velocity_guard_t* guard,
                           const float* target, size_t target_count,
                           const char* label, int timeout_seconds) {
    printf("%s\n", label);
    const dual_arm_ret_code_t code = dual_arm_move_joint(
        api, DUAL_ARM_GROUP_LEFT_ARM, target, target_count, 0, 0, 0);
    printf("  move_joint(low-follow): %s\n", dual_arm_ret_code_name(code));
    return code == DUAL_ARM_RET_SUCCESS &&
           wait_until_target(api, guard, target, target_count, timeout_seconds,
                             NULL, 0);
}

static int move_curve_fit_single(dual_arm_control_api_t* api,
                                 const example_velocity_guard_t* guard,
                                 const float* target, size_t target_count,
                                 uint16_t radio, const char* label,
                                 int timeout_seconds) {
    const float acc_time = curve_fit_acc_time(radio);
    printf("%s radio=%u, acc_time=%.3f s (single command)\n", label, radio,
           acc_time);

    const dual_arm_ret_code_t code = dual_arm_move_joint(
        api, DUAL_ARM_GROUP_LEFT_ARM, target, target_count, 1, 1, radio);
    if (code != DUAL_ARM_RET_SUCCESS) {
        fprintf(stderr, "  move_joint(curve-fit): %s\n",
                dual_arm_ret_code_name(code));
        return 0;
    }

    return wait_until_target(api, guard, target, target_count, timeout_seconds,
                             label, 1);
}

static int stream_curve_fit_waypoints(
    dual_arm_control_api_t* api, const example_velocity_guard_t* guard,
    const float* home_pose, size_t joint_count, float start_offset_deg,
    float finish_offset_deg, float step_deg, uint16_t radio,
    const char* label) {
    const float acc_time = curve_fit_acc_time(radio);
    const int interval_ms = curve_fit_interval_ms(radio);
    printf("%s radio=%u, acc_time=%.3f s, waypoint interval=%d ms\n", label,
           radio, acc_time, interval_ms);

    float waypoints_deg[kMaxJoints];
    size_t waypoint_count = 0;
    for (float deg = start_offset_deg; deg <= finish_offset_deg + 1e-3F;
         deg += step_deg) {
        if (waypoint_count < kMaxJoints) {
            waypoints_deg[waypoint_count++] = deg;
        }
    }

    for (size_t index = 0; index < waypoint_count; ++index) {
        if (example_velocity_guard_triggered(guard)) {
            return 0;
        }

        float target[kMaxJoints];
        j1_offset_target(home_pose, joint_count, waypoints_deg[index], target);
        const dual_arm_ret_code_t code = dual_arm_move_joint(
            api, DUAL_ARM_GROUP_LEFT_ARM, target, joint_count, 1, 1, radio);
        if (code != DUAL_ARM_RET_SUCCESS) {
            fprintf(stderr, "  waypoint %zu/%zu send failed: %s\n", index + 1,
                    waypoint_count, dual_arm_ret_code_name(code));
            return 0;
        }

        float position[kMaxJoints];
        float velocity[kMaxJoints];
        size_t pos_count = 0;
        size_t vel_count = 0;
        const dual_arm_ret_code_t position_code = dual_arm_get_joint(
            api, DUAL_ARM_GROUP_LEFT_ARM, position, kMaxJoints, &pos_count);
        const dual_arm_ret_code_t velocity_code = dual_arm_get_joint_velocity(
            api, DUAL_ARM_GROUP_LEFT_ARM, velocity, kMaxJoints, &vel_count);
        printf("  waypoint %zu/%zu: J1 offset cmd=%.2f deg", index + 1,
               waypoint_count, waypoints_deg[index]);
        if (position_code == DUAL_ARM_RET_SUCCESS && pos_count > 0) {
            printf(", actual J1=%.2f deg", position[0] * kRadToDeg);
        }
        if (velocity_code == DUAL_ARM_RET_SUCCESS && vel_count > 0) {
            printf(", vel J1=%.2f deg/s", velocity[0] * kRadToDeg);
        }
        printf("\n");

        if (index + 1 < waypoint_count) {
            sleep_ms(interval_ms);
        }
    }

    float final_target[kMaxJoints];
    j1_offset_target(home_pose, joint_count, finish_offset_deg, final_target);
    return wait_until_target(api, guard, final_target, joint_count, 20, label,
                             1);
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

    float home_pose[kMaxJoints];
    size_t joint_count = 0;
    printf("move_joint curve-fit mode test (trajectory_mode=1)\n"
           "Waiting for rt_control state...\n");
    if (!wait_for_joint_state(api, DUAL_ARM_GROUP_LEFT_ARM, home_pose,
                              kMaxJoints, &joint_count, kDefaultStateTimeoutMs)) {
        fprintf(stderr, "No rt_control state received within 3 seconds.\n");
        dual_arm_control_api_destroy(api);
        return 1;
    }

    printf("Home pose (only J1 will move relative to this):\n");
    for (size_t joint = 0; joint < joint_count; ++joint) {
        printf("  J%zu: %.3f deg\n", joint + 1,
               home_pose[joint] * kRadToDeg);
    }

    printf("\nMotion plan: J1 offset %.0f -> %.0f -> %.0f deg; other joints "
           "fixed.\n"
           "low-follow uses config.yaml panel.general.acc_time "
           "(recommend >= 1 s).\n"
           "Safety guard: direct hold if any joint exceeds %.0f deg/s for 3 "
           "consecutive samples.\n",
           kJ1PrepOffsetDeg, kJ1MidOffsetDeg, kJ1FinalOffsetDeg,
           kGuardVelocityDegS);

    float target_prep[kMaxJoints];
    float target_mid[kMaxJoints];
    float target_final[kMaxJoints];
    j1_offset_target(home_pose, joint_count, kJ1PrepOffsetDeg, target_prep);
    j1_offset_target(home_pose, joint_count, kJ1MidOffsetDeg, target_mid);
    j1_offset_target(home_pose, joint_count, kJ1FinalOffsetDeg, target_final);

    example_velocity_guard_t guard;
    if (!example_velocity_guard_start(&guard, api, DUAL_ARM_GROUP_LEFT_ARM,
                                      kGuardVelocityRadS)) {
        fprintf(stderr, "Failed to start velocity guard thread.\n");
        dual_arm_control_api_destroy(api);
        return 1;
    }

    if (!wait_for_enter(
            "Stage 1/6: enable left arm, set max speed, J1 -> +15 deg "
            "(low-follow).")) {
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 0;
    }
    if (dual_arm_set_group(api, DUAL_ARM_GROUP_LEFT_ARM, 1) !=
        DUAL_ARM_RET_SUCCESS) {
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 1;
    }
    if (!configure_arm_speed(api, joint_count)) {
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 1;
    }
    if (!move_low_follow(api, &guard, target_prep, joint_count,
                         "Prepare: J1 +15 deg", 25)) {
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 1;
    }

    if (!wait_for_enter(
            "Stage 2/6: curve-fit FAST radio=20 (acc~210 ms), "
            "J1 +15 -> +25 deg (10 deg). Watch: quick snap.")) {
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 0;
    }
    if (!move_curve_fit_single(api, &guard, target_mid, joint_count, kFastRadio,
                               "curve-fit-fast", 20)) {
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 1;
    }

    if (!wait_for_enter("Stage 3/6: reset J1 to +15 deg (low-follow).")) {
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 0;
    }
    if (!move_low_follow(api, &guard, target_prep, joint_count,
                         "Reset to J1 +15 deg", 25)) {
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 1;
    }

    if (!wait_for_enter(
            "Stage 4/6: curve-fit SMOOTH radio=100 (acc~1.01 s), "
            "J1 +15 -> +35 deg (20 deg). Watch: slow smooth arc.")) {
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 0;
    }
    if (!move_curve_fit_single(api, &guard, target_final, joint_count,
                               kSmoothRadio, "curve-fit-smooth", 30)) {
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 1;
    }

    if (!wait_for_enter(
            "Stage 5/6: reset to +15 deg, then curve-fit waypoint stream "
            "radio=100 (5 deg steps, interval ~1.2 s). Watch: stepped creep.")) {
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 0;
    }
    if (!move_low_follow(api, &guard, target_prep, joint_count,
                         "Reset to J1 +15 deg", 25)) {
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 1;
    }
    if (!stream_curve_fit_waypoints(api, &guard, home_pose, joint_count,
                                    kJ1PrepOffsetDeg, kJ1FinalOffsetDeg,
                                    kStreamStepDeg, kSmoothRadio,
                                    "curve-fit-stream")) {
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 1;
    }

    if (!wait_for_enter("Stage 6/6: return to home pose (low-follow).")) {
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 0;
    }
    if (!move_low_follow(api, &guard, home_pose, joint_count, "Return home",
                         30)) {
        example_velocity_guard_stop(&guard);
        dual_arm_control_api_destroy(api);
        return 1;
    }

    float final_joints[kMaxJoints];
    size_t final_count = 0;
    const dual_arm_ret_code_t final_code = dual_arm_get_joint(
        api, DUAL_ARM_GROUP_LEFT_ARM, final_joints, kMaxJoints, &final_count);
    if (final_code == DUAL_ARM_RET_SUCCESS && final_count > 0) {
        printf("Final left-arm J1: %.2f deg\n",
               final_joints[0] * kRadToDeg);
    }

    example_velocity_guard_stop(&guard);
    printf("Curve-fit mode test completed. The arm remains enabled.\n");
    dual_arm_control_api_destroy(api);
    return 0;
}
