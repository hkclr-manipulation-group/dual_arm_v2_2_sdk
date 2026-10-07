/**
 * @file staggered_dual_arm_stream.c
 * @brief 错峰双臂高跟随：左先动 → 右再动（重叠）→ 左先回零 → 右再回零。
 *
 * 控制：set_left/right_joint_target + start_dual_arm_streaming (SDK auto flush)
 *
 * 用法（build/bin/c/ 下）：
 *   ./staggered_dual_arm_stream
 *   ./staggered_dual_arm_stream --left 20 --right 20
 *   ./staggered_dual_arm_stream --left 10,20,30,40,50,60,70 --right 15
 *   ./staggered_dual_arm_stream --right-lag 2 --right-return-lag 2 --move-duration 4
 *   ./staggered_dual_arm_stream /path/to/cuarm_configuration/dual_v2_2
 */

#include "dual_arm_control_api.h"
#include "example_common.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

enum {
    kControlHz = 100,
    kPeriodMs = 10,
    kDefaultMoveDurationS = 4,
    kDefaultRightLagS = 2,
    kDefaultHoldS = 1,
    kDefaultRightReturnLagS = 2,
    kMaxErrLen = 256,
};

static const float kDefaultOffsetDeg = 20.0F;
static const float kMaxSpeedDegS = 30.0F;
static const uint8_t kTrajectoryMode = 2;
static const uint16_t kRadio = 999;

typedef struct {
    float start[kMaxJoints];
    float target[kMaxJoints];
    float cmd[kMaxJoints];
    size_t joint_count;
    float offset_deg[kMaxJoints];
    size_t offset_count;
} ArmPlan;

typedef struct {
    int has_left;
    int has_right;
    float left_deg[kMaxJoints];
    size_t left_count;
    float right_deg[kMaxJoints];
    size_t right_count;
    float move_duration_s;
    float right_lag_s;
    float hold_s;
    float right_return_lag_s;
    int show_help;
} ParsedArgs;

static double monotonic_seconds(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

static void sleep_until_monotonic(const struct timespec* deadline) {
    struct timespec now;
    while (1) {
        clock_gettime(CLOCK_MONOTONIC, &now);
        if (now.tv_sec > deadline->tv_sec ||
            (now.tv_sec == deadline->tv_sec &&
             now.tv_nsec >= deadline->tv_nsec)) {
            break;
        }
        struct timespec rem;
        rem.tv_sec = deadline->tv_sec - now.tv_sec;
        rem.tv_nsec = deadline->tv_nsec - now.tv_nsec;
        if (rem.tv_nsec < 0) {
            rem.tv_sec -= 1;
            rem.tv_nsec += 1000000000L;
        }
        nanosleep(&rem, NULL);
    }
}

static void timespec_add_ms(struct timespec* ts, int ms) {
    ts->tv_sec += ms / 1000;
    ts->tv_nsec += (long)(ms % 1000) * 1000000L;
    if (ts->tv_nsec >= 1000000000L) {
        ts->tv_sec += 1;
        ts->tv_nsec -= 1000000000L;
    }
}

static float smooth_step(float u) {
    if (u <= 0.0F) {
        return 0.0F;
    }
    if (u >= 1.0F) {
        return 1.0F;
    }
    return 0.5F - 0.5F * cosf(kPi * u);
}

static int parse_offset_spec(const char* text, float* out, size_t* out_count,
                             char* err, size_t err_size) {
    *out_count = 0;
    char buffer[256];
    snprintf(buffer, sizeof(buffer), "%s", text);

    char* save = NULL;
    char* token = strtok_r(buffer, ",", &save);
    while (token != NULL) {
        token = trim_inplace(token);
        if (token[0] == '\0') {
            snprintf(err, err_size, "empty value in offset list '%s'", text);
            return 0;
        }
        char* end = NULL;
        const float value = strtof(token, &end);
        if (end == token || (end != NULL && *end != '\0')) {
            snprintf(err, err_size, "invalid number '%s' in '%s'", token, text);
            return 0;
        }
        if (*out_count >= kMaxJoints) {
            snprintf(err, err_size, "too many offsets in '%s'", text);
            return 0;
        }
        out[(*out_count)++] = value;
        token = strtok_r(NULL, ",", &save);
    }
    return *out_count > 0;
}

static int resolve_offsets(const float* user, size_t user_count,
                           size_t joint_count, float* out, size_t* out_count,
                           char* err, size_t err_size, const char* label) {
    if (user_count == 0) {
        for (size_t i = 0; i < joint_count; ++i) {
            out[i] = kDefaultOffsetDeg;
        }
        *out_count = joint_count;
        return 1;
    }
    if (user_count == 1) {
        for (size_t i = 0; i < joint_count; ++i) {
            out[i] = user[0];
        }
        *out_count = joint_count;
        return 1;
    }
    if (user_count != joint_count) {
        snprintf(err, err_size,
                 "%s offset count must be 1 or %zu, got %zu", label,
                 joint_count, user_count);
        return 0;
    }
    memcpy(out, user, joint_count * sizeof(float));
    *out_count = joint_count;
    return 1;
}

static void build_arm_plan(ArmPlan* plan, const float* start,
                           const float* offset_deg, size_t offset_count) {
    plan->offset_count = offset_count;
    memcpy(plan->offset_deg, offset_deg, offset_count * sizeof(float));
    memcpy(plan->start, start, plan->joint_count * sizeof(float));
    memcpy(plan->cmd, start, plan->joint_count * sizeof(float));
    for (size_t i = 0; i < plan->joint_count; ++i) {
        plan->target[i] = start[i] + offset_deg[i] * kDegToRad;
    }
}

static void lerp_pose(float* out, const float* a, const float* b, float u,
                      size_t n) {
    const float s = smooth_step(u);
    for (size_t i = 0; i < n; ++i) {
        out[i] = a[i] + (b[i] - a[i]) * s;
    }
}

static void fill_outbound_cmd(ArmPlan* plan, double t, double out_start,
                              double out_end) {
    if (t <= out_start) {
        memcpy(plan->cmd, plan->start, plan->joint_count * sizeof(float));
        return;
    }
    if (t >= out_end) {
        memcpy(plan->cmd, plan->target, plan->joint_count * sizeof(float));
        return;
    }
    const float u = (float)((t - out_start) / (out_end - out_start));
    lerp_pose(plan->cmd, plan->start, plan->target, u, plan->joint_count);
}

static void fill_return_cmd(ArmPlan* plan, double t, double home_start,
                            double home_end) {
    static float zero[kMaxJoints];
    if (t <= home_start) {
        memcpy(plan->cmd, plan->target, plan->joint_count * sizeof(float));
        return;
    }
    if (t >= home_end) {
        memset(plan->cmd, 0, plan->joint_count * sizeof(float));
        return;
    }
    const float u = (float)((t - home_start) / (home_end - home_start));
    lerp_pose(plan->cmd, plan->target, zero, u, plan->joint_count);
}

static void fill_left_cmd(ArmPlan* plan, double t, double out_start,
                          double out_end, double home_start, double home_end) {
    if (t < home_start) {
        fill_outbound_cmd(plan, t, out_start, out_end);
    } else {
        fill_return_cmd(plan, t, home_start, home_end);
    }
}

static void fill_right_cmd(ArmPlan* plan, double t, double out_start,
                           double out_end, double home_start, double home_end) {
    if (t < out_start) {
        memcpy(plan->cmd, plan->start, plan->joint_count * sizeof(float));
        return;
    }
    if (t < home_start) {
        fill_outbound_cmd(plan, t, out_start, out_end);
        return;
    }
    fill_return_cmd(plan, t, home_start, home_end);
}

static int write_streaming_targets(dual_arm_control_api_t* api,
                                   const ArmPlan* left, const ArmPlan* right) {
    if (dual_arm_set_left_joint_target(api, left->cmd, left->joint_count) !=
        DUAL_ARM_RET_SUCCESS) {
        return 0;
    }
    if (dual_arm_set_right_joint_target(api, right->cmd, right->joint_count) !=
        DUAL_ARM_RET_SUCCESS) {
        return 0;
    }
    return 1;
}

static int configure_speed(dual_arm_control_api_t* api, dual_arm_group_t group,
                           size_t joint_count) {
    float speed[kMaxJoints];
    float applied[kMaxJoints];
    size_t applied_count = 0;
    for (size_t i = 0; i < joint_count; ++i) {
        speed[i] = kMaxSpeedDegS * kDegToRad;
    }
    return dual_arm_set_joint_max_speed(api, group, speed, joint_count,
                                        applied, kMaxJoints,
                                        &applied_count) == DUAL_ARM_RET_SUCCESS &&
           applied_count == joint_count;
}

static void print_usage(const char* prog) {
    printf(
        "Usage: %s [config_prefix] [options]\n"
        "  Staggered dual-arm high-follow via set_left/right + SDK streaming.\n"
        "\n"
        "  --left  <deg>| <d1,d2,...>   left offset from start (default +20/joint)\n"
        "  --right <deg>| <d1,d2,...>   right offset (default +20/joint)\n"
        "  --move-duration <sec>       each move segment duration (default %.1f)\n"
        "  --right-lag <sec>           delay before right starts outbound (default %.1f)\n"
        "  --hold <sec>                hold at target before return (default %.1f)\n"
        "  --right-return-lag <sec>    delay before right starts return (default %.1f)\n"
        "\n"
        "Timeline:\n"
        "  t=0           left outbound\n"
        "  t=right-lag   right outbound (overlap while left still moving)\n"
        "  both at target, hold\n"
        "  left return to absolute zero\n"
        "  +right-return-lag  right return to zero\n"
        "\n"
        "Examples:\n"
        "  %s --left 20 --right 20\n"
        "  %s --left 10,20,30,40,50,60,70 --right 15 --right-lag 1.5\n",
        prog, (double)kDefaultMoveDurationS, (double)kDefaultRightLagS,
        (double)kDefaultHoldS, (double)kDefaultRightReturnLagS, prog, prog);
}

static int parse_args(int argc, char** argv, ParsedArgs* out, char* err,
                      size_t err_size) {
    memset(out, 0, sizeof(*out));
    out->move_duration_s = (float)kDefaultMoveDurationS;
    out->right_lag_s = (float)kDefaultRightLagS;
    out->hold_s = (float)kDefaultHoldS;
    out->right_return_lag_s = (float)kDefaultRightReturnLagS;

    for (int i = 1; i < argc; ++i) {
        const char* arg = argv[i];
        if (strcmp(arg, "--help") == 0 || strcmp(arg, "-h") == 0) {
            out->show_help = 1;
            return 1;
        }
        if (strcmp(arg, "--left") == 0 || strcmp(arg, "-l") == 0) {
            if (i + 1 >= argc) {
                snprintf(err, err_size, "%s requires a value", arg);
                return 0;
            }
            if (!parse_offset_spec(argv[++i], out->left_deg, &out->left_count,
                                   err, err_size)) {
                return 0;
            }
            out->has_left = 1;
            continue;
        }
        if (strcmp(arg, "--right") == 0 || strcmp(arg, "-r") == 0) {
            if (i + 1 >= argc) {
                snprintf(err, err_size, "%s requires a value", arg);
                return 0;
            }
            if (!parse_offset_spec(argv[++i], out->right_deg, &out->right_count,
                                   err, err_size)) {
                return 0;
            }
            out->has_right = 1;
            continue;
        }
        if (strcmp(arg, "--move-duration") == 0) {
            if (i + 1 >= argc) {
                snprintf(err, err_size, "%s requires a value", arg);
                return 0;
            }
            out->move_duration_s = strtof(argv[++i], NULL);
            continue;
        }
        if (strcmp(arg, "--right-lag") == 0) {
            if (i + 1 >= argc) {
                snprintf(err, err_size, "%s requires a value", arg);
                return 0;
            }
            out->right_lag_s = strtof(argv[++i], NULL);
            continue;
        }
        if (strcmp(arg, "--hold") == 0) {
            if (i + 1 >= argc) {
                snprintf(err, err_size, "%s requires a value", arg);
                return 0;
            }
            out->hold_s = strtof(argv[++i], NULL);
            continue;
        }
        if (strcmp(arg, "--right-return-lag") == 0) {
            if (i + 1 >= argc) {
                snprintf(err, err_size, "%s requires a value", arg);
                return 0;
            }
            out->right_return_lag_s = strtof(argv[++i], NULL);
            continue;
        }
        if (arg[0] == '-') {
            snprintf(err, err_size, "unknown option '%s'", arg);
            return 0;
        }
    }
    return 1;
}

int main(int argc, char** argv) {
    ParsedArgs args;
    char err[kMaxErrLen];
    if (!parse_args(argc, argv, &args, err, sizeof(err))) {
        fprintf(stderr, "Argument error: %s\n", err);
        return 1;
    }
    if (args.show_help) {
        print_usage(argv[0]);
        return 0;
    }

    char config_path[kMaxConfigPathLen];
    config_path_from_args(argc, argv, config_path, sizeof(config_path));

    dual_arm_control_api_t* api = dual_arm_control_api_create(config_path);
    if (api == NULL) {
        fprintf(stderr, "Failed to create API: %s\n",
                dual_arm_control_api_last_error(api));
        return 1;
    }

    float left_start[kMaxJoints];
    float right_start[kMaxJoints];
    size_t left_count = 0;
    size_t right_count = 0;

    if (!wait_for_joint_state(api, DUAL_ARM_GROUP_LEFT_ARM, left_start,
                              kMaxJoints, &left_count, kDefaultStateTimeoutMs) ||
        !wait_for_joint_state(api, DUAL_ARM_GROUP_RIGHT_ARM, right_start,
                              kMaxJoints, &right_count, kDefaultStateTimeoutMs)) {
        fprintf(stderr, "No rt_control state within 3 seconds.\n");
        dual_arm_control_api_destroy(api);
        return 1;
    }

    if (dual_arm_set_group(api, DUAL_ARM_GROUP_LEFT_ARM, 1) !=
            DUAL_ARM_RET_SUCCESS ||
        dual_arm_set_group(api, DUAL_ARM_GROUP_RIGHT_ARM, 1) !=
            DUAL_ARM_RET_SUCCESS ||
        !configure_speed(api, DUAL_ARM_GROUP_LEFT_ARM, left_count) ||
        !configure_speed(api, DUAL_ARM_GROUP_RIGHT_ARM, right_count)) {
        fprintf(stderr, "Failed to enable arms or set speed.\n");
        dual_arm_control_api_destroy(api);
        return 1;
    }

    float left_offsets[kMaxJoints];
    float right_offsets[kMaxJoints];
    size_t left_offset_count = 0;
    size_t right_offset_count = 0;
    if (!resolve_offsets(args.has_left ? args.left_deg : NULL,
                         args.has_left ? args.left_count : 0, left_count,
                         left_offsets, &left_offset_count, err, sizeof(err),
                         "left") ||
        !resolve_offsets(args.has_right ? args.right_deg : NULL,
                         args.has_right ? args.right_count : 0, right_count,
                         right_offsets, &right_offset_count, err, sizeof(err),
                         "right")) {
        fprintf(stderr, "%s\n", err);
        dual_arm_control_api_destroy(api);
        return 1;
    }

    ArmPlan left;
    ArmPlan right;
    memset(&left, 0, sizeof(left));
    memset(&right, 0, sizeof(right));
    left.joint_count = left_count;
    right.joint_count = right_count;
    build_arm_plan(&left, left_start, left_offsets, left_offset_count);
    build_arm_plan(&right, right_start, right_offsets, right_offset_count);

    const double move_d = (double)args.move_duration_s;
    const double right_lag = (double)args.right_lag_s;
    const double hold = (double)args.hold_s;
    const double right_return_lag = (double)args.right_return_lag_s;

    const double t_left_out_start = 0.0;
    const double t_right_out_start = right_lag;
    const double t_left_out_end = t_left_out_start + move_d;
    const double t_right_out_end = t_right_out_start + move_d;
    const double t_return_start =
        (t_left_out_end > t_right_out_end ? t_left_out_end : t_right_out_end) +
        hold;
    const double t_left_home_start = t_return_start;
    const double t_right_home_start = t_return_start + right_return_lag;
    const double t_left_home_end = t_left_home_start + move_d;
    const double t_right_home_end = t_right_home_start + move_d;
    const double t_total =
        (t_left_home_end > t_right_home_end ? t_left_home_end
                                            : t_right_home_end) +
        0.5;

    const double overlap_out_start = t_right_out_start;
    const double overlap_out_end =
        t_left_out_end < t_right_out_end ? t_left_out_end : t_right_out_end;
    const double overlap_ret_start = t_right_home_start;
    const double overlap_ret_end =
        t_left_home_end < t_right_home_end ? t_left_home_end : t_right_home_end;

    printf("Config: %s\n", config_path);
    printf("High-follow streaming @ %d Hz, trajectory_mode=%u radio=%u\n",
           kControlHz, (unsigned)kTrajectoryMode, (unsigned)kRadio);
    printf("Timeline (s):\n");
    printf("  left  outbound: [%.2f, %.2f]\n", t_left_out_start, t_left_out_end);
    printf("  right outbound: [%.2f, %.2f]\n", t_right_out_start,
           t_right_out_end);
    if (overlap_out_end > overlap_out_start) {
        printf("  overlap outbound: [%.2f, %.2f]\n", overlap_out_start,
               overlap_out_end);
    }
    printf("  left  return:    [%.2f, %.2f]\n", t_left_home_start,
           t_left_home_end);
    printf("  right return:    [%.2f, %.2f]\n", t_right_home_start,
           t_right_home_end);
    if (overlap_ret_end > overlap_ret_start) {
        printf("  overlap return:  [%.2f, %.2f]\n", overlap_ret_start,
               overlap_ret_end);
    }
    printf("  total duration:  %.2f s\n\n", t_total);

    if (dual_arm_start_dual_arm_streaming(api, 1, kTrajectoryMode, kRadio,
                                          kControlHz) != DUAL_ARM_RET_SUCCESS) {
        fprintf(stderr, "start_dual_arm_streaming failed\n");
        dual_arm_control_api_destroy(api);
        return 1;
    }

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    const double t0 = monotonic_seconds();

    while (1) {
        const double t = monotonic_seconds() - t0;
        if (t >= t_total) {
            break;
        }

        fill_left_cmd(&left, t, t_left_out_start, t_left_out_end,
                      t_left_home_start, t_left_home_end);
        fill_right_cmd(&right, t, t_right_out_start, t_right_out_end,
                       t_right_home_start, t_right_home_end);

        if (!write_streaming_targets(api, &left, &right)) {
            fprintf(stderr, "streaming write failed at t=%.3f\n", t);
            break;
        }

        timespec_add_ms(&next, kPeriodMs);
        sleep_until_monotonic(&next);
    }

    memset(left.cmd, 0, left.joint_count * sizeof(float));
    memset(right.cmd, 0, right.joint_count * sizeof(float));
    write_streaming_targets(api, &left, &right);
    sleep_ms(500);

    dual_arm_stop_dual_arm_streaming(api);
    printf("Done.\n");
    dual_arm_control_api_destroy(api);
    return 0;
}
