/**
 * @file test_dual_arm_100hz_follow.c
 * @brief 左右臂各自 100 Hz 高跟随读写测试（C API 移植）
 *
 * 对齐 ros2_control JointGroupBase 的调用方式：
 *   read : get_joint / get_joint_velocity / get_group_state
 *   write: move_joint(...) 或 set_joint_target(...) + flush_joint_command(...)
 *
 * 轨迹：从当前姿态平滑偏移到目标（相对起始角 +deg）；到位后双臂同时回零。
 *
 * 用法（在 build/bin/ 目录下）：
 *   ./test_dual_arm_100hz_follow              # 交互：left -> right -> both
 *   ./test_dual_arm_100hz_follow left
 *   ./test_dual_arm_100hz_follow both --left 20 --right 30
 *   ./test_dual_arm_100hz_follow all --left 10,20,30,40,50,60,70 --right 20
 *   ./test_dual_arm_100hz_follow both-flush   # set_joint_target + flush @100 Hz
 *
 * 目标偏移（可选，单位 deg，相对 stage 起始姿态）：
 *   --left  <deg>              左臂各轴相同偏移
 *   --left  <d1,d2,...,dN>     左臂逐轴偏移（N = 关节数，通常 7）
 *   --right <deg> | <d1,...>   右臂同上
 *   未指定时：left 模式默认 J1..J7 为 +10..70；其余模式默认各轴 +20。
 */

#include "dual_arm_control_api.h"
#include "example_common.h"

#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

enum {
    kControlHz = 100,
    kPeriodMs = 1000 / 100,
    kDebugEveryN = 50,
    kHoldAtTargetSeconds = 2,
    kReturnTimeoutSeconds = 20,
    kMaxStages = 5,
    kMaxModeLen = 32,
    kMaxErrLen = 256,
    kMaxPromptLen = 1024,
};

static const float kDurationSeconds = 8.0F;
static const float kLeftAmplitudeDeg = 20.0F;
static const float kRightAmplitudeDeg = 20.0F;
static const float kLeftPerJointAmplitudeDeg[7] = {
    10.0F, 20.0F, 30.0F, 40.0F, 50.0F, 60.0F, 70.0F};
static const float kMaxSpeedDegS = 30.0F;
static const float kGuardVelocityDegS = 35.0F;
static const float kReachFraction = 0.90F;
static const uint8_t kTrajectoryMode = 2;
static const uint16_t kRadio = 999;

typedef struct {
    float cmd_offset_deg;
    float mean_act_offset_deg;
    float mean_lag_deg;
    float mean_abs_vel_deg_s;
    size_t act_count;
    float act_offset_deg[kMaxJoints];
} Sample;

typedef struct {
    Sample last;
    double sum_abs_lag;
    double sum_abs_vel;
    float max_abs_vel;
    float max_abs_lag;
    int samples;
    int read_fail;
    int write_fail;
    float t_cmd_90;
    float t_act_90;
    double sum_cycle_ms;
    int cycles;
} ArmStats;

typedef struct {
    dual_arm_group_t group;
    const char* name;
    float amplitude_deg;
    int has_per_joint_amplitude;
    size_t per_joint_count;
    float amplitude_deg_per_joint[kMaxJoints];
    size_t joint_count;
    float start[kMaxJoints];
    float cmd[kMaxJoints];
    float pos[kMaxJoints];
    float vel[kMaxJoints];
    ArmStats stats;
} ArmContext;

typedef struct {
    int has_left;
    int has_right;
    float left_deg[kMaxJoints];
    size_t left_count;
    float right_deg[kMaxJoints];
    size_t right_count;
} TargetOverrides;

typedef struct {
    char mode[kMaxModeLen];
    int has_mode;
    TargetOverrides targets;
    int show_help;
} ParsedArgs;

typedef enum {
    DRIVE_SEQUENTIAL = 0,
    DRIVE_SEQUENTIAL_BURST,
    DRIVE_THREADED,
    DRIVE_COALESCED_FLUSH,
    DRIVE_GROUP_ALL,
} DriveMode;

typedef struct {
    dual_arm_control_api_t* api;
    ArmContext* arm;
    struct timespec start_time;
    int delay_ms;
    int thread_period_ms;
    volatile int* stop;
    volatile int* safety;
} ThreadWorkerCtx;

static double timespec_diff_seconds(const struct timespec* end,
                                    const struct timespec* start) {
    return (double)(end->tv_sec - start->tv_sec) +
           (double)(end->tv_nsec - start->tv_nsec) * 1e-9;
}

static double timespec_diff_ms(const struct timespec* end,
                               const struct timespec* start) {
    return timespec_diff_seconds(end, start) * 1000.0;
}

static void timespec_add_ms(struct timespec* ts, int ms) {
    ts->tv_sec += ms / 1000;
    ts->tv_nsec += (long)(ms % 1000) * 1000000L;
    if (ts->tv_nsec >= 1000000000L) {
        ts->tv_sec += 1;
        ts->tv_nsec -= 1000000000L;
    }
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

static void default_left_offsets_deg(const char* stage, float* out,
                                     size_t* out_count) {
    if (strcmp(stage, "left") == 0) {
        *out_count = 7;
        memcpy(out, kLeftPerJointAmplitudeDeg, sizeof(kLeftPerJointAmplitudeDeg));
        return;
    }
    out[0] = kLeftAmplitudeDeg;
    *out_count = 1;
}

static void default_right_offsets_deg(const char* stage, float* out,
                                      size_t* out_count) {
    (void)stage;
    out[0] = kRightAmplitudeDeg;
    *out_count = 1;
}

static void print_usage(const char* prog) {
    printf("Usage: %s [mode] [--left <spec>] [--right <spec>]\n"
           "  mode: left | right | both | both-burst | both-thread | "
           "both-flush | all\n"
           "        (default: interactive left -> right -> both)\n"
           "  --left, --right: offset in deg from stage start pose.\n"
           "        One number  -> same offset on every joint.\n"
           "        Comma list  -> per-joint offsets (count must match arm).\n"
           "  Defaults when omitted:\n"
           "        left mode:  L +[10,20,30,40,50,60,70] deg\n"
           "        other modes: L/R +20 deg/joint\n"
           "  Examples:\n"
           "    %s left --left 15\n"
           "    %s both --left 20 --right 30\n"
           "    %s all --left 10,20,30,40,50,60,70 --right 20\n"
           "    %s both-flush --left 20 --right 20\n",
           prog, prog, prog, prog, prog);
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
    if (*out_count == 0) {
        snprintf(err, err_size, "empty offset spec");
        return 0;
    }
    return 1;
}

static int parse_args(int argc, char** argv, ParsedArgs* out, char* err,
                      size_t err_size) {
    memset(out, 0, sizeof(*out));
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
            if (!parse_offset_spec(argv[++i], out->targets.left_deg,
                                   &out->targets.left_count, err, err_size)) {
                return 0;
            }
            out->targets.has_left = 1;
            continue;
        }
        if (strcmp(arg, "--right") == 0 || strcmp(arg, "-r") == 0) {
            if (i + 1 >= argc) {
                snprintf(err, err_size, "%s requires a value", arg);
                return 0;
            }
            if (!parse_offset_spec(argv[++i], out->targets.right_deg,
                                   &out->targets.right_count, err, err_size)) {
                return 0;
            }
            out->targets.has_right = 1;
            continue;
        }
        if (arg[0] == '-') {
            snprintf(err, err_size, "unknown option '%s'", arg);
            return 0;
        }
        if (out->has_mode) {
            snprintf(err, err_size, "unexpected argument '%s'", arg);
            return 0;
        }
        snprintf(out->mode, sizeof(out->mode), "%s", arg);
        out->has_mode = 1;
    }
    return 1;
}

static int resolve_offsets(int has_user, const float* user, size_t user_count,
                           const float* stage_default, size_t default_count,
                           size_t joint_count, float* out, size_t* out_count,
                           char* err, size_t err_size, const char* arm_label) {
    const float* src = has_user ? user : stage_default;
    const size_t src_count = has_user ? user_count : default_count;
    if (src_count == 0) {
        snprintf(err, err_size, "%s offset list is empty", arm_label);
        return 0;
    }
    if (src_count == 1) {
        for (size_t i = 0; i < joint_count; ++i) {
            out[i] = src[0];
        }
        *out_count = joint_count;
        return 1;
    }
    if (src_count != joint_count) {
        snprintf(err, err_size,
                 "%s offset count must be 1 or %zu, got %zu", arm_label,
                 joint_count, src_count);
        return 0;
    }
    memcpy(out, src, joint_count * sizeof(float));
    *out_count = joint_count;
    return 1;
}

static void apply_offsets_to_arm(ArmContext* arm, const float* offsets_deg,
                                 size_t offset_count) {
    int uniform = 1;
    for (size_t i = 1; i < offset_count; ++i) {
        if (float_abs(offsets_deg[i] - offsets_deg[0]) >= 1e-4F) {
            uniform = 0;
            break;
        }
    }
    if (uniform) {
        arm->amplitude_deg = offsets_deg[0];
        arm->has_per_joint_amplitude = 0;
        arm->per_joint_count = 0;
    } else {
        arm->has_per_joint_amplitude = 1;
        arm->per_joint_count = offset_count;
        memcpy(arm->amplitude_deg_per_joint, offsets_deg,
               offset_count * sizeof(float));
        float max_a = offsets_deg[0];
        for (size_t i = 1; i < offset_count; ++i) {
            if (offsets_deg[i] > max_a) {
                max_a = offsets_deg[i];
            }
        }
        arm->amplitude_deg = max_a;
    }
}

static void format_deg_list(const float* values, size_t count, char* out,
                            size_t out_size) {
    size_t written = 0;
    written += (size_t)snprintf(out + written, out_size - written, "[");
    for (size_t i = 0; i < count; ++i) {
        written += (size_t)snprintf(out + written, out_size - written, "%s%.1f",
                                    i == 0 ? "" : ", ", values[i]);
        if (written >= out_size) {
            return;
        }
    }
    snprintf(out + written, out_size - written, "]");
}

static void format_offset_desc(const float* offsets_deg, size_t count, char* out,
                               size_t out_size) {
    if (count == 0) {
        snprintf(out, out_size, "no offset");
        return;
    }
    int uniform = 1;
    for (size_t i = 1; i < count; ++i) {
        if (float_abs(offsets_deg[i] - offsets_deg[0]) >= 1e-4F) {
            uniform = 0;
            break;
        }
    }
    if (uniform) {
        snprintf(out, out_size, "+%.0f deg/joint", offsets_deg[0]);
        return;
    }
    char list[256];
    format_deg_list(offsets_deg, count, list, sizeof(list));
    snprintf(out, out_size, "+deg %s", list);
}

static void offset_target(float* out, const float* start, size_t n,
                          float amplitude_deg, float scale) {
    const float delta = amplitude_deg * kDegToRad * scale;
    for (size_t i = 0; i < n; ++i) {
        out[i] = start[i] + delta;
    }
}

static void offset_target_per_joint(float* out, const float* start,
                                    const float* amplitude_deg, size_t n,
                                    float scale) {
    for (size_t i = 0; i < n; ++i) {
        out[i] = start[i] + amplitude_deg[i] * kDegToRad * scale;
    }
}

static void print_pose(const char* label, const float* joints, size_t count) {
    printf("  %s (deg): [", label);
    for (size_t i = 0; i < count; ++i) {
        printf("%.2f%s", joints[i] * kRadToDeg, i + 1 < count ? ", " : "");
    }
    printf("]\n");
}

static void format_arm_target_label(const ArmContext* arm, char* out,
                                    size_t out_size) {
    if (arm->has_per_joint_amplitude) {
        char list[256];
        format_deg_list(arm->amplitude_deg_per_joint, arm->per_joint_count,
                        list, sizeof(list));
        snprintf(out, out_size, "target +deg %s", list);
        return;
    }
    snprintf(out, out_size, "target +%.0f deg/joint", arm->amplitude_deg);
}

static void describe_stage_motion(const char* name, int use_left, int use_right,
                                  const float* left_offsets,
                                  size_t left_offset_count,
                                  const float* right_offsets,
                                  size_t right_offset_count, char* out,
                                  size_t out_size) {
    char left_desc[128];
    char right_desc[128];
    format_offset_desc(left_offsets, left_offset_count, left_desc,
                       sizeof(left_desc));
    format_offset_desc(right_offsets, right_offset_count, right_desc,
                       sizeof(right_desc));

    if (strcmp(name, "left") == 0) {
        snprintf(out, out_size,
                 "8 s high-follow: @100 Hz left %s; right/head held.",
                 left_desc);
    } else if (strcmp(name, "right") == 0) {
        snprintf(out, out_size,
                 "8 s high-follow: @100 Hz right %s; left/head held.",
                 right_desc);
    } else if (strcmp(name, "both") == 0) {
        snprintf(out, out_size,
                 "8 s high-follow: staggered move_joint (left, 10 ms gap, "
                 "right); L %s, R %s (~50 Hz/arm).",
                 left_desc, right_desc);
    } else if (strcmp(name, "both-burst") == 0) {
        snprintf(out, out_size,
                 "8 s high-follow: burst left then right in one cycle; second "
                 "UDP packet usually dropped (first arm tracks); L %s, R %s.",
                 left_desc, right_desc);
    } else if (strcmp(name, "both-thread") == 0) {
        snprintf(out, out_size,
                 "8 s high-follow: dual-thread 50 Hz each; L %s, R %s.",
                 left_desc, right_desc);
    } else if (strcmp(name, "both-flush") == 0) {
        snprintf(out, out_size,
                 "8 s high-follow: @100 Hz set_joint_target + "
                 "flush_joint_command; L %s, R %s.",
                 left_desc, right_desc);
    } else if (strcmp(name, "all") == 0) {
        snprintf(out, out_size,
                 "8 s high-follow: @100 Hz Group::ALL; L %s, R %s in one "
                 "packet.",
                 left_desc, right_desc);
    } else {
        if (use_left && use_right) {
            snprintf(out, out_size, "8 s high-follow: L %s; R %s.", left_desc,
                     right_desc);
        } else if (use_left) {
            snprintf(out, out_size, "8 s high-follow: L %s.", left_desc);
        } else if (use_right) {
            snprintf(out, out_size, "8 s high-follow: R %s.", right_desc);
        } else {
            snprintf(out, out_size, "8 s high-follow.");
        }
    }
}

static int enable_arm(dual_arm_control_api_t* api, dual_arm_group_t group,
                      int enable) {
    const dual_arm_ret_code_t code = dual_arm_set_group(api, group, enable);
    printf("set_group(%s, %s): %s\n", dual_arm_group_name(group),
           enable ? "enable" : "disable", dual_arm_ret_code_name(code));
    return code == DUAL_ARM_RET_SUCCESS;
}

static int configure_speed(dual_arm_control_api_t* api, dual_arm_group_t group,
                           size_t joint_count) {
    float speed[kMaxJoints];
    float applied[kMaxJoints];
    size_t applied_count = 0;
    for (size_t i = 0; i < joint_count; ++i) {
        speed[i] = kMaxSpeedDegS * kDegToRad;
    }
    const dual_arm_ret_code_t ret = dual_arm_set_joint_max_speed(
        api, group, speed, joint_count, applied, kMaxJoints, &applied_count);
    if (ret != DUAL_ARM_RET_SUCCESS || applied_count != joint_count) {
        fprintf(stderr, "set_joint_max_speed(%s) failed\n",
                dual_arm_group_name(group));
        return 0;
    }
    return 1;
}

static void read_arm(dual_arm_control_api_t* api, ArmContext* arm) {
    size_t pos_count = 0;
    size_t vel_count = 0;
    size_t st_count = 0;
    dual_arm_state_t states[kMaxJoints];

    const dual_arm_ret_code_t pos_ret =
        dual_arm_get_joint(api, arm->group, arm->pos, kMaxJoints, &pos_count);
    const dual_arm_ret_code_t vel_ret = dual_arm_get_joint_velocity(
        api, arm->group, arm->vel, kMaxJoints, &vel_count);
    const dual_arm_ret_code_t st_ret = dual_arm_get_group_state(
        api, arm->group, states, kMaxJoints, &st_count);
    (void)st_ret;

    if (pos_ret != DUAL_ARM_RET_SUCCESS || vel_ret != DUAL_ARM_RET_SUCCESS ||
        st_ret != DUAL_ARM_RET_SUCCESS) {
        ++arm->stats.read_fail;
        return;
    }
    arm->joint_count = pos_count;
}

static void write_arm(dual_arm_control_api_t* api, ArmContext* arm) {
    const dual_arm_ret_code_t ret = dual_arm_move_joint(
        api, arm->group, arm->cmd, arm->joint_count, 1, kTrajectoryMode, kRadio);
    if (ret != DUAL_ARM_RET_SUCCESS) {
        ++arm->stats.write_fail;
    }
}

static int flush_coalesced_targets(dual_arm_control_api_t* api,
                                   const ArmContext* left,
                                   const ArmContext* right,
                                   ArmStats* left_stats, ArmStats* right_stats) {
    dual_arm_ret_code_t ret =
        dual_arm_set_joint_target(api, DUAL_ARM_GROUP_LEFT_ARM, left->cmd,
                                  left->joint_count);
    if (ret != DUAL_ARM_RET_SUCCESS) {
        ++left_stats->write_fail;
        ++right_stats->write_fail;
        return 0;
    }
    ret = dual_arm_set_joint_target(api, DUAL_ARM_GROUP_RIGHT_ARM, right->cmd,
                                    right->joint_count);
    if (ret != DUAL_ARM_RET_SUCCESS) {
        ++left_stats->write_fail;
        ++right_stats->write_fail;
        return 0;
    }
    ret = dual_arm_flush_joint_command(api, 1, kTrajectoryMode, kRadio);
    if (ret != DUAL_ARM_RET_SUCCESS) {
        ++left_stats->write_fail;
        ++right_stats->write_fail;
        return 0;
    }
    return 1;
}

static void update_stats(ArmContext* arm, float elapsed) {
    if (arm->joint_count == 0) {
        return;
    }

    const size_t n = arm->joint_count;
    Sample s;
    memset(&s, 0, sizeof(s));
    s.act_count = n;

    float sum_act_offset = 0.0F;
    float sum_lag = 0.0F;
    float sum_vel = 0.0F;
    for (size_t i = 0; i < n; ++i) {
        const float cmd_off = (arm->cmd[i] - arm->start[i]) * kRadToDeg;
        const float act_off = (arm->pos[i] - arm->start[i]) * kRadToDeg;
        s.act_offset_deg[i] = act_off;
        s.cmd_offset_deg = cmd_off;
        sum_act_offset += act_off;
        sum_lag += float_abs(cmd_off - act_off);
        if (i < arm->joint_count) {
            sum_vel += float_abs(arm->vel[i] * kRadToDeg);
        }
    }
    const float count = (float)n;
    s.mean_act_offset_deg = sum_act_offset / count;
    s.mean_lag_deg = sum_lag / count;
    s.mean_abs_vel_deg_s = sum_vel / count;
    arm->stats.last = s;

    const float target_90 = arm->amplitude_deg * kReachFraction;
    if (arm->stats.t_cmd_90 < 0.0F &&
        arm->stats.last.cmd_offset_deg >= target_90) {
        arm->stats.t_cmd_90 = elapsed;
    }
    if (arm->stats.t_act_90 < 0.0F &&
        arm->stats.last.mean_act_offset_deg >= target_90) {
        arm->stats.t_act_90 = elapsed;
    }

    if (elapsed < 0.3F) {
        return;
    }
    arm->stats.sum_abs_lag += arm->stats.last.mean_lag_deg;
    arm->stats.sum_abs_vel += arm->stats.last.mean_abs_vel_deg_s;
    arm->stats.max_abs_vel =
        float_max(arm->stats.max_abs_vel, arm->stats.last.mean_abs_vel_deg_s);
    arm->stats.max_abs_lag =
        float_max(arm->stats.max_abs_lag, arm->stats.last.mean_lag_deg);
    ++arm->stats.samples;
}

static int over_velocity(const ArmContext* arm) {
    for (size_t i = 0; i < arm->joint_count; ++i) {
        if (float_abs(arm->vel[i]) > kGuardVelocityDegS * kDegToRad) {
            return 1;
        }
    }
    return 0;
}

static float progress_ratio(const ArmContext* arm) {
    if (arm->amplitude_deg <= 1e-6F) {
        return 0.0F;
    }
    return arm->stats.last.mean_act_offset_deg / arm->amplitude_deg;
}

static void print_live_line(const char* stage, float elapsed, double cycle_ms,
                            const ArmContext* left, const ArmContext* right) {
    printf("[%s] t=%.2fs cycle=%.2fms\n", stage, elapsed, cycle_ms);

    const ArmContext* arms[2] = {left, right};
    for (int idx = 0; idx < 2; ++idx) {
        const ArmContext* arm = arms[idx];
        if (arm == NULL) {
            continue;
        }
        char target_label[128];
        format_arm_target_label(arm, target_label, sizeof(target_label));
        char deg_list[256];
        format_deg_list(arm->stats.last.act_offset_deg,
                        arm->stats.last.act_count, deg_list, sizeof(deg_list));
        printf("  %s %s  progress=%.2f%%  cmd=%.2f  act=%.2f  lag=%.2f  "
               "vel=%.2f\n      d=%s\n",
               arm->name, target_label, 100.0F * progress_ratio(arm),
               arm->stats.last.cmd_offset_deg, arm->stats.last.mean_act_offset_deg,
               arm->stats.last.mean_lag_deg, arm->stats.last.mean_abs_vel_deg_s,
               deg_list);
    }
}

static void print_summary_row(const ArmContext* arm) {
    if (arm == NULL) {
        return;
    }
    const ArmStats* s = &arm->stats;
    const float mean_lag =
        s->samples > 0 ? (float)(s->sum_abs_lag / s->samples) : 0.0F;
    const float mean_vel =
        s->samples > 0 ? (float)(s->sum_abs_vel / s->samples) : 0.0F;
    const float mean_cycle =
        s->cycles > 0 ? (float)(s->sum_cycle_ms / s->cycles) : 0.0F;
    const float final_progress = 100.0F * progress_ratio(arm);
    char target_label[128];
    char deg_list[256];
    format_arm_target_label(arm, target_label, sizeof(target_label));
    format_deg_list(s->last.act_offset_deg, s->last.act_count, deg_list,
                    sizeof(deg_list));
    printf("  %s %s  final_progress=%.2f%%  (mean act / max target; "
           "per-joint mode is approximate)\n"
           "       mean|lag|=%.2f deg  max|lag|=%.2f deg  mean|vel|=%.2f deg/s  "
           "peak|vel|=%.2f deg/s\n"
           "       t_cmd_90=%.2f s  t_act_90=%.2f s  cycle=%.2f ms  "
           "read_fail=%d  write_fail=%d\n"
           "       final d=%s\n",
           arm->name, target_label, final_progress, mean_lag, s->max_abs_lag,
           mean_vel, s->max_abs_vel, s->t_cmd_90, s->t_act_90, mean_cycle,
           s->read_fail, s->write_fail, deg_list);
}

static void print_summary(const char* stage, const ArmContext* left,
                          const ArmContext* right) {
    printf("\n======== %s summary (all joints) ========\n", stage);
    print_summary_row(left);
    print_summary_row(right);
    if (left != NULL && right != NULL && left->stats.samples > 0 &&
        right->stats.samples > 0) {
        const float left_progress = progress_ratio(left);
        const float right_progress = progress_ratio(right);
        const float progress_ratio_lr =
            right_progress > 1e-3F ? left_progress / right_progress : 0.0F;
        const float left_lag_norm =
            left->stats.max_abs_lag / float_max(left->amplitude_deg, 1e-3F);
        const float right_lag_norm =
            right->stats.max_abs_lag / float_max(right->amplitude_deg, 1e-3F);
        const float lag_ratio_lr =
            right_lag_norm > 1e-3F ? left_lag_norm / right_lag_norm : 0.0F;
        printf("  progress L/R = %.2f   (<< 1 means left finished less of its "
               "move)\n",
               progress_ratio_lr);
        printf("  lag/amp  L/R = %.2f   (>> 1 means left lags more than "
               "amplitude explains)\n",
               lag_ratio_lr);
    }
    printf("==========================================\n");
}

static int wait_until_pose(dual_arm_control_api_t* api, dual_arm_group_t group,
                           const float* target, size_t target_count) {
    const float kTol = 0.5F * kDegToRad;
    struct timespec deadline;
    clock_gettime(CLOCK_MONOTONIC, &deadline);
    timespec_add_ms(&deadline, kReturnTimeoutSeconds * 1000);
    int stable = 0;

    while (1) {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        if (now.tv_sec > deadline.tv_sec ||
            (now.tv_sec == deadline.tv_sec &&
             now.tv_nsec >= deadline.tv_nsec)) {
            break;
        }

        float pos[kMaxJoints];
        size_t count = 0;
        const dual_arm_ret_code_t code =
            dual_arm_get_joint(api, group, pos, kMaxJoints, &count);
        if (code == DUAL_ARM_RET_SUCCESS && count == target_count) {
            float max_err = 0.0F;
            for (size_t i = 0; i < count; ++i) {
                max_err = float_max(max_err, float_abs(pos[i] - target[i]));
            }
            if (max_err <= kTol) {
                if (++stable >= 8) {
                    return 1;
                }
            } else {
                stable = 0;
            }
        }
        sleep_ms(20);
    }
    return 0;
}

static int go_home_both(dual_arm_control_api_t* api, size_t left_count,
                        size_t right_count, const float* head_now,
                        size_t head_count) {
    printf("Both arms going home (all arm joints -> 0, head unchanged)...\n");

    float left_zero[kMaxJoints];
    float right_zero[kMaxJoints];
    for (size_t i = 0; i < left_count; ++i) {
        left_zero[i] = 0.0F;
    }
    for (size_t i = 0; i < right_count; ++i) {
        right_zero[i] = 0.0F;
    }

    float all[kMaxJoints];
    size_t all_count = 0;
    concat_three_groups(left_zero, left_count, head_now, head_count, right_zero,
                        right_count, all, kMaxJoints, &all_count);

    if (dual_arm_move_joint(api, DUAL_ARM_GROUP_ALL, all, all_count, 0, 0, 0) !=
        DUAL_ARM_RET_SUCCESS) {
        fprintf(stderr, "go-home move_joint(ALL) failed\n");
        return 0;
    }

    const int left_ok =
        wait_until_pose(api, DUAL_ARM_GROUP_LEFT_ARM, left_zero, left_count);
    const int right_ok =
        wait_until_pose(api, DUAL_ARM_GROUP_RIGHT_ARM, right_zero, right_count);
    if (!left_ok || !right_ok) {
        fprintf(stderr, "Timed out going home (left=%s, right=%s)\n",
                left_ok ? "ok" : "timeout", right_ok ? "ok" : "timeout");
        return 0;
    }
    printf("Home reached.\n");
    return 1;
}

static int hold_current(dual_arm_control_api_t* api, int use_left,
                        int use_right) {
    if (use_left) {
        float pos[kMaxJoints];
        size_t count = 0;
        if (dual_arm_get_joint(api, DUAL_ARM_GROUP_LEFT_ARM, pos, kMaxJoints,
                               &count) == DUAL_ARM_RET_SUCCESS) {
            dual_arm_move_joint(api, DUAL_ARM_GROUP_LEFT_ARM, pos, count, 1, 0,
                                0);
        }
    }
    if (use_right) {
        float pos[kMaxJoints];
        size_t count = 0;
        if (dual_arm_get_joint(api, DUAL_ARM_GROUP_RIGHT_ARM, pos, kMaxJoints,
                               &count) == DUAL_ARM_RET_SUCCESS) {
            dual_arm_move_joint(api, DUAL_ARM_GROUP_RIGHT_ARM, pos, count, 1, 0,
                                0);
        }
    }
    return 1;
}

static void fill_cmd(ArmContext* arm, float progress) {
    const float smooth = 0.5F - 0.5F * cosf(kPi * progress);
    if (arm->has_per_joint_amplitude) {
        offset_target_per_joint(arm->cmd, arm->start,
                                arm->amplitude_deg_per_joint, arm->joint_count,
                                smooth);
    } else {
        offset_target(arm->cmd, arm->start, arm->joint_count, arm->amplitude_deg,
                      smooth);
    }
}

static void* thread_worker(void* arg) {
    ThreadWorkerCtx* ctx = (ThreadWorkerCtx*)arg;
    if (ctx->delay_ms > 0) {
        sleep_ms(ctx->delay_ms);
    }

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    while (!*ctx->stop && !*ctx->safety) {
        struct timespec now_ts;
        clock_gettime(CLOCK_MONOTONIC, &now_ts);
        const float elapsed =
            (float)timespec_diff_seconds(&now_ts, &ctx->start_time);
        if (elapsed >= kDurationSeconds) {
            break;
        }
        float progress = elapsed / kDurationSeconds;
        if (progress > 1.0F) {
            progress = 1.0F;
        }

        struct timespec cycle_begin = now_ts;
        read_arm(ctx->api, ctx->arm);
        if (over_velocity(ctx->arm)) {
            *ctx->safety = 1;
            break;
        }
        fill_cmd(ctx->arm, progress);
        write_arm(ctx->api, ctx->arm);
        update_stats(ctx->arm, elapsed);

        struct timespec cycle_end;
        clock_gettime(CLOCK_MONOTONIC, &cycle_end);
        ctx->arm->stats.sum_cycle_ms +=
            timespec_diff_ms(&cycle_end, &cycle_begin);
        ++ctx->arm->stats.cycles;

        timespec_add_ms(&next, ctx->thread_period_ms);
        sleep_until_monotonic(&next);
    }
    return NULL;
}

static int run_stage(dual_arm_control_api_t* api, const char* stage,
                     DriveMode mode, int use_left, int use_right,
                     const float* left_start, size_t left_count,
                     const float* right_start, size_t right_count,
                     const float* head_now, size_t head_count,
                     const float* left_offsets_deg, size_t left_offset_count,
                     const float* right_offsets_deg, size_t right_offset_count) {
    ArmContext left;
    ArmContext right;
    memset(&left, 0, sizeof(left));
    memset(&right, 0, sizeof(right));

    left.group = DUAL_ARM_GROUP_LEFT_ARM;
    left.name = "L";
    left.joint_count = left_count;
    memcpy(left.start, left_start, left_count * sizeof(float));
    memcpy(left.cmd, left_start, left_count * sizeof(float));
    if (use_left) {
        apply_offsets_to_arm(&left, left_offsets_deg, left_offset_count);
    }

    right.group = DUAL_ARM_GROUP_RIGHT_ARM;
    right.name = "R";
    right.joint_count = right_count;
    memcpy(right.start, right_start, right_count * sizeof(float));
    memcpy(right.cmd, right_start, right_count * sizeof(float));
    if (use_right) {
        apply_offsets_to_arm(&right, right_offsets_deg, right_offset_count);
    }

    left.stats.t_cmd_90 = -1.0F;
    left.stats.t_act_90 = -1.0F;
    right.stats.t_cmd_90 = -1.0F;
    right.stats.t_act_90 = -1.0F;

    volatile int stop = 0;
    volatile int safety = 0;
    pthread_mutex_t print_mutex = PTHREAD_MUTEX_INITIALIZER;
    int debug_tick = 0;

    char left_desc[128];
    char right_desc[128];
    format_offset_desc(left_offsets_deg, left_offset_count, left_desc,
                       sizeof(left_desc));
    format_offset_desc(right_offsets_deg, right_offset_count, right_desc,
                       sizeof(right_desc));

    printf("\n==> %s  %.1f s,", stage, kDurationSeconds);
    if (use_left) {
        printf(" L %s,", left_desc);
    } else {
        printf(" L held,");
    }
    if (use_right) {
        printf(" R %s,", right_desc);
    } else {
        printf(" R held,");
    }
    printf(" follow=true mode=%u radio=%u\n", (unsigned)kTrajectoryMode,
           (unsigned)kRadio);

    if (mode == DRIVE_SEQUENTIAL && use_left && use_right) {
        printf("  Write pattern: left, wait 10ms, right, wait 10ms.\n"
               "  rt_control only receives 1 UDP packet / 10ms; this gap "
               "lets both packets be applied.\n"
               "  Each single-arm move_joint() still hold_unselected_groups() "
               "the other arm to current pose.\n");
    } else if (mode == DRIVE_SEQUENTIAL_BURST && use_left && use_right) {
        printf("  Write pattern: left then right in the same 0.1ms "
               "(BURST). The second packet is usually dropped, so only "
               "the first arm moves.\n");
    } else if (mode == DRIVE_THREADED) {
        printf("  Two threads, 50 Hz each, right delayed by 10ms.\n");
    } else if (mode == DRIVE_COALESCED_FLUSH) {
        printf("  Each 10 ms: set_joint_target(L), set_joint_target(R), "
               "flush_joint_command().\n"
               "  Unselected groups (head) held from feedback at flush.\n");
    } else if (mode == DRIVE_GROUP_ALL) {
        printf("  One move_joint(ALL) per 10ms; both arm targets in the "
               "same packet.\n");
    }

    if (mode == DRIVE_COALESCED_FLUSH) {
        dual_arm_reset_joint_target_cache(api);
    }

    struct timespec start_time;
    clock_gettime(CLOCK_MONOTONIC, &start_time);
    struct timespec next_wakeup = start_time;
    const int thread_period_ms =
        (mode == DRIVE_THREADED) ? (2 * kPeriodMs) : kPeriodMs;

    if (mode == DRIVE_THREADED && use_left && use_right) {
        ThreadWorkerCtx left_ctx = {api, &left, start_time, 0,
                                    thread_period_ms, &stop, &safety};
        ThreadWorkerCtx right_ctx = {api, &right, start_time, kPeriodMs,
                                     thread_period_ms, &stop, &safety};
        pthread_t left_thread;
        pthread_t right_thread;
        pthread_create(&left_thread, NULL, thread_worker, &left_ctx);
        pthread_create(&right_thread, NULL, thread_worker, &right_ctx);

        while (!safety) {
            struct timespec now_ts;
            clock_gettime(CLOCK_MONOTONIC, &now_ts);
            const float elapsed =
                (float)timespec_diff_seconds(&now_ts, &start_time);
            if (elapsed >= kDurationSeconds) {
                break;
            }
            if (++debug_tick >= kDebugEveryN) {
                debug_tick = 0;
                pthread_mutex_lock(&print_mutex);
                print_live_line(stage, elapsed, (double)thread_period_ms, &left,
                                &right);
                pthread_mutex_unlock(&print_mutex);
            }
            sleep_ms(kPeriodMs);
        }
        stop = 1;
        pthread_join(left_thread, NULL);
        pthread_join(right_thread, NULL);
    } else {
        while (!safety) {
            struct timespec now_ts;
            clock_gettime(CLOCK_MONOTONIC, &now_ts);
            const float elapsed =
                (float)timespec_diff_seconds(&now_ts, &start_time);
            if (elapsed >= kDurationSeconds) {
                break;
            }
            float progress = elapsed / kDurationSeconds;
            if (progress > 1.0F) {
                progress = 1.0F;
            }
            struct timespec cycle_begin = now_ts;

            if (mode == DRIVE_COALESCED_FLUSH && use_left && use_right) {
                read_arm(api, &left);
                read_arm(api, &right);
                if (over_velocity(&left) || over_velocity(&right)) {
                    safety = 1;
                    break;
                }
                fill_cmd(&left, progress);
                fill_cmd(&right, progress);
                flush_coalesced_targets(api, &left, &right, &left.stats,
                                        &right.stats);
                update_stats(&left, elapsed);
                update_stats(&right, elapsed);

                struct timespec cycle_end;
                clock_gettime(CLOCK_MONOTONIC, &cycle_end);
                const double cycle_ms =
                    timespec_diff_ms(&cycle_end, &cycle_begin);
                left.stats.sum_cycle_ms += cycle_ms;
                right.stats.sum_cycle_ms += cycle_ms;
                ++left.stats.cycles;
                ++right.stats.cycles;
                if (++debug_tick >= kDebugEveryN) {
                    debug_tick = 0;
                    print_live_line(stage, elapsed, cycle_ms, &left, &right);
                }
                timespec_add_ms(&next_wakeup, kPeriodMs);
                sleep_until_monotonic(&next_wakeup);
            } else if (mode == DRIVE_GROUP_ALL && use_left && use_right) {
                read_arm(api, &left);
                read_arm(api, &right);
                if (over_velocity(&left) || over_velocity(&right)) {
                    safety = 1;
                    break;
                }
                fill_cmd(&left, progress);
                fill_cmd(&right, progress);

                float all[kMaxJoints];
                size_t all_count = 0;
                concat_three_groups(left.cmd, left.joint_count, head_now,
                                    head_count, right.cmd, right.joint_count,
                                    all, kMaxJoints, &all_count);
                const dual_arm_ret_code_t ret = dual_arm_move_joint(
                    api, DUAL_ARM_GROUP_ALL, all, all_count, 1, kTrajectoryMode,
                    kRadio);
                if (ret != DUAL_ARM_RET_SUCCESS) {
                    ++left.stats.write_fail;
                    ++right.stats.write_fail;
                }
                update_stats(&left, elapsed);
                update_stats(&right, elapsed);

                struct timespec cycle_end;
                clock_gettime(CLOCK_MONOTONIC, &cycle_end);
                const double cycle_ms =
                    timespec_diff_ms(&cycle_end, &cycle_begin);
                if (use_left) {
                    left.stats.sum_cycle_ms += cycle_ms;
                    ++left.stats.cycles;
                }
                if (use_right) {
                    right.stats.sum_cycle_ms += cycle_ms;
                    ++right.stats.cycles;
                }
                if (++debug_tick >= kDebugEveryN) {
                    debug_tick = 0;
                    print_live_line(stage, elapsed, (double)kPeriodMs, &left,
                                    &right);
                }
                timespec_add_ms(&next_wakeup, kPeriodMs);
                sleep_until_monotonic(&next_wakeup);
            } else if (use_left && use_right && mode == DRIVE_SEQUENTIAL) {
                read_arm(api, &left);
                read_arm(api, &right);
                if (over_velocity(&left) || over_velocity(&right)) {
                    safety = 1;
                    break;
                }
                fill_cmd(&left, progress);
                write_arm(api, &left);
                update_stats(&left, elapsed);

                struct timespec left_end;
                clock_gettime(CLOCK_MONOTONIC, &left_end);
                left.stats.sum_cycle_ms +=
                    timespec_diff_ms(&left_end, &cycle_begin);
                ++left.stats.cycles;

                timespec_add_ms(&next_wakeup, kPeriodMs);
                sleep_until_monotonic(&next_wakeup);
                if (safety) {
                    break;
                }

                struct timespec right_now_ts;
                clock_gettime(CLOCK_MONOTONIC, &right_now_ts);
                const float right_elapsed =
                    (float)timespec_diff_seconds(&right_now_ts, &start_time);
                if (right_elapsed >= kDurationSeconds) {
                    break;
                }
                float right_progress = right_elapsed / kDurationSeconds;
                if (right_progress > 1.0F) {
                    right_progress = 1.0F;
                }
                struct timespec right_cycle_begin = right_now_ts;
                read_arm(api, &left);
                read_arm(api, &right);
                if (over_velocity(&left) || over_velocity(&right)) {
                    safety = 1;
                    break;
                }
                fill_cmd(&right, right_progress);
                write_arm(api, &right);
                update_stats(&left, right_elapsed);
                update_stats(&right, right_elapsed);

                struct timespec right_end;
                clock_gettime(CLOCK_MONOTONIC, &right_end);
                right.stats.sum_cycle_ms +=
                    timespec_diff_ms(&right_end, &right_cycle_begin);
                ++right.stats.cycles;

                if (++debug_tick >= (kDebugEveryN / 2)) {
                    debug_tick = 0;
                    print_live_line(stage, right_elapsed, (double)(2 * kPeriodMs),
                                    &left, &right);
                }
                timespec_add_ms(&next_wakeup, kPeriodMs);
                sleep_until_monotonic(&next_wakeup);
            } else {
                if (use_left) {
                    read_arm(api, &left);
                }
                if (use_right) {
                    read_arm(api, &right);
                }
                if ((use_left && over_velocity(&left)) ||
                    (use_right && over_velocity(&right))) {
                    safety = 1;
                    break;
                }
                if (use_left) {
                    fill_cmd(&left, progress);
                    write_arm(api, &left);
                    update_stats(&left, elapsed);
                }
                if (use_right) {
                    fill_cmd(&right, progress);
                    write_arm(api, &right);
                    update_stats(&right, elapsed);
                }

                struct timespec cycle_end;
                clock_gettime(CLOCK_MONOTONIC, &cycle_end);
                const double cycle_ms =
                    timespec_diff_ms(&cycle_end, &cycle_begin);
                if (use_left) {
                    left.stats.sum_cycle_ms += cycle_ms;
                    ++left.stats.cycles;
                }
                if (use_right) {
                    right.stats.sum_cycle_ms += cycle_ms;
                    ++right.stats.cycles;
                }
                if (++debug_tick >= kDebugEveryN) {
                    debug_tick = 0;
                    print_live_line(stage, elapsed, cycle_ms,
                                    use_left ? &left : NULL,
                                    use_right ? &right : NULL);
                }
                timespec_add_ms(&next_wakeup, kPeriodMs);
                sleep_until_monotonic(&next_wakeup);
            }
        }
    }

    if (safety) {
        fprintf(stderr,
                "\nSAFETY STOP: joint velocity exceeded %.0f deg/s. Holding "
                "current pose.\n",
                kGuardVelocityDegS);
        hold_current(api, use_left, use_right);
        print_summary(stage, use_left ? &left : NULL, use_right ? &right : NULL);
        return 0;
    }

    print_summary(stage, use_left ? &left : NULL, use_right ? &right : NULL);

    printf("Hold %ds at the offset pose, then both arms go home together.\n",
           kHoldAtTargetSeconds);
    sleep_ms(kHoldAtTargetSeconds * 1000);
    return go_home_both(api, left_count, right_count, head_now, head_count);
}

int main(int argc, char** argv) {
    ParsedArgs args;
    char parse_err[kMaxErrLen];
    if (!parse_args(argc, argv, &args, parse_err, sizeof(parse_err))) {
        fprintf(stderr, "%s\n", parse_err);
        print_usage(argv[0]);
        return 1;
    }
    if (args.show_help) {
        print_usage(argv[0]);
        return 0;
    }

    const int interactive = !args.has_mode;
    char stages[kMaxStages][kMaxModeLen];
    int stage_count = 0;

    if (interactive) {
        snprintf(stages[stage_count++], kMaxModeLen, "left");
        snprintf(stages[stage_count++], kMaxModeLen, "right");
        snprintf(stages[stage_count++], kMaxModeLen, "both");
    } else if (strcmp(args.mode, "left") == 0 ||
               strcmp(args.mode, "right") == 0 ||
               strcmp(args.mode, "both") == 0 ||
               strcmp(args.mode, "both-burst") == 0 ||
               strcmp(args.mode, "both-thread") == 0 ||
               strcmp(args.mode, "both-flush") == 0 ||
               strcmp(args.mode, "all") == 0) {
        snprintf(stages[stage_count++], kMaxModeLen, "%s", args.mode);
    } else {
        fprintf(stderr, "Unknown mode '%s'\n", args.mode);
        print_usage(argv[0]);
        return 1;
    }

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

    printf("Waiting for rt_control state...\n");
    if (!wait_for_joint_state(api, DUAL_ARM_GROUP_LEFT_ARM, left_now, kMaxJoints,
                              &left_count, kDefaultStateTimeoutMs) ||
        !wait_for_joint_state(api, DUAL_ARM_GROUP_RIGHT_ARM, right_now,
                              kMaxJoints, &right_count,
                              kDefaultStateTimeoutMs) ||
        !wait_for_joint_state(api, DUAL_ARM_GROUP_HEAD, head_now, kMaxJoints,
                              &head_count, kDefaultStateTimeoutMs)) {
        fprintf(stderr, "No rt_control state within 3 seconds.\n");
        dual_arm_control_api_destroy(api);
        return 1;
    }

    printf("Current pose:\n");
    print_pose("LEFT ", left_now, left_count);
    print_pose("RIGHT", right_now, right_count);

    if (args.targets.has_left) {
        char desc[128];
        format_offset_desc(args.targets.left_deg, args.targets.left_count, desc,
                           sizeof(desc));
        printf("Left target override: %s\n", desc);
    }
    if (args.targets.has_right) {
        char desc[128];
        format_offset_desc(args.targets.right_deg, args.targets.right_count,
                           desc, sizeof(desc));
        printf("Right target override: %s\n", desc);
    }

    if (!enable_arm(api, DUAL_ARM_GROUP_LEFT_ARM, 1) ||
        !enable_arm(api, DUAL_ARM_GROUP_RIGHT_ARM, 1)) {
        dual_arm_control_api_destroy(api);
        return 1;
    }
    if (!configure_speed(api, DUAL_ARM_GROUP_LEFT_ARM, left_count) ||
        !configure_speed(api, DUAL_ARM_GROUP_RIGHT_ARM, right_count)) {
        dual_arm_control_api_destroy(api);
        return 1;
    }

    if (!wait_for_enter("First go home (both arms to 0) for a known start pose "
                        "before the high-follow test.")) {
        dual_arm_control_api_destroy(api);
        return 0;
    }
    if (!go_home_both(api, left_count, right_count, head_now, head_count)) {
        dual_arm_control_api_destroy(api);
        return 1;
    }

    for (int i = 0; i < stage_count; ++i) {
        const char* name = stages[i];
        int use_left = 0;
        int use_right = 0;
        DriveMode mode = DRIVE_SEQUENTIAL;
        const char* label = name;

        if (strcmp(name, "left") == 0) {
            use_left = 1;
            label = "LEFT only";
        } else if (strcmp(name, "right") == 0) {
            use_right = 1;
            label = "RIGHT only";
        } else if (strcmp(name, "both") == 0) {
            use_left = 1;
            use_right = 1;
            label = "BOTH staggered (left, 10ms, right, 10ms)";
        } else if (strcmp(name, "both-burst") == 0) {
            use_left = 1;
            use_right = 1;
            mode = DRIVE_SEQUENTIAL_BURST;
            label = "BOTH burst (left+right in 0.1ms, second packet dropped)";
        } else if (strcmp(name, "both-thread") == 0) {
            use_left = 1;
            use_right = 1;
            mode = DRIVE_THREADED;
            label = "BOTH threaded (50 Hz each, right delayed 10ms)";
        } else if (strcmp(name, "both-flush") == 0) {
            use_left = 1;
            use_right = 1;
            mode = DRIVE_COALESCED_FLUSH;
            label = "BOTH coalesced (set_joint_target + flush @100 Hz)";
        } else if (strcmp(name, "all") == 0) {
            use_left = 1;
            use_right = 1;
            mode = DRIVE_GROUP_ALL;
            label = "BOTH via Group::ALL (control)";
        }

        float left_offsets[kMaxJoints];
        float right_offsets[kMaxJoints];
        size_t left_offset_count = 0;
        size_t right_offset_count = 0;
        float left_default[kMaxJoints];
        float right_default[kMaxJoints];
        size_t left_default_count = 0;
        size_t right_default_count = 0;

        default_left_offsets_deg(name, left_default, &left_default_count);
        default_right_offsets_deg(name, right_default, &right_default_count);

        if (use_left &&
            !resolve_offsets(args.targets.has_left, args.targets.left_deg,
                             args.targets.left_count, left_default,
                             left_default_count, left_count, left_offsets,
                             &left_offset_count, parse_err, sizeof(parse_err),
                             "left")) {
            fprintf(stderr, "%s\n", parse_err);
            dual_arm_control_api_destroy(api);
            return 1;
        }
        if (use_right &&
            !resolve_offsets(args.targets.has_right, args.targets.right_deg,
                             args.targets.right_count, right_default,
                             right_default_count, right_count, right_offsets,
                             &right_offset_count, parse_err, sizeof(parse_err),
                             "right")) {
            fprintf(stderr, "%s\n", parse_err);
            dual_arm_control_api_destroy(api);
            return 1;
        }

        char motion_desc[kMaxPromptLen];
        describe_stage_motion(name, use_left, use_right, left_offsets,
                              left_offset_count, right_offsets,
                              right_offset_count, motion_desc,
                              sizeof(motion_desc));
        printf("\nStage %d/%d: %s\n%s\nThen hold 2 s and both arms go home "
               "together.",
               i + 1, stage_count, label, motion_desc);
        if (!wait_for_enter(NULL)) {
            dual_arm_control_api_destroy(api);
            return 0;
        }

        if (!wait_for_joint_state(api, DUAL_ARM_GROUP_LEFT_ARM, left_now,
                                  kMaxJoints, &left_count,
                                  kDefaultStateTimeoutMs) ||
            !wait_for_joint_state(api, DUAL_ARM_GROUP_RIGHT_ARM, right_now,
                                  kMaxJoints, &right_count,
                                  kDefaultStateTimeoutMs) ||
            !wait_for_joint_state(api, DUAL_ARM_GROUP_HEAD, head_now, kMaxJoints,
                                  &head_count, kDefaultStateTimeoutMs)) {
            fprintf(stderr, "Failed to refresh start pose.\n");
            dual_arm_control_api_destroy(api);
            return 1;
        }

        if (!run_stage(api, label, mode, use_left, use_right, left_now,
                       left_count, right_now, right_count, head_now, head_count,
                       left_offsets, left_offset_count, right_offsets,
                       right_offset_count)) {
            dual_arm_control_api_destroy(api);
            return 1;
        }
    }

    printf("\nDone. Watch live `progress` / per-joint `d=` during each stage.\n"
           "Use --left / --right to override per-arm offset targets (deg).\n"
           "Mode reference:\n"
           "  left        : left arm only; default L +[10..70] deg.\n"
           "  right       : right arm only; default R +20 deg/joint.\n"
           "  both        : staggered packets; left slows (hold overwrite).\n"
           "  both-burst  : second packet often dropped; first arm tracks.\n"
           "  both-thread : two threads @50 Hz each, right delayed 10 ms.\n"
           "  both-flush  : set_joint_target + flush @100 Hz (SDK coalesce).\n"
           "  all         : both arms sync via Group::ALL @100 Hz.\n");

    dual_arm_control_api_destroy(api);
    return 0;
}
