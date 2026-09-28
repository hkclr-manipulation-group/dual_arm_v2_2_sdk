/**
 * @file test_single_joint_increment.c
 * @brief 单臂单关节增量运动，用于校验 config_left/right.yaml 中 sign 方向是否正确。
 *
 * 前置条件：rt_control 已启动。
 *
 * 用法（在 build/bin/ 目录下）：
 *   ./test_single_joint_increment left 1 5
 *   ./test_single_joint_increment right 3 -3
 *   ./test_single_joint_increment left 2 10 --return
 *   ./test_single_joint_increment /path/to/cuarm_configuration/dual_v2_2 left 1 5
 *
 * sign 配置文件（修改后需重启 rt_control）：
 *   cuarm_rt_control/hardware_modules/realman_dual_arm/config_left.yaml
 *   cuarm_rt_control/hardware_modules/realman_dual_arm/config_right.yaml
 */

#include "dual_arm_control_api.h"
#include "example_common.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const float kMaxSpeedDegS = 10.0F;
static const float kPositionToleranceDeg = 0.5F;
static const int kReachTimeoutSeconds = 20;

typedef struct {
    char config_path[kMaxConfigPathLen];
    dual_arm_group_t arm;
    int joint_index; /* 0-based */
    float delta_deg;
    int return_to_start;
} options_t;

static void print_usage(const char* prog) {
    fprintf(stderr,
            "Usage:\n"
            "  %s <left|right> <joint 1-7> <delta_deg> [--return]\n"
            "  %s <config_prefix> <left|right> <joint 1-7> <delta_deg> "
            "[--return]\n\n"
            "Examples:\n"
            "  %s left 1 5       # left arm J1 +5 deg\n"
            "  %s right 4 -3     # right arm J4 -3 deg\n"
            "  %s left 2 10 --return\n",
            prog, prog, prog, prog, prog);
}

static int looks_like_config_prefix(const char* path) {
    char config_file[kMaxConfigPathLen];
    if (!join_path(config_file, sizeof(config_file), path, "config.yaml")) {
        return 0;
    }
    return path_exists(config_file);
}

static int parse_arm_token(const char* token, dual_arm_group_t* arm) {
    if (strcmp(token, "left") == 0 || strcmp(token, "L") == 0 ||
        strcmp(token, "l") == 0) {
        *arm = DUAL_ARM_GROUP_LEFT_ARM;
        return 1;
    }
    if (strcmp(token, "right") == 0 || strcmp(token, "R") == 0 ||
        strcmp(token, "r") == 0) {
        *arm = DUAL_ARM_GROUP_RIGHT_ARM;
        return 1;
    }
    return 0;
}

static int parse_options(int argc, char* argv[], options_t* opts) {
    const char* positional[4];
    int positional_count = 0;

    opts->arm = DUAL_ARM_GROUP_LEFT_ARM;
    opts->joint_index = 0;
    opts->delta_deg = 0.0F;
    opts->return_to_start = 0;

    for (int i = 1; i < argc; ++i) {
        const char* arg = argv[i];
        if (strcmp(arg, "--return") == 0) {
            opts->return_to_start = 1;
            continue;
        }
        if (strcmp(arg, "-h") == 0 || strcmp(arg, "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        }
        if (positional_count >= 4) {
            print_usage(argv[0]);
            return 0;
        }
        positional[positional_count++] = arg;
    }

    if (positional_count == 3) {
        config_path_from_args(argc, argv, opts->config_path,
                              sizeof(opts->config_path));
    } else if (positional_count == 4) {
        if (!looks_like_config_prefix(positional[0])) {
            fprintf(stderr, "Config prefix not found: %s\n", positional[0]);
            return 0;
        }
        snprintf(opts->config_path, sizeof(opts->config_path), "%s",
                 positional[0]);
        positional[0] = positional[1];
        positional[1] = positional[2];
        positional[2] = positional[3];
        positional_count = 3;
    } else {
        print_usage(argv[0]);
        return 0;
    }

    if (!parse_arm_token(positional[0], &opts->arm)) {
        fprintf(stderr, "Invalid arm: %s (expected left or right)\n",
                positional[0]);
        return 0;
    }

    opts->joint_index = atoi(positional[1]) - 1;
    if (opts->joint_index < 0 || opts->joint_index > 6) {
        fprintf(stderr, "Invalid joint: %s (expected 1..7)\n", positional[1]);
        return 0;
    }

    opts->delta_deg = strtof(positional[2], NULL);
    if (opts->delta_deg == 0.0F) {
        fprintf(stderr, "Warning: delta_deg is 0; the arm will not move.\n");
    }
    return 1;
}

static void print_joint_deg(const char* label, const float* joints_rad,
                            size_t count) {
    printf("%s (deg): [", label);
    for (size_t i = 0; i < count; ++i) {
        printf("%.3f", joints_rad[i] * kRadToDeg);
        if (i + 1 < count) {
            printf(", ");
        }
    }
    printf("]\n");
}

static int wait_until_joint_reached(dual_arm_control_api_t* api,
                                    dual_arm_group_t arm,
                                    const float* target, size_t count) {
    const float tolerance_rad = kPositionToleranceDeg * kDegToRad;
    int elapsed_ms = 0;
    int stable = 0;

    while (elapsed_ms <= kReachTimeoutSeconds * 1000) {
        float current[kMaxJoints];
        size_t current_count = 0;
        const dual_arm_ret_code_t code =
            dual_arm_get_joint(api, arm, current, kMaxJoints, &current_count);
        if (code != DUAL_ARM_RET_SUCCESS || current_count != count) {
            sleep_ms(100);
            elapsed_ms += 100;
            continue;
        }

        float max_error = max_joint_error(target, current, count);
        if (max_error <= tolerance_rad) {
            if (++stable >= 3) {
                return 1;
            }
        } else {
            stable = 0;
        }
        sleep_ms(100);
        elapsed_ms += 100;
    }
    return 0;
}

static int move_single_joint(dual_arm_control_api_t* api, dual_arm_group_t arm,
                             int joint_index, const float* start_rad,
                             size_t count, float delta_deg,
                             const char* step_label) {
    float target[kMaxJoints];
    float min_limit[kMaxJoints];
    float max_limit[kMaxJoints];
    size_t min_count = 0;
    size_t max_count = 0;

    if (count > kMaxJoints) {
        return 0;
    }
    memcpy(target, start_rad, count * sizeof(float));
    target[(size_t)joint_index] += delta_deg * kDegToRad;

    if (dual_arm_get_joint_min_limit(api, arm, min_limit, kMaxJoints,
                                     &min_count) == DUAL_ARM_RET_SUCCESS &&
        dual_arm_get_joint_max_limit(api, arm, max_limit, kMaxJoints,
                                     &max_count) == DUAL_ARM_RET_SUCCESS &&
        min_count == count && max_count == count) {
        const float q = target[(size_t)joint_index];
        const float q_min = min_limit[(size_t)joint_index];
        const float q_max = max_limit[(size_t)joint_index];
        if (q < q_min || q > q_max) {
            fprintf(stderr,
                    "%s: target J%d = %.3f deg is outside soft limit [%.3f, "
                    "%.3f] deg\n",
                    step_label, joint_index + 1, q * kRadToDeg,
                    q_min * kRadToDeg, q_max * kRadToDeg);
            return 0;
        }
    }

    printf("\n==> %s\n", step_label);
    printf("    %s J%d command delta: %+.3f deg\n",
           dual_arm_group_name(arm), joint_index + 1, delta_deg);

    if (dual_arm_move_joint(api, arm, target, count, 0, 0, 0) !=
        DUAL_ARM_RET_SUCCESS) {
        fprintf(stderr, "move_joint failed\n");
        return 0;
    }

    if (!wait_until_joint_reached(api, arm, target, count)) {
        printf("    WARNING: timed out after %d s waiting to settle\n",
               kReachTimeoutSeconds);
    }

    float actual[kMaxJoints];
    size_t actual_count = 0;
    if (!wait_for_joint_state(api, arm, actual, kMaxJoints, &actual_count,
                              kDefaultStateTimeoutMs)) {
        fprintf(stderr, "Failed to read joint feedback\n");
        return 0;
    }

    print_joint_deg("    target", target, count);
    print_joint_deg("    actual", actual, actual_count);

    const float cmd_delta =
        (target[(size_t)joint_index] - start_rad[(size_t)joint_index]) *
        kRadToDeg;
    const float fb_delta =
        (actual[(size_t)joint_index] - start_rad[(size_t)joint_index]) *
        kRadToDeg;
    printf("    J%d feedback delta: %+.3f deg (command %+.3f deg)\n",
           joint_index + 1, fb_delta, cmd_delta);
    return 1;
}

static void print_sign_hint(dual_arm_group_t arm, int joint_index) {
    const char* yaml = (arm == DUAL_ARM_GROUP_LEFT_ARM) ? "config_left.yaml"
                                                        : "config_right.yaml";
    printf("\nSign 校验提示:\n");
    printf("  - SDK 下发用户域 (MDH) 角度；rt_control 用 %s 中 "
           "hardware.motor.sign 换算电机方向\n",
           yaml);
    printf("  - 观察 J%d 物理转动是否与命令增量同号；若相反，将该关节 sign "
           "取反并重启 rt_control\n",
           joint_index + 1);
    printf("  - 左臂 sign 参考: [1, 1, 1, -1, 1, -1, 1]\n");
    printf("  - 右臂 sign 参考: [-1, 1, -1, -1, -1, 1, -1]\n");
}

int main(int argc, char* argv[]) {
    options_t opts;
    if (!parse_options(argc, argv, &opts)) {
        return argc > 1 ? 1 : 0;
    }

    dual_arm_control_api_t* api =
        dual_arm_control_api_create(opts.config_path);
    if (api == NULL) {
        fprintf(stderr, "Failed to create API: %s\n",
                dual_arm_control_api_last_error(api));
        return 1;
    }

    float start[kMaxJoints];
    size_t start_count = 0;
    if (!wait_for_joint_state(api, opts.arm, start, kMaxJoints, &start_count,
                              kDefaultStateTimeoutMs)) {
        fprintf(stderr,
                "No state received within 3 s. Is rt_control running?\n");
        dual_arm_control_api_destroy(api);
        return 1;
    }
    if ((size_t)opts.joint_index >= start_count) {
        fprintf(stderr, "Unexpected joint count: %zu\n", start_count);
        dual_arm_control_api_destroy(api);
        return 1;
    }

    if (dual_arm_set_group(api, opts.arm, 1) != DUAL_ARM_RET_SUCCESS) {
        fprintf(stderr, "Failed to enable %s\n",
                dual_arm_group_name(opts.arm));
        dual_arm_control_api_destroy(api);
        return 1;
    }

    float speed[kMaxJoints];
    float out_speed[kMaxJoints];
    size_t out_count = 0;
    for (size_t i = 0; i < start_count; ++i) {
        speed[i] = kMaxSpeedDegS * kDegToRad;
    }
    if (dual_arm_set_joint_max_speed(api, opts.arm, speed, start_count,
                                     out_speed, kMaxJoints,
                                     &out_count) != DUAL_ARM_RET_SUCCESS ||
        out_count != start_count) {
        fprintf(stderr, "set_joint_max_speed failed\n");
        dual_arm_control_api_destroy(api);
        return 1;
    }

    printf("Config: %s\n", opts.config_path);
    printf("Arm: %s, joint J%d, delta %+.3f deg, max speed %.1f deg/s\n",
           dual_arm_group_name(opts.arm), opts.joint_index + 1, opts.delta_deg,
           kMaxSpeedDegS);
    print_joint_deg("Start", start, start_count);

    if (!move_single_joint(api, opts.arm, opts.joint_index, start, start_count,
                           opts.delta_deg, "increment move")) {
        dual_arm_control_api_destroy(api);
        return 1;
    }

    print_sign_hint(opts.arm, opts.joint_index);

    if (opts.return_to_start) {
        if (!move_single_joint(api, opts.arm, opts.joint_index, start,
                               start_count, -opts.delta_deg,
                               "return to start")) {
            dual_arm_control_api_destroy(api);
            return 1;
        }
    }

    dual_arm_control_api_destroy(api);
    return 0;
}
